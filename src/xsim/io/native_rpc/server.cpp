#include "server.hpp"
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <poll.h>
#include <sstream>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <unordered_set>
namespace xsim {
namespace {
Json result_json(const Result &r) {
  return {{"applied", r.applied},
          {"success", r.success},
          {"reason", r.reason},
          {"entity_id", r.key.id},
          {"generation", r.key.generation},
          {"enabled", r.enabled},
          {"step", r.step},
          {"simulation_time_ns", r.stamp}};
}
const char *kind_name(Kind k) {
  return k == Kind::FS150 ? "fs150" : k == Kind::Scout ? "scout" : "mecanum";
}
} // namespace

Server::Server(const Json &config, const std::string &path, World &world,
               Sensors &sensors, RuntimeIO io)
    : instance_(config.at("instance_id").get<std::string>()),
      io_(std::move(io)), world_(world), sensors_(sensors) {
  if (instance_.empty())
    throw std::invalid_argument("instance_id required");
  if (path.size() >= sizeof(sockaddr_un::sun_path))
    throw std::invalid_argument("Unix socket path too long");
  // Do not unlink somebody else's or a previous process's socket implicitly.
  socket_.fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_.fd < 0)
    throw std::runtime_error("socket failed");
  sockaddr_un a{};
  a.sun_family = AF_UNIX;
  std::strcpy(a.sun_path, path.c_str());
  if (bind(socket_.fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) < 0) {
    close(socket_.fd);
    socket_.fd = -1;
    throw std::runtime_error("socket bind failed (path must be absent)");
  }
  socket_.path = path;
  chmod(path.c_str(), 0600);
  if (listen(socket_.fd, 32) < 0)
    throw std::runtime_error("socket listen failed");
  world_.sample_sensor = [&sensors, &world](const State &s) { sensors.submit(s, &world); };
  world_.metrics.paused = config.value("paused", false);
  input_poll_ns_=config.value("input_poll_ns",int64_t(1000000));
  if(input_poll_ns_<=0 || input_poll_ns_>std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::time_point::max()-Clock::now()).count())throw std::invalid_argument("input_poll_ns must be positive");
  for (const auto &j : config.value("entities", Json::array())) {
    auto t = std::make_shared<Command>();
    t->op = Op::Add;
    t->prepared = prepare(j);
    world_.submit(t);
    world_.boundary();
    if (!t->result.success)
      throw std::runtime_error("initial entity rejected");
  }
  latest_ = world_.capture();

}

Server::BoundSocket::~BoundSocket() {
  if (fd >= 0) close(fd);
  if (!path.empty()) unlink(path.c_str());
}

Server::~Server() { shutdown(); }

void Server::run(const volatile sig_atomic_t &stopping) {
  world_.start();
  preparing_ = true;
  for (unsigned i = 0; i < 2; ++i)
    service_workers_.emplace_back([this] { service_work(); });
  output_running_ = true;
  output_ = std::thread([this] { output(); });
  pthread_setname_np(pthread_self(), "xsim-input");
  auto next_input=Clock::now();
  while (!stopping && (!io_.okay || io_.okay())) {
    if(Clock::now()>=next_input){if (io_.poll_inputs) io_.poll_inputs();next_input=Clock::now()+std::chrono::nanoseconds(input_poll_ns_);}
    Frame view;
    {
      std::lock_guard<std::mutex> l(view_mutex_);
      view = latest_;
    }
    for (auto &s : view.states)
      if (auto e = s.entity.lock(); e && e->io && e->io->reconcile)
        e->io->reconcile();
    poll_once();
    for (auto i = requests_.begin(); i != requests_.end();) {
      auto &request = i->second;
      auto &t = request.ticket;
      if (request.preparation && !request.submitted) {
        auto &prep = *request.preparation;
        if (t->phase == 0 && Clock::now() >= t->deadline)
          t->phase = 3;
        if (prep.done.load(std::memory_order_acquire)) {
          if (t->phase == 0) {
            if (!prep.error.empty()) {
              t->result.reason = 5;
              t->phase = 4;
            } else {
              t->prepared = std::move(prep.value);
              world_.submit(t);
            }
          }
          prep.value.reset();
          request.submitted = true;
        }
      }
      if (t->phase == 2 && t->retired) {
        if (t->retired->io && t->retired->io->reconcile) {
          t->retired->io->reconcile();
        }
        t->retired.reset();
      }
      if (t->phase >= 2)
        t->prepared.reset();
      if (i->second.ticket->phase >= 2 && i->second.expiry < Clock::now())
        i = requests_.erase(i);
      else
        ++i;
    }
  }
  shutdown();
}

std::unique_ptr<Prepared> Server::prepare(const Json &j) {
  auto e = std::make_shared<Entity>(parse_entity(j));
  return prepare(e, j);
}

std::unique_ptr<Prepared> Server::prepare(const std::shared_ptr<Entity> &e, const Json &settings) {
  auto model = prepare_model(e->config);
  e->sensor = sensors_.prepare(e, settings.value("sensor", Json::object()));
  if (io_.attach_entity)
    io_.attach_entity(e, settings.value("ros", Json::object()));
  return std::make_unique<Prepared>(Prepared{e, std::move(model)});
}

void Server::shutdown() {
  world_.stop();
  preparing_ = false;
  preparation_wake_.notify_all();
  for (auto &worker : service_workers_)
    if (worker.joinable()) worker.join();
  service_workers_.clear();
  output_running_ = false;
  if (output_.joinable())
    output_.join();
  sensors_.stop();
  world_.sample_sensor = {};
  for (auto &c : clients_)
    close(c.fd);
  clients_.clear();
}

void Server::output() {
  pthread_setname_np(pthread_self(), "xsim-output");
  Frame frame;
  size_t sensor_cursor=0;
  while (output_running_) {
    if (world_.take_frame(frame)) {
      {
        std::lock_guard<std::mutex> l(view_mutex_);
        latest_ = frame;
      }
      if (io_.publish_frame) io_.publish_frame(frame);
      // Flush the entire telemetry snapshot before any large cloud. After
      // each cloud, check for newer telemetry again instead of serializing
      // a fleet's clouds ahead of everyone else's heartbeat.
      for(auto& s:frame.states)if(auto e=s.entity.lock();e && e->io && e->io->publish)e->io->publish(s);
    }
    bool published=false;
    for(size_t n=0;n<frame.states.size();++n) {
      const size_t i=(sensor_cursor+n)%frame.states.size();
      if(auto e=frame.states[i].entity.lock();e && e->io && e->io->publish_sensor && e->io->publish_sensor()) {
        sensor_cursor=(i+1)%frame.states.size();published=true;break;
      }
    }
    if(!published)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Json Server::status() {
  auto &m = world_.metrics;
  const auto completed = m.steps.load();
  auto seconds = std::chrono::duration<double>(Clock::now() - began_).count();
  return {
      {"instance_id", instance_},
      {"steps", completed},
      {"simulation_time_ns", world_.epoch + int64_t(completed) * world_.dt},
      {"epoch_ns", world_.epoch},
      {"model_step_ns", world_.dt},
      {"paused", m.paused.load()},
      {"rtf", double(completed) * double(world_.dt) * 1e-9 / seconds},
      {"lag_ns", m.lag_ns.load()},
      {"step_latency_ns", m.step_latency_ns.load()},
      {"max_step_latency_ns", m.max_step_latency_ns.load()},
      {"output_misses", m.output_misses.load()},
      {"input_coalesced", m.input_misses.load()},
      {"sensors", sensors_.status()}};
}

Frame Server::view() {
  std::lock_guard<std::mutex> l(view_mutex_);
  return latest_;
}

Json Server::receipt(const std::string &id, const Ticket &t) {
  Json j = {
      {"instance_id", instance_}, {"request_id", id}, {"accepted", true}};
  int phase = t->phase.load();
  if (phase == 4) {
    j["phase"] = "failed";
    j["result"] = result_json(t->result);
    j["error"] = requests_.at(id).preparation->error;
  } else if (phase == 2) {
    j["phase"] = "applied";
    j["result"] = result_json(t->result);
  } else if (phase == 3) {
    j["phase"] = "cancelled";
    j["result"] = {{"applied", false},
                   {"success", false},
                   {"reason", "deadline before execution"}};
  } else
    j["phase"] = phase == 1 ? "executing" : "accepted";
  return j;
}

Json Server::route(const std::string &method, const std::string &path,
             const Json &body, int &code) {
  if (method == "GET" && path == "/status")
    return status();
  if (method == "GET" && path == "/entities") {
    Json list = Json::array();
    for (const auto &s : view().states)
      if (auto entity = s.entity.lock(); entity && entity->alive) {
        const auto &e = *entity;
        list.push_back({{"id", e.id},
                        {"generation", e.generation.load()},
                        {"name", e.config.name},
                        {"kind", kind_name(e.config.kind)},
                        {"enabled", e.enabled.load()}});
      }
    return {{"instance_id", instance_}, {"entities", list}};
  }
  if (method == "GET" && path.rfind("/requests/", 0) == 0) {
    auto i = requests_.find(path.substr(10));
    if (i == requests_.end()) {
      code = 404;
      return {{"error", "unknown or expired request"}};
    }
    return receipt(i->first, i->second.ticket);
  }
  if (method != "POST" && method != "DELETE") {
    code = 404;
    return {{"error", "unknown endpoint"}};
  }
  if (body.at("instance_id") != instance_) {
    code = 409;
    return {{"error", "instance identity mismatch"}};
  }
  const auto id = body.at("request_id").get<std::string>();
  if (id.empty() || id.size() > 128)
    throw std::invalid_argument("request_id must have 1..128 characters");
  const auto payload = method + path + body.dump();
  auto prior = requests_.find(id);
  if (prior != requests_.end()) {
    if (prior->second.payload != payload) {
      code = 409;
      return {{"error", "request_id reused with different request"}};
    }
    return receipt(id, prior->second.ticket);
  }
  auto t = std::make_shared<Command>();
  std::shared_ptr<Entity> addition;
  Json addition_settings;
  std::shared_ptr<Entity> provider;
  std::shared_ptr<Preparation> preparation;
  auto timeout = body.value("timeout_ms", 1500);
  if (timeout < 1 || timeout > 5000)
    throw std::invalid_argument("timeout_ms must be 1..5000");
  t->deadline = Clock::now() + std::chrono::milliseconds(timeout);
  if (method == "POST" && path == "/entities") {
    // Check published projection and accepted in-flight additions before
    // preparing ROS resources; the world repeats the authoritative check.
    auto name = body.at("entity").at("name").get<std::string>();
    for (auto &s : view().states)
      if (auto e = s.entity.lock(); e && e->alive && e->config.name == name)
        throw std::invalid_argument("duplicate entity name");
    for (auto &r : requests_)
      if (r.second.ticket->op == Op::Add) {
        auto e = r.second.resource.lock();
        if (e && (r.second.ticket->phase < 2 || e->alive) &&
            e->config.name == name)
          throw std::invalid_argument("duplicate entity name");
      }
    t->op = Op::Add;
    addition_settings = body.at("entity");
    addition = std::make_shared<Entity>(parse_entity(addition_settings));
    preparation = std::make_shared<Preparation>();
  } else if (method == "POST" && path.rfind("/entities/", 0) == 0 &&
             path.size() > 19 &&
             path.compare(path.size() - 9, 9, "/provider") == 0) {
    const auto entity_id = path.substr(10, path.size() - 19);
    if (entity_id.find_first_not_of("0123456789") != std::string::npos)
      throw std::invalid_argument("provider entity_id must be numeric");
    const auto id = std::stoull(entity_id);
    const auto &generation = body.at("generation");
    if (!generation.is_number_integer() ||
        (!generation.is_number_unsigned() && generation.get<int64_t>() < 0))
      throw std::invalid_argument("provider generation must be unsigned integer");
    t->op = Op::Provider;
    t->key = {id, generation.get<uint64_t>()};
    const auto action = body.at("action").get<std::string>();
    if (action == "start")
      t->action = 1;
    else if (action == "stop")
      t->action = 2;
    else
      throw std::invalid_argument("provider action must be start or stop");
    for (const auto &s : view().states)
      if (s.key.id == id)
        if (auto e = s.entity.lock(); e && e->alive)
          provider = std::move(e);
    if (!provider)
      throw std::invalid_argument("unknown provider entity");
    if (t->action == 1)
      preparation = std::make_shared<Preparation>();
  } else if (method == "DELETE" && path.rfind("/entities/", 0) == 0 &&
             path.size() > 10 &&
             path.find_first_not_of("0123456789", 10) == std::string::npos) {
    t->op = Op::Remove;
    t->key = {std::stoull(path.substr(10)),
              body.at("generation").get<uint64_t>()};
  } else if (path == "/pause")
    t->op = Op::Pause;
  else if (path == "/resume")
    t->op = Op::Resume;
  else if (path == "/step") {
    t->op = Op::Step;
    t->steps = body.value("steps", uint64_t(1));
  } else if (path == "/reset") {
    t->op = Op::Reset;
    if (body.contains("entity_id")) {
      t->key = {body.at("entity_id").get<uint64_t>(),
                body.at("generation").get<uint64_t>()};
      for (auto &s : view().states)
        if (s.key.id == t->key.id)
          if (auto e = s.entity.lock())
            t->prepared = std::make_unique<Prepared>(
                Prepared{e, prepare_model(e->config)});
      if (!t->prepared)
        throw std::invalid_argument("unknown entity");
    } else
      for (auto &s : view().states)
        if (auto e = s.entity.lock())
          t->resets.emplace_back(s.key, prepare_model(e->config));
  } else {
    code = 404;
    return {{"error", "unknown endpoint"}};
  }
  requests_.emplace(id,
                    Request{t,
                            addition ? addition
                                     : provider ? provider
                                     : t->prepared ? t->prepared->entity
                                                   : std::weak_ptr<Entity>{},
                            Clock::now() + std::chrono::minutes(5), payload,
                            preparation, !preparation});
  if (preparation) {
    {
      std::lock_guard<std::mutex> lock(preparation_mutex_);
      preparation_tasks_.push_back([this, t, addition, addition_settings, provider, preparation] {
          if (t->phase == 0)
            try {
              preparation->value = addition
                  ? prepare(addition, addition_settings)
                  : std::make_unique<Prepared>(
                        Prepared{provider, prepare_model(provider->config)});
            } catch (const std::exception &e) {
              preparation->error = e.what();
            }
          preparation->done.store(true, std::memory_order_release);
      });
    }
    preparation_wake_.notify_one();
  } else
    world_.submit(t);
  code = 202;
  return receipt(id, t);
}

void Server::reply(Client &c) {
  try {
    const auto end = c.input.find("\r\n\r\n");
    if (end == std::string::npos)
      return;
    const auto first = c.input.find("\r\n");
    std::istringstream line(c.input.substr(0, first));
    std::string method, path, protocol;
    line >> method >> path >> protocol;
    if (protocol != "HTTP/1.1" && protocol != "HTTP/1.0")
      throw std::invalid_argument("HTTP/1.x required");
    size_t length = 0;
    bool have_length = false;
    std::istringstream headers(c.input.substr(first + 2, end - first - 2));
    std::string h;
    while (std::getline(headers, h)) {
      auto colon = h.find(':');
      if (colon == std::string::npos)
        throw std::invalid_argument("malformed header");
      auto key = h.substr(0, colon);
      for (auto &ch : key)
        ch = std::tolower(static_cast<unsigned char>(ch));
      if (key == "transfer-encoding")
        throw std::invalid_argument("chunked requests unsupported");
      if (key == "content-length") {
        if (have_length)
          throw std::invalid_argument("duplicate content length");
        length = std::stoull(h.substr(colon + 1));
        have_length = true;
      }
    }
    if (length > 1024 * 1024)
      throw std::invalid_argument("management JSON exceeds 1 MiB");
    if (c.input.size() < end + 4 + length)
      return;
    Json body = length ? Json::parse(c.input.substr(end + 4, length))
                       : Json::object();
    int code = 200;
    auto j = route(method, path, body, code);
    respond(c, code, j);
  } catch (const std::exception &error) {
    respond(c, 400, {{"error", error.what()}});
  }
}

void Server::respond(Client &c, int code, Json j) {
  j["instance_id"] = instance_;
  auto text = j.dump();
  c.output = "HTTP/1.1 " + std::to_string(code) + " " +
             (code == 200 ? "OK" : code == 202 ? "Accepted" : "Error") +
             "\r\nContent-Type: application/json\r\nConnection: "
             "close\r\nContent-Length: " +
             std::to_string(text.size()) + "\r\n\r\n" + text;
}

void Server::poll_once() {
  std::vector<pollfd> fds{{socket_.fd, POLLIN, 0}};
  for (auto &c : clients_)
    fds.push_back({c.fd, short(c.output.empty() ? POLLIN : POLLOUT), 0});
  if (poll(fds.data(), fds.size(), 1) < 0 && errno != EINTR)
    throw std::runtime_error("poll failed");
  const auto count = clients_.size();
  for (size_t i = count; i-- > 0;) {
    auto &c = clients_[i];
    bool close_now = Clock::now() > c.deadline;
    if (fds[i + 1].revents & (POLLERR | POLLNVAL))
      close_now = true;
    if (fds[i + 1].revents & POLLIN) {
      char buffer[8192];
      auto n = recv(c.fd, buffer, sizeof buffer, 0);
      if (n > 0) {
        c.input.append(buffer, n);
        if (c.input.size() > 1024 * 1024 + 8192)
          close_now = true;
        else
          reply(c);
      } else if (n == 0)
        close_now = true;
      else if (errno != EAGAIN && errno != EWOULDBLOCK)
        close_now = true;
    }
    if (fds[i + 1].revents & POLLOUT) {
      auto n = send(c.fd, c.output.data() + c.sent, c.output.size() - c.sent,
                    MSG_NOSIGNAL);
      if (n > 0) {
        c.sent += n;
        if (c.sent == c.output.size())
          close_now = true;
      } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        close_now = true;
    }
    if (close_now) {
      close(c.fd);
      clients_.erase(clients_.begin() + i);
    }
  }
  if (fds[0].revents & POLLIN)
    for (;;) {
      int fd = accept4(socket_.fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0)
        break;
      clients_.push_back(Client{fd, {}, {}});
    }
}

void Server::service_work() {
  while (preparing_) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(preparation_mutex_);
      if (preparation_tasks_.empty() && !io_.poll_services)
        preparation_wake_.wait(lock, [this] { return !preparing_ || !preparation_tasks_.empty(); });
      if (!preparing_) return;
      if (!preparation_tasks_.empty()) {
        task = std::move(preparation_tasks_.front());
        preparation_tasks_.pop_front();
      }
    }
    if (task) task();
    // These are the original two service workers, not an additional pool.
    // Native preparation is ordinary C++; only actual ROS services enter its queue.
    if (io_.poll_services) io_.poll_services();
  }
}
} // namespace xsim
