#pragma once
#include "io/runtime_io.hpp"
#include "systems/sensors.hpp"
#include <csignal>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
namespace xsim {
class Server {
public:
  Server(const Json &config, const std::string &socket, World &, Sensors &, RuntimeIO io = {});
  ~Server();
  void run(const volatile sig_atomic_t &stopping);
private:
  struct Client {
    int fd;
    std::string input, output;
    size_t sent = 0;
    Clock::time_point deadline = Clock::now() + std::chrono::seconds(5);
  };
  struct Preparation {
    std::atomic<bool> done{false};
    std::unique_ptr<Prepared> value;
    std::string error;
  };
  struct Request {
    Ticket ticket;
    std::weak_ptr<Entity> resource;
    Clock::time_point expiry;
    std::string payload;
    std::shared_ptr<Preparation> preparation;
    bool submitted = false;
  };
  std::string instance_;
  RuntimeIO io_;
  World &world_;
  Sensors &sensors_;
  struct BoundSocket {
    int fd = -1;
    std::string path;
    ~BoundSocket();
  } socket_;
  std::vector<Client> clients_;
  std::unordered_map<std::string, Request> requests_;
  std::mutex view_mutex_;
  Frame latest_;
  std::thread output_;
  std::atomic<bool> output_running_{false};
  Clock::time_point began_ = Clock::now();
  int64_t input_poll_ns_ = 1000000;
  std::mutex preparation_mutex_;
  std::condition_variable preparation_wake_;
  std::deque<std::function<void()>> preparation_tasks_;
  std::vector<std::thread> service_workers_;
  std::atomic<bool> preparing_{false};
  std::unique_ptr<Prepared> prepare(const Json &);
  std::unique_ptr<Prepared> prepare(const std::shared_ptr<Entity> &, const Json &);
  void service_work();
  void shutdown();
  void output();
  Json status();
  Frame view();
  Json receipt(const std::string &, const Ticket &);
  Json route(const std::string &, const std::string &, const Json &, int &);
  void reply(Client &);
  void respond(Client &, int, Json);
  void poll_once();
};
} // namespace xsim
