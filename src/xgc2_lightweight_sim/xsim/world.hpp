#pragma once
#include "../vehicle_model.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <list>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unordered_map>
#include <variant>

namespace xsim {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace xgc_lightweight;
enum class Kind { FS150, Scout, Mecanum };
struct Key {
  uint64_t id = 0, generation = 0;
};
struct Config {
  std::string name;
  Kind kind = Kind::FS150;
  Eigen::Vector3d initial{Eigen::Vector3d::Zero()},
      local_origin{Eigen::Vector3d::Zero()};
  double yaw = 0, ground_z = 0;
  FlightControllerParameters fcu;
  Json ros = Json::object(), sensor = Json::object();
};
Config parse_entity(const Json &);
struct RosEntity;
struct Sensor;
struct Entity {
  explicit Entity(Config c) : config(std::move(c)) {}
  const Config config;
  uint64_t id = 0;
  // Written only at world boundaries; callback/output projections use atomics.
  std::atomic<uint64_t> generation{0};
  std::atomic<bool> alive{false}, enabled{false};
  std::atomic<int64_t> generation_stamp{0};
  std::shared_ptr<RosEntity> ros;
  std::shared_ptr<Sensor> sensor;
};
struct Flight {
  FlightModel model;
  std::string mode = "POSCTL";
  bool ever_started = false;
  explicit Flight(const Config &c)
      : model(c.initial, c.yaw, c.fcu, c.ground_z) {}
};
struct Scout {
  ScoutModel model;
  double age = 0;
  uint64_t steps = 0;
  explicit Scout(const Config &c) : model({c.initial.head<2>(), c.yaw}) {}
};
struct Mecanum {
  MecanumModel model;
  explicit Mecanum(const Config &c) : model({c.initial.head<2>(), c.yaw}) {}
};
using Model = std::variant<Flight, Scout, Mecanum>;
Model prepare_model(const Config &);
struct Prepared {
  std::shared_ptr<Entity> entity;
  Model model;
};
struct State {
  // Snapshots observe lifecycle; they must not keep removed ROS/sensor
  // resources alive.
  std::weak_ptr<Entity> entity;
  Key key;
  int64_t stamp = 0;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()},
      velocity{Eigen::Vector3d::Zero()}, omega{Eigen::Vector3d::Zero()},
      specific_force{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  FlightControlOutput control;
  Eigen::Vector4d rotors{Eigen::Vector4d::Zero()};
  bool enabled = false, armed = false, landed = true;
  std::array<char, 24> mode{};
};
struct Frame {
  int64_t stamp = 0;
  uint64_t steps = 0, revision = 0;
  std::vector<State> states;
};
enum class Op {
  Add,
  Remove,
  Pause,
  Resume,
  Step,
  Reset,
  Provider,
  Arm,
  Mode,
  Pva,
  Attitude,
  Velocity
};
struct Result {
  bool applied = false, success = false, enabled = false;
  uint32_t reason = 0;
  Key key;
  uint64_t step = 0;
  int64_t stamp = 0;
};
struct Command {
  Op op = Op::Pause;
  Key key;
  int action = 0;
  bool arm = false;
  std::string mode;
  uint64_t steps = 1;
  int64_t at = 0;
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // forward,left,yaw rate
  xgc_position_target_v1 pva{};
  FlightAttitudeSetpoint attitude;
  std::unique_ptr<Prepared> prepared;
  std::shared_ptr<Entity> retired;
  std::vector<std::pair<Key, Model>> resets;
  Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(1500);
  // 0 queued; 1 claimed; 2 applied; 3 cancelled. Cancellation wins before
  // claim, or caller waits for the already executing bounded boundary
  // operation.
  std::atomic<int> phase{0};
  Result result;
  std::mutex mutex;
  std::condition_variable done;
};
using Ticket = std::shared_ptr<Command>;
struct Metrics {
  std::atomic<uint64_t> steps{0}, output_misses{0}, input_misses{0};
  std::atomic<int64_t> sim_ns{0}, lag_ns{0}, step_latency_ns{0},
      max_step_latency_ns{0};
  std::atomic<bool> paused{false};
};
class World {
public:
  World(int64_t epoch, int64_t dt = 1000000, int64_t output = 10000000,
        unsigned catchup = 8);
  ~World();
  World(const World &) = delete;
  void submit(const Ticket &); // nonworld threads; continuous inputs coalesce
                               // by entity/type/time
  static bool wait(const Ticket &);
  void boundary(); // world owner only, also used by deterministic fixtures
  void advance();
  Frame capture() const;    // world owner only (fixtures)
  bool take_frame(Frame &); // output owner; never holds lock while publishing
  void start();
  void stop();
  Metrics metrics;
  const int64_t epoch, dt, output_period;
  std::function<void(const State &)> sample_sensor;

private:
  struct Slot {
    std::shared_ptr<Entity> entity;
    size_t dense = 0;
  };
  std::unordered_map<uint64_t, Slot> slots_;
  std::vector<uint64_t> flight_ids_, scout_ids_, mecanum_ids_;
  BodyColumns bodies_;
  PlanarColumns scout_poses_, mecanum_poses_;
  std::vector<Flight> flights_;
  std::vector<Scout> scouts_;
  std::vector<Mecanum> mecanums_;
  uint64_t next_id_ = 1, steps_ = 0, stepping_ = 0, revision_ = 0,
           emitted_revision_ = UINT64_MAX;
  int64_t emitted_stamp_ = -1;
  std::deque<Ticket> step_waiters_;
  int64_t time_, next_output_;
  unsigned catchup_;
  std::mutex input_mutex_, output_mutex_;
  std::list<Ticket> inbox_, commands_;
  Frame ready_, scratch_;
  bool ready_available_ = false;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::condition_variable wake_;
  std::mutex wake_mutex_;
  State state(uint64_t, const Slot &) const;
  Result apply(Command &);
  void reset(Slot &, Model &&);
  void emit();
  void finish(const Ticket &, Result);
  void run();
};
} // namespace xsim
