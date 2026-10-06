# xsim

One independent ROS1 simulation server per world. `xsim` owns entity identity,
physical state, provider generations and the simulation clock. It runs FS150,
Scout and Mecanum together. It does not load a sync-runtime Host or plugin.

## Build, install and package

Requires C++17, CMake 3.16, Eigen3, nlohmann-json, ROS Noetic (`roscpp`,
`geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_srvs`, `rosgraph_msgs`,
`mavros_msgs`), the installed robotics interface headers, xgc2-math headers,
and the owning FS150 SITL assets. First install this repository's
`src/xgc2_lightweight_sim_msgs` catkin package and the sensor owner's standalone
`convex_geometry/xgc2_world_lidar/library` CMake package into the chosen prefix.
No host installation is needed to build/test into a private prefix.

```sh
cmake -S ../convex_geometry/xgc2_world_lidar/library -B /private/lidar-build \
  -DCMAKE_INSTALL_PREFIX=/private/install -DCMAKE_BUILD_TYPE=Release
cmake --build /private/lidar-build -j1
cmake --install /private/lidar-build
cmake -S src/xsim -B /private/xsim-build \
  -DCMAKE_PREFIX_PATH='/private/install;/opt/ros/noetic' \
  -DCMAKE_INSTALL_PREFIX=/private/install \
  -DXGC2_MATH_INCLUDE=/path/to/math/include \
  -DFS150_ASSET_SOURCE_ROOT=/path/to/gazebo-sim/fs150-sitl \
  -DCMAKE_BUILD_TYPE=Release -DXSIM_TESTS=ON
cmake --build /private/xsim-build -j1
(cd /private/xsim-build && ctest --output-on-failure)
cmake --install /private/xsim-build
cpack --config /private/xsim-build/CPackConfig.cmake
```

The installed executable is `bin/xsim`; configuration and extracted FS150 asset
provenance are under `share/xsim`. The archive contains xsim, not its external
ROS/geometry dependencies. Build both the sensor library with
`-DXGC_WORLD_LIDAR_GPU=ON` and xsim with `-DXSIM_GPU=ON` for the original MARSIM
GPU backend. This additionally requires PCL/OpenCV/OpenGL/GLFW/GLM/OpenMP and a
real GPU/GL context at runtime. An unavailable backend is reported explicitly;
there is no CPU fallback. The fixed GPU renderer's GPL license accompanies its
installed shader assets.

## Process/configuration contract

```sh
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:PRIVATE_PORT \
  /private/install/bin/xsim --config /private/world.json --socket /private/world.sock
```

Core's future integration owns desired configuration, the ROS master/time-domain
selection, socket directory, process spawning/supervision and restart. It must
supply a unique `instance_id` and a positive integer `epoch_ns` from the formal
session time domain; the example epoch is illustrative, not a production default.
Core must ensure a single `/clock` publisher when `publish_clock=true`, and set
`/use_sim_time` consistently for consumers. xsim never selects wall-clock stamps
or silently starts at zero. Socket mode is 0600; its parent directory must already
exist. An existing path is refused, not unlinked. SIGINT/SIGTERM joins all owned
workers, closes ROS and removes only the server's own socket. Crash recovery and
removing a confirmed stale socket belong to the supervisor.

[config/example.json](config/example.json) shows all three kinds and one optional
sensor. `position` is world ENU base-origin position; yaw is radians. The explicit
`local_origin` is a world-axis translation (default zero): MAVROS local pose is
world position minus this vector; frame-1 local PVA positions add it back. It does
not rotate ENU axes or double-transform body setpoints. `ground_z` defaults to 0.
`fcu_parameters` is the original validated PX4 parameter-name/value object,
restricted to FS150. No gains, plant parameters or PX4 call periods were retuned.

Entities are initially provider-disabled, generation 0. Start/stop use the
existing `xgc2_lightweight_sim_msgs/SetProvider` ROS service. Start inactive uses
CAS on current generation, resets just that body, increments generation and
enables it. An active retry accepts current or predecessor generation. Stop
requires current generation. Observe (`action=0`) never starts or resets anything.
FS150 after stop continues its original passive last-motor physics while measured
outputs are gated. Ground providers stop their applicable velocity input (Scout's
original delayed response still settles); restarting resets the body. Ground
provider/reset is a new explicit lifecycle around the old always-running numerical
plant, not an invented flight arm/mode interface.

`model_step_ns` defaults to 1 ms, `output_period_ns` to 10 ms. Time advances only
after all kinds finish a whole step: `epoch_ns + steps * model_step_ns`. A steady
absolute wall deadline schedules bounded catch-up batches (default 8); lag does
not skip steps, alter dt or manufacture clock time. Pause freezes physical time
but still executes management and services. Step requires pause and completes
exactly the requested number of whole-world steps. Reset increments generations,
clears controller/filter/motor histories and returns initial poses; world reset
resets all currently snapshotted entities atomically after CAS, without rewinding
the session clock or enabling a disabled provider.

## ROS surface

Defaults below are per entity namespace `/<name>`. All output samples from one
step share the integer-derived stamp. ROS names can be overridden in `ros` with
the listed key; `frame` defaults to `map` for FS150 and `world` for UGV,
`body_frame` to the original `base_link`.

| Config key | Default suffix/path | Type / semantics |
|---|---|---|
| `truth_topic` | `/simulation/body_pose` | PoseStamped, world truth |
| `mocap_topic` | `/vrpn_client_node/<name>/pose` | PoseStamped, measurement source |
| `pose_topic` | `/mavros/local_position/pose` (FS150), `/simulation/pose` (UGV) | PoseStamped, local-origin translation |
| `velocity_topic` | `/mavros/local_position/velocity_local` (FS150), `/simulation/velocity` (UGV) | TwistStamped, world axes |
| `odometry_topic` | `/mavros/local_position/odom` (FS150), `/odom` (UGV) | Odometry, twist rotated into body child frame |
| `mocap_velocity_topic` | `/vrpn_client_node/<name>/twist` | TwistStamped, original world axes |
| `raw_imu_topic` | `/mavros/imu/data_raw` | FS150 specific force/gyro, orientation covariance -1 |
| `imu_topic` | `/mavros/imu/data` | FS150, body specific force/gyro, measured attitude |
| `state_topic`, `extended_state_topic` | `/mavros/state`, `/mavros/extended_state` | FS150 provider/FCU state |
| `target_attitude_topic` | `/mavros/setpoint_raw/target_attitude` | FS150 actual cascaded controller output |
| `setpoint_topic`, `attitude_topic` | `/mavros/setpoint_raw/local`, `/mavros/setpoint_raw/attitude` | Original PositionTarget/AttitudeTarget masks/frames |
| `cmd_vel_topic` | `/cmd_vel` | UGV Twist, body forward/left/yaw rate; Scout ignores left |
| `arming_service`, `command_service`, `mode_service` | `/mavros/cmd/arming`, `/mavros/cmd/command`, `/mavros/set_mode` | Original MAVROS services, FS150 only |
| `provider_service` | `/simulation/provider` | SetProvider, all kinds |
| `reset_service` | `/simulation/reset` | Trigger, resets this generation at world boundary |

`mocap_noise: [sx,sy,sz]` and `mocap_seed` (default 1) configure the original
publication-boundary Gaussian position noise, independently of truth and local
FCU feedback. The canonical `/<name>/pose` remains the existing measurement
Adapter's responsibility; xsim supplies its original measurement source, not a
truth substitute. The private test consumes the unchanged original Adapter projection function and
checks canonical offsets/stamps over ROS. Full production Adapter supervision and
adoption remain subsequent integration work.

Arm returns the actual model result (airborne disarm denied; forced operation
unsupported). MAVROS `mode_sent` retains its original meaning: a valid transmitted
request may be true even when the model rejects OFFBOARD for lack of stream or an
unsupported mode; state reports the actual mode. Responses wait for the world
execution boundary. A timed-out unclaimed request is atomically cancelled; a
request already claimed reports its completed result, so timeout cannot cause a
later hidden unlock. Subscriptions and FCU service callbacks bind immutable entity
ID/generation; resets invalidate queued callbacks/commands and old sensor samples.
As with the original unstamped Twist interface, a new message arriving through a
new subscription has no sender generation field; external senders must stop their
old stream before restarting the provider. TCPROS data already queued by external
subscribers cannot be retracted.

Core 保留当前 plant manifest 的特例路径时，显式传入
`ros.truth_topic="/xgc/simulation/body/<name>/pose"` 和
`ros.provider_service="/xgc/lightweight/providers/<name>"`；同时沿原 recipe 填入
`ros.mocap_noise=[1e-7,1e-7,1e-7]` 及原 `sim_mocap_noise_seed` 的数值到
`ros.mocap_seed`。当前 manifest 的 MAVROS `map`、mocap `world` 和 `base_link`
已是默认值。每个配置只有一个实际 topic/service，不创建旧新 alias 或兼容双实现。

## Unix HTTP/JSON management

Robot telemetry/control remains ROS. Management endpoints deliberately do not
carry pose/velocity streams.

- `GET /status`: instance, steps, simulation time, pause, lifetime wall RTF, current
  lag, latest/max step latency, output/coalescing misses, per-sensor latency/misses/errors.
- `GET /entities`: actual ID/generation/name/kind/enabled projection.
- `POST /entities`: add `entity` (same object as configuration).
- `DELETE /entities/<id>`: remove with `generation`.
- `POST /pause`, `/resume`, `/step` (`steps` integer), `/reset`.
  Reset optionally takes `entity_id` and `generation`; otherwise resets the world.
- `GET /requests/<request_id>`: accepted/executing/applied/cancelled/failed receipt.

Every mutation requires `instance_id`, nonempty `request_id`, and optional
`timeout_ms` (1..5000, default 1500). 202 means **accepted**, not applied. Poll the
receipt for `result.success`, `reason`, generation and actual step/time. Reasons:
0 success, 1 stale identity/generation, 2 denied/invalid, 3 unsupported, 4 duplicate
name, 5 execution/resource error. Duplicate IDs with identical payload return the
same receipt; changed payload is 409. Receipts remain five minutes; supervisors
must not replay expired mutation IDs. HTTP/1.x with Content-Length is supported,
one request per connection; slow sockets are handled with nonblocking poll.

```sh
curl --unix-socket /private/world.sock http://localhost/status
curl --unix-socket /private/world.sock -H 'Content-Type: application/json' \
  -d '{"instance_id":"example-session-world","request_id":"pause-1"}' \
  http://localhost/pause
curl --unix-socket /private/world.sock http://localhost/requests/pause-1
```

One shared boundary command executor is the only mutation authority for both ROS
and HTTP. Add prepares models, ROS endpoints and sensor resources outside the world
thread. Per-kind dense arrays and authoritative body/planar SoA columns share one
stable ID mapping; removal swaps the last dense element and rebinds its index.
Controller/filter state stays contiguous AoS; names/config/ROS handles are cold.
The world performs no ROS publication, service wait, socket I/O or sensor scan.
Resource preparation uses the two existing service executors; the input loop stays
responsive during model/sensor setup. GPU initialization/upload finishes on the
GPU owner before the add is submitted to the world. Preparation failure has
`phase: failed` and `applied: false`. Output receives reusable whole-world snapshots; it releases snapshot locks before
serialization/publication. Owned threads are world 1, input/HTTP 1 (main), service
2, output 1, plus CPU sensor workers (default 2) and/or one GPU context worker only
when requested. ROS middleware and driver threads are measured separately.

## Sensors

Sensors use the same immutable `LidarScene`/geometry/index. CPU uses the existing
`WorldLidar` raycast, penetrating or pinhole depth implementation. Configure range,
FOV, resolution, rate, mount translation/quaternion, and existing noise/seed.
Penetrating sampling spacing/buried policy is shared at world scene level. GPU PointCloud2 retains the original XYZ/intensity fields; CPU emits XYZ. GPU
uses the existing spherical-nearest `lidar_scan` implementation and one static
map upload/context; per-sensor projection switches within that owner thread.
It requires equal horizontal/vertical angular steps and the original point-cover
constraints. CPU/GPU are explicit observation models, not interchangeable claims.
The original GPU model does not implement CPU Gaussian range noise, so nonzero
GPU noise is rejected.

Due samples capture immutable pose, ID/generation, stamp and scene version 1.
This first server accepts one immutable configured scene; dynamic scene editing
is not a management endpoint. Each sensor has at most one executing, one pending
sample and one completed output. Overload replaces pending/completed older work
and increments misses; configured Hz/FOV/precision stay unchanged. Completion
publishes the original sample stamp and discards retired generations. Memory grows
with configured entities, beam patterns, output buffers and the shared scene;
there is no fixed fleet cap or arbitrary world disconnect threshold.

## Validation and retirement boundary

`tests/validate.sh` builds in a named network-none private container with one build
job, installs into its private prefix, extracts the exact baseline into that
private directory, and runs owning CTest plus isolated ROS. It never reads a prior
optimized TMP World package or changes live runtime mounts. See
[docs/validation.md](docs/validation.md) and the three kind reports for actual
commands, numeric evidence, differences and unverified scope.

The owning `lightweight_vehicle.cpp` Host plugin target and ABI/controller/provider
Host fixtures are retired. Pure model/PX4 tests remain; replay compiles the exact
historical source solely as a private test oracle, including its historical DTO
headers. The production simulation DTO exporter is retired; xsim installs no
Host/plugin records. Core now launches `/opt/xgc2/xsim/bin/xsim` directly through
`xsim-world`, with one desired `configJson` and the actual Session `epoch_ns`.
The old station launcher, Host bundle, manifest generator and graph have been
removed in the Core integration checkpoint. The original Adapter remains
unchanged; unrelated external `ros_io` simulation consumers are outside this
server's production path and must be removed by their owners if still used.

A prepared canonical YAML scene may be supplied through `scene_file`; it is
converted by the existing geometry library. `pointCloudBackend` in Scene
parameters selects integrated CPU or original GPU sensing. GPU startup fails
explicitly when its real implementation/hardware is unavailable. The ROS node
is `/xsim`; `/clock` is available before any per-robot provider is enabled.

The original GPU kernel uses one `polar_res` for both axes. Its real parameter
domain therefore requires `h_fov_deg / h_res == v_fov_deg / v_res` (the native
float angular step). For example, 120° × 60° with 240 × 120 samples is valid;
240 × 30 for those FOVs is rejected. xsim does not change FOV/resolution or
substitute CPU to accept a mismatched request. CPU retains its original model.
