# Lightweight simulation product

This product owns the flight rigid body, virtual FCU, world provider lifecycle
and native facade. It builds independently of ROS and the SSS ROS meta-package.
Runtime supplies only `XgcRuntime::SDK`; no Runtime domain source is compiled.
The plugin remains `liblightweight_vehicle.so`, version 0.3.0.

Build/install with the owning FS150 asset sources and either installed SDK or
an explicit `XGC_RUNTIME_SDK_SOURCE_ROOT`:

```sh
FS150_ASSET_SOURCE_ROOT=/path/to/fs150-sitl \
XGC_RUNTIME_SDK_SOURCE_ROOT=/path/to/sync-runtime \
XGC2_MATH_INCLUDE=/path/to/math/include ./build.sh /tmp/build /tmp/install
```

The FS150 generator consumes `models/fs150/iris.sdf`, the active
`config/generated/fs150-sitl.params` overlay and this product's names-only FCU
catalog. It creates the physical/header input and installed provenance JSON.
Machine coefficients and active overrides have one asset source. They describe
an Iris-equivalent simulation, not calibrated real-airframe measurements.
PX4 default values/ranges and startup overrides are owned by the FCU parameter
block. Unsupported names, nonfinite values and duplicate overrides fail closed.
Startup TOML uses `[fcu_parameters]`; old `hover_thrust_ratio` and
`offboard_timeout_ms` aliases are rejected.

The necessary unmodified PX4 v1.12.3 control/mixer sources are pinned at
`2e8918da66af37922ededee1cc2d2efffec4cfb2` in `px4_v1_12_3`, with source hashes
and licenses. PositionControl, AttitudeControl, RateControl and
MultirotorMixer keep their parameters, history, saturation feedback and resets.
SMC acceleration input bypasses the position/velocity feedback; raw body rates
bypass attitude feedback. All channels drive the same four motors and 6DoF
state. There is no old ideal-flight implementation or alternative simple FCU.

The execution boundary converts the original mixer's [-1,1] output once to
normalized motor command. The original thrust-model inverse is applied once;
asset control channels map command to target rotor speed, first-order motors
advance actual speed, and force is `kf * omega^2`. No PWM or ESC protocol is
implemented. `MPC_THR_HOVER` is a controller expectation, not a second physical
thrust map. The active FS overlay value is approximately .36; a selected
calibrated profile must explicitly record its startup override (approximately
.27726948 for the current physical equilibrium with THR_MDL_FAC=0).

COM position/velocity use ENU; the quaternion maps body FLU to ENU; angular
velocity and IMU specific force are body-frame. Pose/velocity/IMU come from the
same model step and base measurement point. Ground is the independent finite
`world_ground_z` (default zero), never initial pose height. A unilateral
frictionless normal impulse updates COM velocity and angular momentum using
contact-point effective mass. This is one flat-plane contact, not obstacle or
mesh collision dynamics.

A batch contains at most six robots, each with a ten-port block. The original
first eight port positions are retained; attitude_command/2 and
attitude_target/2 are the next two. Global ports 60..63 are fcu_result/1,
fcu_extended_state/1, provider_request/1 and provider_result/1. FCU requests use
fcu_request/2 with correlated execution results. Old eight-robot layouts fail
before launch. Scout/Mecanum keep their original planar models and do not yet
implement the flight provider lifecycle.

The FS150 provider is initially offline. Action 0 observes without mutation.
Inactive start requires the current previous generation and creates the next;
active retry is idempotent. Stop requires the current generation. Every result
returns the current generation/enabled state even on stale rejection. First
start and restart reset only that body to its frozen initial pose, clearing
physical/control histories and queued old controls. Physics-owner execution is
serial; world time, siblings and independent observers continue. User stop and
ordinary command source-loss are distinct. Inactive post-stop physics retains
the last physical target; a subsequent start resets it. Source-loss follows
COM_OF_LOSS_T and does not create a generation or reset pose.

Use a shared epoch, 1 ms model step and 10 ms output period. Centralized Core
sets `advance_on_round=true` so batches share a round target; distributed hosts
share an epoch/grid and use their synchronized clock. Consumers pair source
stamps. This introduces no wireless barrier and does not guarantee synchronized
packet arrival. PVA conversion preserves the fixed MAVROS1.20/PX4receiver
semantics, including receiver heartbeat versus PositionControl validity.
Rejected user input does not refresh the valid stream or terminate the batch.

`FlightModel::step` reads no FCU parameter by name: the values it needs
(`MPC_TILTMAX_AIR/LND`, `THR_MDL_FAC`, `MOT_SLEW_MAX`, `COM_OF_LOSS_T`,
`MPC_LAND_SPEED`) are read once at construction from the immutable parameter
block. `hot_path_test` asserts zero `get()` calls inside any step and that each
hoisted value is the float `get()` returns, with default and overridden
parameters.

`test.sh` builds owning targets, retaining assertions in Release tests.
`provider_test` loads the actual installed ELF and checks lifecycle/generation
fences. `test-controller.sh PLANT.so CTL.so HTE.so ...` consumes actual installed
artifacts and the real external controller/estimator. Component/oracle tests
and artifact publication do not substitute for the real SMC/DFBC/HTE,
measurement/Adapter, sensor, scene and experiment acceptance gates.
