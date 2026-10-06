# xsim 独立服务器交付与验证

2026-10-06；本报告是限定范围的本地证据，待 Root 最终审查，不是整站接入/部署验收。
改动仅在 lightweight-sim owning 仓和 convex_geometry 的必要 sensor library 接口/导出/测试。
没有提交、推送、父 pins 修改，没有 live 站、Stop/Run、host 安装或用户 runtime 物料变更。

## 代码与运行合同

- `src/xgc2_lightweight_sim/xsim/`：world/ECS、三 kind 连续批处理、唯一时钟、ROS、
  Unix HTTP/JSON 管理、输出快照、CPU/GPU sensor 异步执行。
- `rigid_body.hpp` / `vehicle_model.hpp`：物理状态迁入 world-owned SoA/planar columns，
  controller/filter 保持紧凑同类 AoS；原模型/控制数学未调参。
- `CMakeLists.txt` / build/test scripts：实际 `xsim` build/install 与 TGZ package。
- `tests/`：精确 baseline 回放、world/generation/timeout、真实私有 ROS、原 Adapter
  投影、慢服务、20/100 扩展、GPU smoke。
- convex_geometry `xgc2_world_lidar/library/`：普通 CMake `Geometry` / `Gpu` 导出；
  GPU overload 用一个 context/map 和离屏 framebuffer 切换投影，原 shader/扫描数学未改。
  原三参数固定窗口入口与新入口的实际 XYZ/intensity 输出另有永久对照。

[完整配置、启动、安装、ROS 与 Core 管理合同](../README.md)。Core 后续普通 spawn
`xsim --config WORLD.json --socket PRIVATE.sock`；Core owns desired config/监督，xsim owns
actual world。管理 accepted 与 applied/result 分开，耗时准备在固定 service/GPU 线程完成
后才入 world 边界。telemetry/control 仍是原 ROS + 原 Adapter，未接入新的数据总线。

## 实际构建与测试

私有镜像 `ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0`，network none，
CPU 28/29、quota 2 CPU、memory 5 GiB、单个 `-j1` job。源码挂载 `/source:ro`，输出
`/tmp/xsim-validation-20261006` 映射 `/work`；没有复用站运行 cache 或旧 TMP World 包。
CPU 0 留给 Viewer 的并行任务。

实际执行的等价命令（全部在私有容器内，先按 `tests/validate.sh` 安装本次 message、
robotics headers 和当前 sensor library 到同一私有 prefix）：

```sh
cmake -S /source/ros1/simulator/lightweight-sim/src/xgc2_lightweight_sim -B /work/xsim \
  -DCMAKE_PREFIX_PATH='/work/install;/opt/ros/noetic' -DCMAKE_INSTALL_PREFIX=/work/install \
  -DCMAKE_BUILD_TYPE=Release -DXGC2_MATH_INCLUDE=/source/common/math/include \
  -DFS150_ASSET_SOURCE_ROOT=/source/ros1/simulator/gazebo-sim/fs150-sitl \
  -DLIGHTWEIGHT_TESTS=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DXSIM_BASELINE_SOURCE=/work/src/xgc2_lightweight_sim \
  -DXSIM_BASELINE_SDK=/source/common/sync-runtime \
  -DXSIM_ADAPTER_SOURCE=/source/ros1/communication/ros1-adapter -DXSIM_PRIVATE_ROS_TESTS=ON
cmake --build /work/xsim -j1
cmake --install /work/xsim
cd /work/xsim
ctest --output-on-failure
cpack --config CPackConfig.cmake
/work/install/bin/xsim --help
```

结果：CPU build/install/package 成功。最终 C++ 版本的原数值/ECS **9 项 PASS**；
ROS fixture 首次发现它自己把不同 stamp 的 latest pose/twist 混配，修正为等待原
mocap pose/twist/odom 同 stamp（未放松容差），定向重跑 **1/1 PASS**，合计 10 项通过。
两个 service executor 同时被真实 ROS callbacks 阻塞 250 ms，100 ms 内仍完成
100 个启用 FS150 的 whole-world 物理步，管理 pause 返回已执行。
另外验证首步等满 dt 才推进时钟，以及 dt=3 ms/output=10 ms 的原 epoch 网格
输出序列（12/21/30/42/51/60 ms）。输出先发布整批机器人状态，再轮换一个完成点云，
每个点云后重新检查状态快照，避免连续序列化整批点云压住遥测。

永久重放入口：`XSIM_TEST_CPUSET=28,29 tests/validate.sh NEW_PRIVATE_OUTPUT_DIR`。
[测试日志](evidence/ctest.txt)、[慢服务结果](evidence/slow-service.json)、
[测试过的私有二进制/资产哈希与原件版本](evidence/artifacts.json)。这些哈希只是证据，
不进入服务器启动或 Core 准入逻辑。`ldd` 不含 runtime Host/SDK；baseline SDK headers
仅服务历史 oracle fixture，不是 xsim 的运行依赖。

## 三份对照证据

- [FS150](fs150-regression.md)：8000 步；位姿/速度/IMU/串级输出/thrust/时间逐步最大差 0；
  对照 FCU request ID 的实际结果，另测非零 world/local、canonical projection、Arm/Mode、
  provider、paused 状态、reset/旧 generation。
- [Scout](scout-regression.md)：8000 步；原差速前进/转向/停止模型逐步最大差 0；
  真 ROS checkpoint 与原输出比较，provider/reset/旧命令另验。
- [Mecanum](mecanum-regression.md)：8000 步；前后/横移/转向/组合/停止逐步最大差 0；
  真 ROS checkpoint、provider/reset/旧命令另验。

本轮原 rigid body、flight controller、model、PX4 setpoint、ground contact 测试保留并通过。
没有用“源码没变”“创建成功”或编译结果替代上述行为回放。

## 20/100 同一扩展 fixture

默认 dt=1 ms/output=10 ms。三 kind 轮换，全部 provider enabled、零运动命令/地面 FS150；
每 5 个实体一个 CPU sensor（720×120，180°×60°，30 Hz），同一个小 box scene/index。
这测到了真实物理/扫描推进，但不代表百机主动飞行、复杂地图或大量外部订阅者的性能。
没有引入新的性能框架，没有设置实体容量 gate，也不将本表写成一般实时 PASS。

| 实体 | CPU sensor 路数 | 实际完成步 | wall s | 区间 RTF | RSS | 实际总线程 | scan 完成 | 漏采/替换 | 最大 sensor latency ms |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20 | 4 | 2006 | 2.009045 | 0.998485 | 34492 kB | 11 | 242 | 0 | 20.663 |
| 100 | 20 | 2006 | 2.009410 | 0.998303 | 119588 kB | 11 | 1158 | 45 | 38.547 |

线程数包含 ROS 中间件；own threads 为 world 1、input/HTTP 1、service 2、output 1、
CPU sensor 2。两种规模线程数相同。sensor 错误为 0；漏采/替换和延迟按实际值保留，
没有改 Hz/FOV/分辨率来换通过。完整 step/lag/max latency/output miss 数据在
[scale.json](evidence/scale.json)；其中 max step latency 是该进程累计值，不冒充每段分位数。

## GPU 的编译与真实运行

在相同私有环境编译安装 `XGC_WORLD_LIDAR_GPU=ON` 的 sensor library，及 `XSIM_GPU=ON`
的 xsim。镜像缺失的 GLFW 3.3.2-1 与 GLM 0.9.9.7+ds-1 仅通过已签名 Ubuntu focal
仓下载并解包到 `/work/gpu-deps`；没有 host install。CPU/GPU 混合私有 prefix 的公共
sensor headers 必须来自同一当前版本；本次已重装当前 headers 后重新编译链接。

```sh
cmake -S /source/ros1/simulator/convex_geometry/xgc2_world_lidar/library -B /work/lidar-gpu \
  -DCMAKE_INSTALL_PREFIX=/work/install-gpu -DCMAKE_PREFIX_PATH=/work/gpu-deps/usr \
  -DCMAKE_BUILD_TYPE=Release -DXGC_WORLD_LIDAR_GPU=ON -DXGC_WORLD_LIDAR_GPU_REFERENCE_TEST=ON
cmake --build /work/lidar-gpu -j1
cmake --install /work/lidar-gpu
# xsim uses the same CPU configure contract plus the current GPU prefix and -DXSIM_GPU=ON.
cmake --build /work/xsim-gpu --target xsim -j1
cmake --install /work/xsim-gpu
/work/lidar-gpu/world_lidar_gpu_reference_test
python3 /tests/gpu_smoke.py --xsim /work/install-gpu/bin/xsim --output /work/gpu-closure
```

运行在独立 network-none、`--gpus all` 容器，CPU 28/29、2 CPU、3 GiB、pids 128；
只读挂载既有 X11 socket/当前用户 Xauthority，原 renderer 使用 invisible window，
没有操作 live 站或主机图形配置。实际 GL：NVIDIA RTX 4060 Laptop GPU，OpenGL
`4.6.0 NVIDIA 595.91.07`。

原三参数扫描入口与共享 context 的两个分辨率（120×40、240×80）交替 12 次，
原输出分别 3760 / 14480 点，**XYZ/intensity 最大绝对差 0**（检查容差 1e-6）。
离屏 framebuffer 避免 X11 异步窗口 resize 读到上一尺寸，保留查询得到的原生 depth
精度，不改投影、遮挡或点云科学数学。[原入口对照](evidence/gpu-reference.json)。

真实 server 两 sensor 共用一个地图上传/context，一个 GPU 执行线程，CPU sensor
worker 为 0；三帧点数分别稳定为 3760 / 14480，PointCloud2 保留 XYZ/intensity
(16 bytes/point)，原采样 stamp。GPU 资源准备期间 world 实际前进 360 步后完成第一个
add；错误和漏采为 0，退出码 0。[server 实测](evidence/gpu.json)。
GPU 仅实测 2 路，不是 7 路或 100 路。CPU 的 100 实体场景仅有 20 路 sensor。
这只是同小 scene 的真实硬件验证，不扩大为多地图/大规模 GPU、其他 GPU/driver 或论文 E2E。
CPU-only build 请求 GPU 也有永久失败路径检查：明确 failed/applied=false，不创建实体、
不自动 fallback 到 CPU。

## 退役与后续接入边界

owning 旧 `lightweight_vehicle.cpp` / `lightweight_vehicle` shared Host target、flat Host
config parser、ABI/provider/controller Host fixtures 已删除，替换为实际 server。
纯数学测试与原固定 PX4 core 保留。历史 baseline 只在私有测试导出树中存在。
旧 simulation DTO/interface 包仍保留给尚未改动的外部消费者；不声称全仓已退役。

Root 后续接入/删除项：Core `internal/lightweightplant` 及其 Host graph/launcher，
`scripts/package-lightweight-plant.py` 与现 catalog/recipes，外部 ros_io simulation
edges/provider group，以及旧外部消费者的 DTO 依赖。新的 desired config/Unix socket
spawn、provider 接线、旧消费者删除、父 pins、包发布/安装、前端按钮/工作流采用和
站部署均未执行。现行声明的 topic/service 特例见 README 的 Core 映射说明。

未运行：六论文矩阵、整站 Stop/Run/E2E、实车、生产 Adapter 完整监督/健康生命周期、
Core/前端/工作流接入与部署。服务器与证据交付后停在此边界，由 Root 汇报并与用户
讨论下一步接入；这些接入项不阻塞独立 server 已完成的限定验证。
