# xsim

xsim 是每个世界一个进程的轻量多机器人仿真器，统一管理实体、动力学、Provider 生命周期和仿真时钟。控制与遥测通过可选 ROS 1 接口连接，世界管理通过 Unix socket 上的 HTTP/JSON RPC 完成。

[完整配置与接口参考](docs/reference.md)

## 支持的机器人

| `kind` | 模型 | 控制输入 |
| --- | --- | --- |
| `fs150` | PX4 v1.12.3 串级控制 → 四电机响应 → 六自由度刚体 RK4 → 平面接地 | PVA、姿态或机体系角速度目标；MAVROS Arm / Mode |
| `scout` | 速度限幅 → 纯延迟与一阶响应 → 差速平面运动学 | 车体系前进速度、偏航角速度 |
| `mecanum` | 速度限幅 → 全向平面运动学 | 车体系前进、横移速度与偏航角速度 |

FS150 使用轻量 FCU 接口和固定 PX4 控制源码；Scout、Mecanum 使用平面模型。传感器场景用于观测，不参与机器人或障碍物碰撞动力学。

## 启动

```sh
source /opt/ros/noetic/setup.bash
xsim --config /path/to/world.json --socket /path/to/world.sock
```

以 [config/example.json](config/example.json) 为配置起点，填入唯一 `instance_id` 和会话时间域的正整数 `epoch_ns`；socket 的缺失父目录由服务器以 `0700` 创建，socket 本身为 `0600`；已有路径不会被删除。示例世界初始暂停，实体初始 Provider 禁用。通过 RPC 启动对应 Provider，并恢复世界后运行。

工作流也可用 `xsim --experiment-file /private/experiment.json --socket /private/world.sock [--scene-file /private/scene.yaml]`。`--experiment-file` 与 `--config` 互斥；它接收冻结的 `{instanceId, epochNs, robots, context, settings}`，其中 `epochNs` 是精确十进制字符串。机器人为 `asset.experiment-robots@4` 的公开输出，`context` 含原 Session 的 opening 身份/接受时间、明确部署事实和分组可视化声明；`settings` 保留原工作流输入。模型、传感器、时序和 seed 在原生产品内解释，不读取当前资产或系统时间补齐缺失事实；原始 `authoredSimulationSensors` 必须存在。`--scene-file` 仍复用原场景 YAML 解析器。

默认世界调度 500 Hz（2 ms），整群共享墙钟实测 dt，时间为 `epoch_ns + Σ实际 dt`。短时落后限量追赶，持续过载平滑增大周期，默认上限 10 ms；空闲时阻塞等待。快照周期 8 ms，公共定位默认 125 Hz，IMU/local 默认 30 Hz，遥测频率可通过 RPC 调整；点云默认 10 Hz、独立降频。Pause 保持管理接口可用，Step 在暂停中按名义步长推进指定步数，Reset 保持会话时钟与 Provider 启用状态。

## 接口

以下为默认路径，`<name>` 是实体名称；ROS 配置可覆盖相应接口名。

| 方向 / 范围 | 话题或服务 |
| --- | --- |
| 输入 · FS150 | `/<name>/mavros/setpoint_raw/{local,attitude}` |
| 输入 · Scout / Mecanum | `/<name>/cmd_vel` |
| 输出 · 全部机器人 | `/<name>/{pose,twist}`，世界系公共定位；pose 支持配置位置噪声 |
| 输出 · FS150 | `/<name>/mavros/local_position/{pose,velocity_local,odom}`、`imu/{data,data_raw}`、`state`、`extended_state`、`setpoint_raw/target_attitude`（均在同一 MAVROS 命名空间） |
| 输出 · Scout / Mecanum | Scout `/<name>/imu/data_raw`，Mecanum `/<name>/imu`；机体系 IMU，默认 30 Hz |
| 输出 · 传感器 / 世界 | 可选 `/<name>/cloud`，CPU 可选 `/<name>/simple_lidar/beams`；`publish_clock=true` 时输出 `/clock` |
| ROS 服务 · FS150 | `/<name>/mavros/{cmd/arming,set_mode,cmd/command}` |
| RPC 查询 | `GET /capabilities`、`/config`、`/status`、`/entities`、`/telemetry-rates`、`/requests/<request_id>` |
| RPC 管理 | 实体增删、`/entities/<id>/provider` start / stop、`/pause`、`/resume`、`/step`、`/reset`；`POST /telemetry-rates` 改频 |

RPC 写请求带 `instance_id`、`request_id`，实体生命周期操作校验 `generation`。世界命令的 `202` 表示已受理，查询回执中的 `result.applied` 与 `success` 确认实际结果；遥测改频同步返回 `200 applied`，不推进物理时钟。Provider start 与 FS150 Arm 分别控制实体运行和飞行解锁。

## 源码与构建

| 目录（相对 `src/xsim/`） | 职责 |
| --- | --- |
| `core/` | 唯一实体表、稳定 ID / generation、组件、命令与世界边界执行 |
| `systems/` | 三类机器人批量步进；CPU / GPU 传感器任务与工作线程 |
| `models/` | 机器人数值模型与固定 PX4 控制源码 |
| `io/` | 配置、Unix RPC、ROS 输入与输出 |
| `main.cpp` | 组合 World、Sensors、IO 与 Server |

World 的物理组件采用按类型排列的 SoA，控制器与滤波状态保存在紧凑模型数组中；ROS 与 RPC 命令在同一个世界边界执行。不可变快照共享给输出、传感器和发布线程，点云数据共享复用；ROS 发布默认使用 2 个固定分片线程，每话题初始 4 个发送缓冲，按连接占用补足后复用。

构建、安装和打包见 [参考文档](docs/reference.md#构建安装与打包)。`XSIM_ROS=OFF` 可构建无 ROS 的同一世界与 RPC 服务；GPU 观测通过显式构建选项启用。

测试入口为 `src/xsim/test.sh`；`tests/validate.sh` 提供隔离构建与 ROS 集成检查。
