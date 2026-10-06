# FS150 对照回归证据

Baseline 是现行 owning 仓 `restart/pre-optimization-2026-10-06` 的
`fed3fdc3d6625d9020d363bb5b9c3c8f8f2c72a6`。测试从这个精确 Git 对象导出
原实现到私有目录，编译原 `lightweight_vehicle.cpp` 动态库；没有引用旧优化分支或
旧 TMP World 包。xsim 与 baseline 使用同一 FS150 资产提取结果、同一未改动的固定
PX4 pure core、同一工具链及原默认参数。测试加载原插件只是离线 oracle，xsim
服务器没有 Host/SDK 执行依赖。

同输入回放：初态 `(x,y,z,yaw)=(2,-1,0,0.35)`，epoch
`1700000000000000000 ns`，dt/output 均为 `1 ms`，8000 步；两侧均关闭噪声。
时间全部从整数 step index 推进。逐步比较使用绝对容差 `1e-10`；这是一致实现迁移的
数值相等检查，比原模型物理行为测试的 `1e-6` 等容差严格，没有为了通过调整物理参数
或放松容差。实际最大差值为 0。

命令（私有容器内 `/work/xsim`）：

```sh
ctest --output-on-failure -R xsim_replay_fs150
ctest --output-on-failure -R xsim_ros_contract
```

ROS fixture 使用私有 master、原消息/服务定义及同一 xsim 执行口。
canonical 路径在 owning 私有 fixture 中调用 Adapter 原件
`localization_projection.cpp`，源码 HEAD 为
`ba4ce7d3c14d16d7015dc62fd06954a1e41d323b`，不复制算法、不改 Adapter。
验证偏移 `(0.4,-0.2,0.3)` 后 stamp/frame 保留，偏移误差 `<1e-12`。
这证明原投影函数与真实 ROS source 的连线；没有执行整套生产 Adapter 的监督、健康、
Core/workflow 生命周期，不能写成整站或接入通过。

## 数值行为

初始 provider start/Arm/OFFBOARD 后，每 20 ms 输入一次原 PositionTarget：
frame 1，mask `8|16|32|64|128|256|2048`，目标 `(2.5,-0.5,1)`、yaw `0.6`；
第 3000 步起目标 x 改为 `1.5`。第 4000 步请求空中 disarm（拒绝 2），第 4500 步
请求 unsupported mode（结果 3）。第 6000 步 stop，第 6500 步 start/reset 到
初态、generation 2，并重发同类 setpoint/Arm/OFFBOARD。每个 FCU 执行结果均与
baseline 对应 request ID 比较，Arm/Mode 成功 0，拒绝码一致。

位置、世界速度/角速度、四元数、IMU specific force/gyro、串级 attitude/rate
目标、normalized thrust、时间逐步最大绝对差值全部为 **0**；provider enabled、armed、
mode 状态转换一致。

8000 步末两侧 position `[1.6406001389292781, -0.6374780113806277, 1.1110521208648898]`，velocity `[-0.2948121302451665, 0.29427989687253164, 0.19211216403704623]`。

## ROS 外部行为

真实 ROS 检查使用非零 world 初态 `(2,-1,0)`，并显式设置 local_origin `(2,-1,0)`；
local pose 为 `(0,0,0)`，初始样本 stamp `1700000000010000000 ns`。
默认 local_origin=0 的数值回放与旧实现直接相同；这里另外验证新显式平移配置，
不是声称旧 Host 存在该新配置字段。MAVROS frame `map`、mocap frame `world`、
body `base_link`、raw IMU orientation covariance `-1` 和 mocap twist 均实际检查。

输入 local PVA 后真实起飞；ground Arm 返回 0，空中 disarm 返回 2，forced arm 返回 3，
offline Arm 返回 4。无新鲜 setpoint 的 OFFBOARD 请求保持原 `mode_sent=true` 传输语义，
实际 state 仍 POSCTL；不把这个布尔值当作模式执行成功。paused provider stop 的
connected=false 在时间不推进时仍立即可见。Provider reset/CAS 及旧 generation stop
拒绝、旧未来 setpoint 清除通过。

本次 ROS 起飞点 `[2.081381495517889, -0.9195814523538868, 0.6490817600784169]`，target thrust `0.25043943524360657`；
真实 CPU 点云 `912` 点，保留采样 stamp `1700000000601000000`。

## 证据与未验证范围

- [完整数值 checkpoint/误差/事件](evidence/fs150-replay.json)
- [实际 ROS 输出与服务结果](evidence/fs150-ros.json)
- [总验证记录、构建命令和扩展测量](validation.md)

数值回放和上述私有 ROS 检查通过。未执行 live 站、论文 E2E、真实机器人、生产
Adapter 全生命周期、Core/前端/工作流接入或部署；这些不是本报告的 PASS 范围。
