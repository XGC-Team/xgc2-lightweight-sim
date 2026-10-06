# Scout 对照回归证据

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
ctest --output-on-failure -R xsim_replay_scout
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

每段 1000 步，按同一时间表依次输入 body `(forward,left,yaw_rate)`：
`(1,0,0)`、`(1,0,0.5)`、`(0,0,0)`、`(-1,0,0)`、`(0,1,0)`、
`(0.7,-0.5,-0.4)`、`(0,0,0)`、`(0,0,0)`。

Scout 按原差速接口忽略 left，保留原 delayed planar velocity 的前进/转向/停止响应，
没有套用 flight Arm/Mode。位姿、速度、角速度、姿态和时间逐步最大差值全部为 **0**。

8000 步末两侧 position `[3.6497036170035906, -0.42878563434034483, 0.0]`。

## ROS 外部行为与差异

真实 Twist 输入在暂停边界发出并待接收，再执行每段 1000 步；逐段匹配同 stamp 的原 mocap pose/twist，
对照上述原实现的实际 checkpoint。额外的静止起始时间只平移绝对 stamp，不改变
输入间隔。检查 `world` frame、Odometry `base_link` 及原 mocap source/twist。

ROS 最大 pose/velocity 绝对差值 `2.48245868306185e-13`，容差 `1e-9`；
容差用于 ROS double 序列化/时间平移的舍入，不用于放松动力学行为。

旧 baseline 的地面模型没有 provider 协议；xsim 新增的 provider/reset 生命周期
单独验证，不能标成“原 UGV provider 原有行为”。Start 为 generation 1；reset 返回
初态并变为 generation 2；旧 generation stop 返回 stale reason 1。永久 world 测试
还对该 kind 验证 reset 前排队的未来旧命令不会作用于重置实体，swap/remove 后旧 ID
也不能命中新 replacement。所有这些检查均沿 UGV Twist 接口，不构造飞行 Arm/Mode。

## 证据与未验证范围

- [完整数值 checkpoint/误差/事件](evidence/scout-replay.json)
- [实际 ROS 输出与服务结果](evidence/scout-ros.json)
- [总验证记录、构建命令和扩展测量](validation.md)

数值回放和上述私有 ROS 检查通过。未执行 live 站、论文 E2E、真实机器人、生产
Adapter 全生命周期、Core/前端/工作流接入或部署；这些不是本报告的 PASS 范围。
