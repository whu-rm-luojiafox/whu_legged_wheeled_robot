# 底盘算法 MATLAB 对照仿真

`run_chassis_algorithm_sim.m` 按固件的 2 ms 周期复现以下链路：

- HT 电机反馈速度（rad/s）
- 五连杆正运动学、腿长速度和腿角速度
- `LQR.c` 的 40×6 变腿长增益拟合
- 轮毂/关节 LQR 力矩、VMC 映射与 ±18 N·m 限幅
- 腿长速度差分、支撑力估计和离地判断

脚本直接读取工程中的 `chassis_task.c/.h`、`CAN_receive.h` 和 `LQR.c`，避免 MATLAB 参数与固件手工复制后失配。它同时比较：

1. `legacy`：最近改动前的速度缩放和旧腿角速度公式；
2. `problem`：本次上车发散版本中的错误腿长雅可比、右轮使用左腿角速度、0.18 m/s 零指令偏置；
3. `fixed`：当前修正版的五连杆精确 `dL/dt`、与腿角导数一致的角速度符号、正确右腿反馈和零速度偏置。

合成轨迹在 2.0 s 处加入一个单采样、2 rad/s 的关节速度诊断脉冲（仍远小于 HT 协议的 ±45 rad/s），用于检查 500 Hz 腿长加速度和离地判定对速度毛刺的敏感性。

在 MATLAB 中运行：

```matlab
cd('E:\02GC\01RM_Balance_git\01Balance_chassis_task\MDK-ARM\matlab_sim')
results = run_chassis_algorithm_sim;
```

输出：

- `chassis_algorithm_sim_result.png`：关键通道对照图；
- `sim_summary.csv`：倍率、腿长速度误差、误离地次数和饱和比例。

这是控制算法级回放，不是假装完整的整车多体动力学。工程中没有完整的车体惯量、电机电气参数、轮地接触和摩擦参数，因此本脚本精确验证“代码中的估计与控制链路”，而不虚构缺失的物理参数。后续把实车记录的四个关节角/角速度、IMU 和轮速接入相同函数，即可做逐采样实车回放。
