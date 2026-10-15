# Chassis model and validation tools

Build commands and architecture: [project README](../../README.md).

| Path | Purpose |
| --- | --- |
| `model/export_fixed_leg_model.m` | Export a fixed-leg continuous/discrete model |
| `model/generated/` | Exported model data |
| `mpc/generate_solver.py` | Generate MPC caches and reproducible runtime patches |
| `mpc/run_host_checks.py` | Build native targets through root CMake and run CTest |
| `mpc/host_probe.cpp` | Diagnostic executable with solver internals |
| `mpc/board_probe.c/.h` | DWT measurement record for the standalone MPC firmware |
| `simulation/compare_lqr_mpc.py` | Independent QP comparison, failure guards and linear rollout |

Run from the project root:

```powershell
python tools/chassis/mpc/run_host_checks.py
# Only after changing model/solver parameters:
python tools/chassis/mpc/run_host_checks.py --regenerate
```

Builds and reports go into `build/`, including `build/host/reports/comparison.json`.
The obsolete MATLAB LQR replay is in the sibling legacy archive, as it
depends on firmware parameters that no longer exist.

Current MPC model: both legs 0.24 m, prediction sample 4 ms, 10 states,
4 inputs, 8 state knots, 7 input intervals, rho=1, 200 iterations and 0.001
tolerance. Model torque bounds are +/-6 N*m for wheels and +/-12 N*m for
virtual leg torques. The latter are not physical joint motor limits.
Terminal weight is Pinf-rho*I, not the model export's Pd.

The native checks cover 24 double/float, bounded/unbounded QP cases,
5 non-convergence guards and a 2-second linear rollout. Host timings do not
represent F407 timings. The C API is single-instance, non-reentrant and
requires heap allocation. A failed solve sets valid=0 and clears command;
candidate is diagnostic only. Live state adaptation, moving references,
variable leg lengths, VMC integration and on-vehicle MPC control remain
outside this experiment.

## 云台跟随模块与后退问题排查（2026-10-15）

### 跟随模块怎么读

入口是 `src/module/chassis/chassis_follow.c` 的 `chassis_follow_step()`。
任务层 `chassis_follow_reference_update()` 负责传感器/指令映射和输出回填，
模块自身只生成 `angle_ref_rad` 和 `rate_ref_rad_s`，力矩仍由原 LQR 计算。

1. 校验使能、实测 `dt`、输入和在线参数；首次调用仅建立当前航向基准。
2. 控制优先级为：手动转向、手动松杆制动、移动自动跟随。
3. 自动跟随以速度和原始角差的滞回决定是否启动，对连续角差滤波，
   然后由 P 生成目标转速，同时施加最高转速和制动限速。
4. 自动转速按实测 `dt` 限加速度；手动指令直接响应，松杆用指数衰减。
5. 自动转向使用 `实际 yaw + 有界超前时间 * 指令转速`；指令归零后捕获航向，
   此后保持该航向。手动转向和手动制动不加超前。

角差为 `wrap_deg(zero_deg - move_deg - relative_deg)`。编码器角差是度，
转向参考是弧度；P 的单位为 `(rad/s)/deg`，不要当成 `(rad/s)/rad`。
`active == 0` 不一定代表转速归零，应同时查看 `rate_ref_rad_s` 和 `holding`。
在自动跟随退出时，滤波状态回到原始角差，所以调试曲线上的 `error_deg`
并非始终来自连续滤波。

本次结构整理保留状态结构体成员、布局和算法。用相同输入分别运行旧、新实现
100000 拍，全部状态及输出逐拍完全相同；原有 host 检查也通过。
另将已实车试调、用户反馈摆头减轻且转向可接受的一组参数写入编译默认值：
P=0.16、加速度=4.5 rad/s²、超前=0.06 s；最高转速仍为 1.5 rad/s。
这里是源码默认值，只有烧录对应固件才会在上电后生效。

### 持续后退：修改前的代码事实

下列是修改前的静态检查结果，不是已经确定的实车故障根因。
后续模块化修正见下方“里程、位置锁与前馈试验版”。

| 位置 | 代码现状 | 可能的影响 |
| --- | --- | --- |
| `chassis_feedback_update()` | `foot_speed_KF` 只是左右轮速平均，没有 KF | 名称容易误导，不能认为已经抑制轮速噪声或滑移 |
| 同上 | 固定按 0.002 s 积分；任务末延时 2 tick，发送中另延时 1 tick，tick 为 1 ms | 距离随真实周期产生比例误差；应记录实测 dt，而非直接把常数改成 0.003 |
| 同上 | `abs(speed) >= 1.6 m/s` 时整拍不累计距离 | 速度仍进入 LQR、距离却冻结，快速运动和停车时状态不一致 |
| `Target_Value_Set()` | 任意非零速度指令都解除位置锁，没有平移死区 | 微小负指令能持续请求后退，同时取消位置保持 |
| 同上 | 指令归零立即捕获位置，不等实际速度降下来 | 前进松杆后的惯性位移会变成位置误差，可能随后倒回松杆点 |
| 同上 | 零速可先进入位置锁分支，优先于 `NO_FORCE`；失能/离地没有统一清掉捕获状态 | 失能期间推动车体或轮空转后，再使能/落地可能仍追旧位置目标 |
| `uart_receive.c` | 收包时间被记录，但无人使用它做运动指令超时；只靠帧头与固定长度收包，无 CRC/数值检查 | 串口断流时旧负速度可能一直保留；错误数据也可能进入指令 |

当前 `FORWARD_SPEED` 为 0，没有额外固定后退速度。
轮速正负号、减速比和轮径还需推车/标尺验证，不能凭后退现象直接反转符号。
现有里程是轮速平均值的积分，不是 IMU 融合定位，也没有滑移补偿。

### 质心偏置与里程如何区分

`OFFEST_PITCH=0.1072 rad`（约 6.14°），`OFFEST_ANGLE=0.1172 rad`（约 6.72°）。
两者都参与 LQR，因此调整它们会同时改变姿态参考与轮端平衡力矩。
地面系腿角已经包含 pitch；普通站立时两种参考之差只有 0.01 rad（约 0.57°），
不能简单把二者理解为互不相关的质心补偿。
物理质心是否偏后仍需实测；轮速为零不保证这两组参考对应真实静态平衡。
位置反馈有效且未饱和时，恒定偏置也可能表现为新的位置平衡点，
不能单凭“偏后”就断言它解释了持续后退。

重新接电后先失能核对固件和参数，再在可控测试中同步记录：

- 接收的 `vx_set/vy_set/wz_set`、`foot_speed_set`：指令是否真归零。
- `position_lock_flag/state`、`target_distance_set`、`foot_distance_set`、
  `foot_distance_K`：是否追旧目标，位置误差是否随倒退增长。
- 左右轮速、`foot_speed_KF`、实际任务 dt：方向和里程比例是否正确。
- pitch、地面系/本地腿角、轮端 `balancing_torque` 和 `moving_torque`：
  区分姿态偏置产生的驱动与位置/速度反馈产生的驱动。

修正顺序建议：先统一位置锁生命周期、检查零指令/串口新鲜度，再修复实测 dt
积分与高速冻结，最后独立标定质心平衡点和力矩前馈。串口超时应采用明确的
平移/转向停车策略，不能直接凭超时切断支撑力矩。各项分开验证，避免同时
改变反馈尺度和姿态平衡点后无法判断效果。

### 里程、位置锁与前馈试验版

任务层只负责接线；以下三个模块均不依赖 HAL/RTOS，可在 host 上验证。

| 模块 | 职责 | 调试状态 |
| --- | --- | --- |
| `chassis_odometry.c/.h` | 用实测 dt 积分平均轮速，移除 1.6 m/s 冻结；非有限输入或 dt 超过 50 ms 时跳过该样本 | `chassis_odometry`、`chassis_control_dt_s` |
| `chassis_position.c/.h` | 平移指令 0.02 m/s 死区；停车制动后捕获当前位置；失能、离地、手动自旋、跳跃、攀爬、自救时释放位置锁 | `chassis_position`，旧 `position_lock_*`、`foot_distance_set` 同步回填 |
| `chassis_feedforward.c/.h` | 可选的水平质心偏置重力矩补偿，分配给左右虚拟腿，再经原 VMC 和关节限幅 | `chassis_feedforward`、`tune_com_ff_enable`、`tune_com_ff_moment` |

位置锁在实际轮速不超过 0.06 m/s 并持续约 0.15 s 后捕获；制动期间位置参考
跟随当前里程，LQR 零速反馈继续制动。若一直缓慢漂移，最多等待约 0.50 s
也会捕获位置，避免一直取消位置反馈。捕获后目标固定，发生位移时会正常纠偏。
特殊动作期间释放捕获状态，恢复正常平衡后重新制动/捕获，不追此前位置。

里程、位置锁、跟随与前馈统一使用每拍开始时的实测 dt。此次没有改腿长
导数、PID 的时间尺度、轮速正负号和轮径/减速比，也没有加入 IMU 融合定位。
UART 的超时停车和收包校验仍待单独实现；本轮并未消除旧串口指令持续有效的风险。

前馈默认 `tune_com_ff_enable=0`、`tune_com_ff_moment=0`。非零参数的单位为
带控制方向符号的 `kg*m`，表示水平偏置的一阶质量矩，**符号必须先标定**。
每腿目标为 `0.5 * 9.81 * moment * cos(pitch)` N*m；不包含垂直质心项，
也不是完整的全姿态逆动力学补偿。试验范围限制为 `|moment| <= 0.10 kg*m`、
每腿 `|torque| <= 0.50 N*m`、变化率不超过 `0.50 N*m/s`。
失能、特殊动作、离地、异常 dt/参数或 `|pitch| > 0.35 rad` 时立即清零；
仅关闭在线使能开关时按同样变化率衰减至零。

模块不向轮端直接追加恒定驱动力。原有腿角/机身角偏置保留，前馈试验前要
先观测静态平衡力矩，避免重复补偿。应先验证前馈关闭时的停车/重使能行为，
再失能设定一个已明确方向的小补偿量，单独试验前馈效果。

```powershell
cmake --build --preset host --target chassis_follow_checks chassis_motion_checks
ctest --preset host -R '^chassis_(follow|motion)_checks$' --output-on-failure
```

新增检查覆盖实测周期、2 m/s 高速里程、倒车、纯转动、调试暂停、松杆滑行、
漂移超时、失能后移动再使能，以及前馈默认关闭、方向、限幅、斜率和失效门控。
host 测试通过只能说明这些算法场景符合预期，不能替代实车验证稳定性。

### 朝前角标定与平衡参考模块

用户将云台摆到朝前位置后，连续五次读取的相对编码器角均为
123.7939453125°。新编译默认 `CHASSIS_GIMBAL_FORWARD_DEG` 采用该值，
并提供独立 RAM 标量 `tune_gimbal_forward_deg`；任务校验其为有限的
[-180, 180] 度值后回填旧观察项 `relative_angle_init`。
此次标定不清编码器，也不修改云台绝对角/陀螺仪反馈。
直接写旧结构体成员被 EmberProbe 的 `WRITE_NOT_ALLOWED` 拒绝，未生效；
本次朝前角需要通过新固件及该独立标量生效。

`chassis_balance_reference.c/.h` 替代原 `OFFEST_PITCH/OFFEST_ANGLE` 宏：

- `tune_balance_pitch_rad`：机身 pitch 参考偏置，第一步默认从 0.1072
  减至 0.0972 rad，约降低 0.57°，尚未归零。
- `tune_balance_leg_local_rad`：腿相对机身的参考偏置，默认保持 0.01 rad。
- `chassis_balance_reference.leg_ground_rad`：上述两项的和，作为原 LQR
  对地腿角参考偏置。首轮为 0.1072 rad；避免改变 pitch 时无意改变本地腿角目标。

模块保留原始 IMU/对地腿角反馈，仅改变控制参考。参数范围各为 +/-0.15 rad；
使能时每项参考的变化率不超过 0.01 rad/s；失能时接受有效标定请求，
异常请求保持已有参考。`chassis_balance_reference` 可观察实际应用值和有效性。
把请求设回 pitch=0.1072、local=0.01 可恢复原参考关系；模块不自动识别真实质心。

新固件前馈仍默认关闭。负向 -0.01 kg*m 在主观反馈中有所减轻，但单次完整记录
仍出现约 1.10 m 的峰值后回退，尚未证明它能替代姿态偏置。
先核对朝前零点、正常站立和第一步 pitch 调整，之后再用一致速度对比前馈。
云台 PID 参数和绝对角误差算法本轮未改，旋转稳定性需要实测后再判断。

`normal_move_scale`、`chassis_control_dt_s`、`uart_data` 也分配了独立数据段，
解决 ARMClang 合并全局变量后导致 EmberProbe 类型/结构体布局绑定不完整的问题。
新增 host 检查覆盖旧参考关系复现、参考缓变、本地/对地角一致性、零 pitch 请求、
异常输入保持，以及新朝前角处不请求自动转向。

### LQR 对侧腿反馈诊断

在固件 `8dc10695…da36ca1`、pitch 参考 0.0972 rad 下试验 -0.02 kg*m 前馈：
连续零平移/转向指令、双腿触地、位置锁固定的记录中，轮速里程仍后退约
1.29 m，用户反馈后漂差不多。该次试验已恢复 `tune_com_ff_enable=0`、
`tune_com_ff_moment=0`，读回实际输出为零。不能据此否定所有质心补偿方案，
但本次没有继续扩大常值前馈的实测依据。

代码审查确认，原计算在每行都遗漏对侧腿的两项反馈。以当前 K_Fit 在双腿
0.24 m 处为例，左虚拟腿自身角度系数约 16.031，对侧角度系数约 -11.795，
后一项不是可以忽略的小量。它是状态反馈项，不能用一个常值力矩替代。

用已导出的 0.24 m 固定腿长模型作离线检查，完整拟合增益与当前省略实现的
连续闭环最大特征值实部约为 -0.704 / -0.773，4 ms 离散谱半径约为
0.99719 / 0.99691。两者在该理想模型点都稳定，**不能把遗漏项直接认定为
实车后漂或失稳根因**；该检查不包含实际机械偏置、采样延时和接触切换。
计算记录在 `build/follow-review/lqr-matrix-audit.json`。

新增 `chassis_lqr_feedback.c/.h` 统一状态误差顺序和矩阵计算，任务层负责
电机镜像、离地/跳跃分支及输出限幅。对侧腿权重 `tune_lqr_cross_leg_scale`
默认 0，复现原计算；1 表示完整矩阵，范围外或非有限值回退为 0。
只有普通平衡控制条件成立时才允许非零权重。`chassis_lqr_feedback.cross_leg`
输出四行未乘权重的对侧腿力矩诊断量，可先观察再决定是否逐步引入。

host 对照覆盖 200 组不同腿角/角速度与运动误差：0 权重对照原公式、
1 权重对照完整矩阵乘法、0.1 权重验证增量，另验证非法权重回退。
该重构只在 `build/lqr-follow-review` 构建验证，未写入芯片；当前配置的
`build/lqr/chassis_firmware.elf` 仍与板上的 8dc10695…da36ca1 固件一致。

### 机身水平与位移保持：独立对地腿角参考

为验证“用静态力矩补偿减小机身 pitch 偏置，同时保持位置”，平衡参考接口
进一步改为两个独立请求：`tune_balance_pitch_rad` 和
`tune_balance_leg_ground_rad`。前者为机身参考偏置，后者为对地腿角参考偏置；
原 `tune_balance_leg_local_rad` 不再是可写参数，
`chassis_balance_reference.leg_local_rad` 保留为两者差值的观察量。

原因是：固定本地腿角再降低 pitch，会同时降低对地腿角目标，改变支撑几何
参考；将机身与对地腿角分别标定，才能在减小机身倾斜时独立保留/调整腿的位置。
这只保证参考坐标一致，不保证真实支撑几何或位置一定不变，仍需实车确认。
默认有效参考保持上一版的 pitch=0.0972、对地腿角=0.1072 rad。
后续第一个计划点是 pitch=0.0872、对地腿角仍为 0.1072 rad。

两项参考继续独立限速 0.01 rad/s。pitch 请求限 +/-0.15 rad，对地腿角限
+/-0.30 rad，且二者差值限 +/-0.20 rad；无效请求保持当前输出。
测试覆盖降低 pitch 至零时对地腿角保持不动、派生本地角正确、无效值与
调试暂停保持，以及默认有效参考与上一版相同。

静态力矩前馈仍只作用于虚拟腿的模型坐标力矩，默认关闭，LQR 的位置/速度
反馈和位置捕获模块继续工作。前馈需按实际平衡点标定，不能把原姿态参考偏置
简单乘 K 再当作“机身变平”的补偿：若对全部执行器完整等价迁移，控制律只是
改写了形式，物理平衡姿态并不会因此改变。只补关节端还无法等价替代原来
同时影响轮端和关节端的姿态反馈项。

本轮没有新增位置积分、自动学习前馈或启用对侧腿耦合，避免一起改变多条反馈。
位置是否稳定须同时观察零指令下的轮速、固定位置目标误差、实际 pitch、腿角、
前馈与反馈力矩，以及输出是否限幅。若腿/机身参考与实际质心不匹配，单个常值
前馈不能保证同时消除姿态和位置误差。

2026-10-15 独立腿角试验记录：RAM pitch=0.0872、对地腿角=0.1172 rad，
前馈关闭、对侧腿反馈权重为 0。125 个样本跨度 20.677 s，采样期间均为
使能、平衡就绪、双腿接地、平移/转向指令为零且位置锁保持。轮速积分位置
相对锁定目标的误差由 -0.9086 m 缩小至 -0.5741 m，期间位移 +0.3345 m；
这不是本轮新增的 0.91 m 后退距离，因为采样开始已经存在位置偏差。
实际 pitch 均值 0.11078 rad，左右对地腿角均值 0.19305/0.19756 rad。
左轮模型坐标平衡反馈均值 +1.1592 N*m、运动反馈均值 -1.1118 N*m，
仅为非原子运行采样的分量统计，不能据此单独判断静态平衡或最终执行器输出。
用户反馈“差不多”，因此不将增加腿角认定为有效改善。随后已读回确认失能。
原始记录：build/follow-review/stand-pitch00872-ground01172.jsonl。
