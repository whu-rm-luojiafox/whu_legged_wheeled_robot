# 底盘改动与调试记录

这里记录每次修改的目的、代码位置、参数、实测感受和遗留问题。历史记录不表示当前板上参数；RAM 调参与编译默认值必须分别看。

## 阅读代码的顺序

1. [根目录 README](../README.md)：先看工程启动、任务和数据流，以及底盘一拍的调用顺序。
2. [chassis_task.c](../src/application/chassis_task.c)：从 `chassis_task()` 顺着看；参数和状态在顶部，模式/反馈/力矩流程在中部，私有衔接函数在后部。
3. [chassis_config.h](../src/application/chassis_config.h)：查 PID 编译参数和在线 `tune_*` 声明；变量定义已收回任务内。
4. [algorithm](../src/components/algorithm) / [controller](../src/components/controller)：分别读五连杆数学、平衡计算和独立 MPC；[module/chassis](../src/module/chassis) 保留跟随、停车与动作状态机。文件合并见 006，当前归类与 pitch 默认值变更见 007。

| 模块 | 负责什么 | 关键观察量 |
| --- | --- | --- |
| `chassis_follow` | 移动时跟随、手动转向优先、减速和停止航向保持 | `error_deg`、`angle_ref_rad`、`rate_ref_rad_s`、`heading_locked` |
| `chassis_leg_math` | 五连杆正解、完整逆雅可比、左右腿 VMC | `chassis_move` 左右腿的局部几何与力矩分量 |
| `chassis_motion` | 轮速积分与停车位置捕获 | 保留全局名 `chassis_odometry`、`chassis_position` |
| `chassis_balance` | 独立姿态参考、LQR 分量、实验质心前馈 | 保留全局名 `chassis_balance_reference`、`chassis_lqr_feedback`、`chassis_feedforward` |

数据流：反馈与轮速里程 → 模式检测 → 平移指令/停车位置/yaw 跟随 → 攀爬及姿态参考 → 腿长/roll/LQR → 虚拟腿前馈 → VMC → 自救/功率限制 → 电机输出。

LQR 输出数组的顺序为左轮、右轮、左虚拟腿、右虚拟腿。模块使用模型坐标，右轮电机方向和右腿镜像仍由任务的原输出路径处理。`cross_leg` 是未乘试调权重的诊断量，不能直接当作实际输出。

## 记录索引

| 记录 | 内容 |
| --- | --- |
| [000 烧录与调试基础](000-flash-debug.md) | ELF 调试信息、异常记录、探针连接和变量解析 |
| [001 腿长与 roll](001-leg-roll.md) | 积分参数、积分限幅、roll 支撑分配及反馈 |
| [002 跟随与响应](002-follow.md) | 跟随模块、P/加速度/超前时间试调、正前方标定 |
| [003 里程、姿态与前馈](003-balance-position-feedforward.md) | 回退问题、位置锁、独立姿态参考和前馈试验 |
| [004 可读性整理](004-control-readability.md) | 本次文件拆分、验证范围与使用注意事项 |
| [005 逻辑与命名简化](005-control-simplification.md) | 删除旧 yaw PID、放宽临时参数门槛、新旧命名对照 |
| [006 工程框架与腿部数学](006-framework-and-leg-math.md) | 收回过细应用层，合并功能模块，提取正解/雅可比/VMC，完善总 README |
| [007 算法与功能边界](007-component-boundaries-and-zero-pitch.md) | 算法/控制器归位，MPC 路径迁移，源码默认 pitch 固化为零 |
| [记录模板](TEMPLATE.md) | 后续每轮改动复制一份，按编号递增 |

000—003 根据 2026-10-15 调试会话回填，于 2026-10-16 整理。没有可靠记录的内容不补猜测值或时间。

## 本轮整理时的参数快照

以下 RAM 值来自最近一次读回（2026-10-15 会话末），不是持续在线状态。

| 参数 | 源码编译默认 | 最近 RAM / 实测状态 |
| --- | --- | --- |
| 腿长 PID | P=300、I=0.075、D=300，积分限幅 2.5、输出限幅 50 | 已纳入试调固件 |
| roll PID | P=300、I=0.08、D=10，积分限幅 2.5、输出限幅 10 | 已纳入试调固件 |
| 跟随 | P=0.16，最高 1.5 rad/s，加速度 4.5 rad/s²，滤波 0.04 s，超前 0.06 s | 用户反馈摆头减轻、转向可接受 |
| 云台朝前编码器零点 | 123.7939453125° | 已纳入试调固件 |
| 机身 pitch 参考偏移 | **0 rad（007 已固化源码，尚未烧录）** | **最近 RAM 为 0，用户反馈“挺好的”** |
| 对地腿角参考偏移 | **0.1072 rad** | **RAM 为 0.1172 rad** |
| 质心前馈 | 关闭，moment=0 kg·m | 关闭 |
| 对侧腿反馈权重 | 0 | 0 |

007 已将源码 pitch 默认值改为 0，但尚未烧录。当前板上旧固件复位仍会恢复其原默认值；只有烧录新版后复位才保持源码新默认。零参考偏移表示不再人为添加机身倾角，不代表实际机身 pitch 能严格为 0，也不表示后漂和云台抖动已经解决。

在线写入按当前 ELF 的符号与类型解析。整理后重新链接可能改变 RAM 地址，不使用历史地址直接写。板上仍运行旧固件时，调试器继续使用与其匹配的旧 ELF。
