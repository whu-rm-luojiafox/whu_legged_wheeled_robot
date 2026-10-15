# 006 工程框架与腿部数学整理

日期：2026-10-16。

## 用户反馈与目标

上一轮把参数应用、参考回填拆成独立 application 文件，模块按小功能分得过细；真正较大的五连杆正解、雅可比和 VMC 仍在任务里。此次按完整功能重新划分，主循环只展示控制顺序，总工程说明放在根目录 README。

## 文件合并与去向

| 原文件 | 当前去向 |
| --- | --- |
| `application/chassis_control.c/.h` | 状态与输入回填收回 `chassis_task.c`，函数作为任务私有实现，不再保留额外应用接口 |
| `application/chassis_tuning.c/.h` | 在线变量和应用逻辑收回任务，默认 PID 参数及变量声明在 `chassis_config.h` |
| `chassis_odometry.c/.h`、`chassis_position.c/.h` | 合并为 `chassis_motion.c/.h`：轮速里程和停车目标是同一平移流程 |
| `chassis_balance_reference.c/.h`、`chassis_lqr_feedback.c/.h`、`chassis_feedforward.c/.h` | 合并为 `chassis_balance.c/.h`：姿态参考、反馈、前馈在同一平衡控制文件中分段 |
| 任务内 `Forward_kinematic_solution()`、`get_jacobian_element()`、VMC 公式 | 提取到 `chassis_leg_math.c/.h` |

原类型和 `tune_*`、`chassis_odometry` 等全局观察名保留，不为旧文件再加转发头。历史记录 001—005 中的文件路径描述的是当时结构，新位置以上表和总 README 为准。

## 五连杆数学接口

- `chassis_leg_forward_kinematics()`：输入第一/第二电机角度和角速度，更新局部腿长、腿角及导数。删除了原完全相同的 `ce` 两侧分支和无意义参数。
- `chassis_leg_inverse_jacobian()`：一次返回 `n11/n12/n21/n22`，不再四次调用带 `element_type=1/2/3/4` 的查询函数。
- `chassis_leg_map_wrench()`：输入逆雅可比、支撑力、模型腿力矩及左右腿标志，输出两个电机的支撑/平衡分量。保留右腿坐标镜像和原奇异矩阵处理。

机械尺寸从任务宏移到数学实现顶部，名称改为第一主动杆、第一连杆、第二连杆、第二主动杆和髋间距，单位 m。数值保持 0.21/0.25/0.25/0.21/0。`chassis_leg_posture_t` 移入数学头文件，原浮点字段布局保持。

第一/第二电机在左腿为 1/2，右腿为 4/3。第一关节的角度、速度取负仍在正解中完成；机身局部腿角和对地腿角继续分开。VMC 仍保留分量观察字段与最终电机限幅。

## 周期与参数应用

主循环不再铺开 RTOS tick 转换和三个 PID 的参数同步。这些工作归到已有 `chassis_feedback_update()` 开头，然后才读取反馈，保持每拍原来的“参数应用 → 反馈 → 模式 → 目标 → 力矩 → 输出”顺序。

保留实测 dt，因为任务中电机发送等待和循环末尾等待共同决定周期，不能用末尾 delay=2 ms 代替。初始化阶段调用反馈时尚未建立上一个周期基准；与原先 dt=0 一样，不作为有效里程积分样本。初始化完成后重新记录 tick，正常周期使用无符号 tick 差计算。

仅一处调用的姿态参考更新包装函数也已撤掉，直接归入完整的 `Chassis_Torque_Calculation()` 阶段。未改变在线参数值、默认参考、转向门限、PID 增益、跳跃/攀爬/自救状态机和电机发送等待。

## 总 README

新增启动顺序、线程入口、生产/消费数据的位置、底盘每拍函数表、完整模块职责、目录树和“要改某功能看哪里”的索引。明确目前实车是 LQR，MPC 为独立验证路径；打滑模块为空，云台实际入口仍在 application。

## 验证

- `cmake --build build/lqr-follow-review --parallel 2`：ARM 编译/链接通过，不覆盖板上旧固件的匹配 ELF。原有 `CONSTACTING_LEGS` 未覆盖及 `NDEBUG` 重定义警告仍存在。
- 原跟随、运动检查迁移到合并后的文件；新增 `chassis_leg_math_checks`，通过位置差分验证速度，通过逆雅可比验证关节速度，通过虚功验证左右腿电机映射，并检查奇异矩阵输出。三个主机检查目标构建成功，`ctest --preset host -R '^chassis_(follow|motion|leg_math)_checks$' --output-on-failure` 为 3/3 通过。
- 本地 20000 组输入姿态与原数学实现对比：覆盖原 `ce=0/1` 两个分支、全部正解输出与四个逆雅可比元素，全部通过。
- 主机差分测试使用非退化姿态与浮点容差；不将靠近折叠姿态的单精度差分误差当作实车故障，也不据这些软件检查宣称全机械行程有效。
- 七个收回任务的控制衔接函数与整理前对比，计算语句保持一致。总 README 和日志索引中的本地链接已核对，`git diff --check` 无空白错误。

此轮未连接探针、未烧录，未新增实车验证。`build/lqr` 的旧 ELF 保留；新源码对应 review 构建。pitch=0 仍是此前 RAM 标定，默认值未在本轮固化。
