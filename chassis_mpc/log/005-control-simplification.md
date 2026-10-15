# 005 逻辑与命名简化

日期：2026-10-16。

## 目的

用户认为新增逻辑和保护判断偏多，部分名称不直观。本轮删去已不参与控制的兼容数据，放宽初期试验时的临时调参门槛，让函数名能看出动作。常用增益和编译默认值保持不变。

## 删减内容与行为差异

| 项目 | 原先 | 本次 |
| --- | --- | --- |
| 旧 yaw PID 对象 | 跟随已由独立 P 外环计算，仍初始化 `chassis_yaw_pid` 并每拍写入仿 PID 观察数据 | 删除初始化、结构体成员及回填；跟随状态和真实参考统一从 `chassis_follow` 读取 |
| 跟随 PID 宏 | 带 PID 名称，含未使用的 I/D/积分限幅宏 | 只保留 `FOLLOW_KP_RAD_S_PER_DEG`；转速上限继续使用模块的限速参数 |
| 跟随参数门槛 | P≤0.4、速度上限参数≤3、加速度参数在 0.5—10、滤波/超前时间≤0.2，否则整环重置 | 取消这些临时上限与最小加速度 0.5 门槛；保留有限值、非负参数和加速度必须大于 0 的要求 |
| PID 积分试调门槛 | Ki>0.1 或积分限幅>3 时整组忽略 | 接受有限、非负值；积分限幅最多取该 PID 总输出限幅，已有积分输出同步夹回范围 |
| 停车位置释放 | 无控制条件、输入异常、移动指令分别有重复释放分支 | 合并为一个释放分支，捕获时机和目标保持行为不变 |
| 跟随转速更新 | 只有一处调用的包装函数 | 合并回主流程，手动、松杆衰减、自动转向顺序不变 |
| 使能/触地条件 | 普通平衡与跟随重复检查 | 共用 `chassis_is_enabled_on_ground()`；各自的运动模式和特殊动作条件仍分别保留 |
| 零速偏置宏 | `FORWARD_SPEED` 固定为 0，仍加进速度误差 | 删除恒为零的加项 |
| LQR 长成员访问 | 每行重复 `chassis_posture_info.chassis_posture_L/R` | 用 `posture`、`left`、`right` 局部只读指针表达反馈来源 |

放宽调参范围意味着以前被忽略的请求现在可能生效，这是明确的行为变化；默认参数和原有效范围内的算法不变。保留失能/离地撤力、特殊动作控制权切换、电机/虚拟腿力矩限幅、实际转速及加速度限幅、超前角限幅、非有限数据与异常 dt 处理。姿态参考的范围与缓变保持不变。

## 函数命名对照

| 旧名 | 新名 | 含义 |
| --- | --- | --- |
| `chassis_normal_control_allowed` | `chassis_is_balancing_on_ground` | 普通接地平衡，且未被攀爬/自救接管 |
| `chassis_position_reference_update` | `chassis_update_stop_position` | 更新停车位置目标 |
| `chassis_follow_reference_update` | `chassis_update_yaw_follow` | 更新 yaw 跟随参考 |
| `chassis_balance_reference_update` | `chassis_update_balance_reference` | 更新独立的机身与对地腿角参考 |
| `chassis_odometry_feedback_update` | `chassis_update_wheel_odometry` | 更新轮速里程 |
| `chassis_lqr_feedback_update` | `chassis_calculate_lqr_feedback` | 计算 LQR 反馈分量 |
| `chassis_feedforward_apply` | `chassis_apply_com_feedforward` | 叠加质心偏置前馈 |
| `chassis_tuning_apply` | `chassis_apply_pid_tuning` | 应用腿长/roll 在线积分参数 |

## 跟随 Watch 名称对照

| 旧观察项 | 新观察项 |
| --- | --- |
| `chassis_follow.active` | `chassis_follow.auto_following`：正在自动跟随 |
| `chassis_follow.manual` | `chassis_follow.manual_turning`：正在手动转向 |
| `chassis_follow.manual_braking` | `chassis_follow.manual_stopping`：手动松杆后的减速阶段 |
| `chassis_follow.holding` | `chassis_follow.heading_locked`：已锁定停止航向 |
| `chassis_move.chassis_yaw_pid.fdb` | `chassis_follow.error_deg`：外环角差 |
| `chassis_move.chassis_yaw_pid.out` | `chassis_follow.rate_ref_rad_s`：实际交给 LQR 的转速参考 |

`tune_*` 参数名保持不变，便于继续在线调参。由于删去了 `chassis_move` 中的旧 PID 成员，后续固件的结构布局和 RAM 地址会变化。新名称适用于本次源码构建；板上旧固件仍须使用旧 ELF 和原观察字段，不应提前切换新 ELF 调试。

## 验证与状态

- 独立目录 ARM 构建：`cmake --build build/lqr-follow-review --parallel 2` 通过。原有枚举分支和 NDEBUG 重定义警告仍存在。
- host 跟随/运动检查：2/2 通过。跟随检查补充了超出旧试调门槛的合法参数，以及零加速度、负 P 的拒绝行为。
- 简化前后 100000 步跟随对比：原调参范围内，状态和输出一致；覆盖变 dt、前后移动、手动接管和退出、失能。
- 本地主机专项检查验证了腿长/roll 默认值、正负积分收回、放宽 Ki 门槛、积分限幅不超总输出限幅，以及 NaN 请求不应用。临时检查源码在 `build/follow-review/simplify-before/`，不作为版本库测试入口。

本次没有烧录、没有修改当前板上 RAM 参数。pitch=0 仍是先前 RAM 标定，源码默认值未在本轮改变，也没有新增板端控制效果结论。
