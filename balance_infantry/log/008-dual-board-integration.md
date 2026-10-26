# 008 balance_robot 双板工程融合与模块整理

日期：2026-10-21。

## 目的

将现有底盘工程和 `02Balance_gimbal_task` 融合为一份可分别构建底盘板、云台板的工程，保留底盘原有功能及调参默认值，明确公共与板级边界。工程名改为 `balance_robot`，物理目录仍保留 `chassis_mpc`，避免原调试路径失效。

## 修改

| 范围 | 内容 |
| --- | --- |
| 构建选择 | 新增 `chassis`、`gimbal` preset，由 `ROBOT_BOARD` 选择源码、板级 include、输出和宏；禁止双宏/缺失宏及云台 MPC probe 组合。保留旧 `lqr`、`host`、`mpc-probe` 入口和内部构建目标名。 |
| 云台导入 | `src/gimbal` 保存不同版本的任务、BSP、IMU/EKF、算法、UI；`platform/stm32f407/gimbal` 保存外设、启动接入和 CubeMX 参考。根据原 uvprojx 建立显式源清单；`IMPORT.json` 记录来源、导入前哈希和共用映射。原同级工程未修改。 |
| 共享代码 | 复用相同 BSP/设备实现、公共类型、RGB 灯任务、AHRS/DSP、CMSIS、现有 FreeRTOS；复用底盘 PID，其计算与云台原 PID 相同，额外保留底盘前馈函数。 |
| HAL/RTOS | 云台保留原 HAL 和 USB device 库，避免混用两板 HAL。云台原 FreeRTOS 10.0.1/RVDS 迁移到现有 10.3.1/GCC CM4F 端口；任务顺序、优先级、栈与 24 KiB RTOS 堆保留。底盘 RTOS/外设配置不变。 |
| 板间协议 | 两板共用 `gim_cha_protocol.h`、`robot_link_frames.h`。命令负载 48 B、UART 命令帧 52 B、反馈 8 B，保留原对齐和旧字段别名。两板均编译尺寸/偏移、短枚举、遥控器 packed 布局断言。`SHOOT_BACK` 追加在枚举末尾，原值不变。 |
| ARMClang 兼容 | 云台 `__packed struct` 改为有效的 packed 属性；补齐 SysTick 与 CRC 声明，修正 flash 静态声明冲突和头文件大小写。未新增通信协议功能。 |
| 云台模块化 | `gimbal_command` 集中 RC、键盘边沿、yaw 请求与动作标志；任务保留 pitch 计算、传感器接入及 CAN。参数抽到 `gimbal_config.h`，自动瞄准/输入使能由查询接口提供。修正 UART/自瞄任务句柄赋值对象。 |
| 底盘模块化 | LQR 拟合改为输入左右腿长、显式输出矩阵，不依赖任务/HAL；增加原多项式固定样本回归。行为模块公开头文件仅使用不透明状态声明，实现仍保留旧状态适配。自救只返回接管标志，由任务显式重新映射，去除模块反向调用任务函数。 |
| 工程可读性 | 修正公共 FIFO 的 C++ linkage 和 NDEBUG 重定义；增加双板工作区、VS Code 生成任务、构建隔离检查。README 重写双板结构，原底盘数据流迁到 `docs/chassis-control.md`。 |
| 日志 | 只保留最近 3 次编号记录；`rotate_logs.py` 删除更旧记录并更新索引，保留模板。 |

## 保留的控制约定

- 底盘仍为 LQR；PID/LQR 系数、电机正负方向、pitch 默认 0 rad、对地腿角 0.1072 rad、实验前馈默认关闭均保持。
- 自救前的常规 VMC 及接管后的重新 VMC 顺序保持；只将后一次调用移到任务层。
- 云台指令仍在反馈刷新前更新，每拍仍执行两次原 `shoot_control_loop()`；原 pitch 速度环参数数组中的 D 项仍取 `PITCH_ANGLE_PID_KD`。这些历史行为没有随文件拆分修改。
- 原 `chassis_power_limit()` 仍仅缩放参考且保留原调用时序；跳跃准备检查仍有占位逻辑。它们需要单独的功能改动和上板验证。
- UART 固定长度帧、共享数据访问方式不变；布局一致不代表提供 CRC、通信原子快照或实时性能保证。

## 验证

- `cmake --build --preset chassis`：完整底盘固件链接通过，`build/chassis/chassis_firmware.elf`；ROM 106776 B。
- `cmake --build --preset gimbal`：完整云台固件链接通过，`build/gimbal/gimbal_firmware.elf`；ROM 71620 B。
- `cmake --preset mpc-probe -B build/mpc-integration` 与 `cmake --build build/mpc-integration --parallel 2`：独立 ARM MPC probe 链接通过。
- `cmake --build --preset host` 与 `ctest --preset host --output-on-failure`：**5/5 通过**，含新增 LQR 增益固定样本回归。
- `python tools/project/check_board_builds.py`：底盘 104、云台 111 个翻译单元，检查源码、include、宏及输出隔离通过。
- 旧 `lqr` preset 在 `build/lqr-integration` 配置通过，编译数据库源码集合与 `chassis` 完全相同；未覆盖原 `build/lqr`。
- 宏检查：单个 chassis/gimbal 宏可编译，缺失/双宏均拒绝；未知板名、云台配 MPC probe 均在配置阶段拒绝。
- 原云台与拆分后代码的 11 个控制/指令函数、原云台与共用 PID 的 3 个函数，允许已记录的名称/访问器替换后，函数体归一化对照 **14/14 通过**。这是静态结构回归，未覆盖新 RTOS 的实际调度；报告在 `build/gimbal-import-regression.json`。
- README/指南/日志索引链接检查通过；日志轮换后编号记录为 006、007、008，再次 dry-run 无额外删除。

ELF SHA-256：

| 固件 | SHA-256 |
| --- | --- |
| chassis | `7377ce2cb2f648d14e9eb1eff2ea1090fbf7daa25475ef188977652d34111293` |
| gimbal | `bc7966a0f4c37503b457e4d1a3ea7a80ff30a0086eceb457f50c343a7a5b087d` |

保留的旧警告包括底盘未处理的跳跃枚举/OLED 字符串指针符号、云台原射击逻辑括号/旧注释及旧 HAL 括号提示。未将“编译通过”写成“零警告”。

本轮未连接探针、未烧录、未做实车闭环验证。原 `build/lqr/chassis_firmware.elf` 的修改时间仍为 2026-10-15 22:25；另建 `build/chassis-baseline` 保存修改前底盘构建。新旧 ELF 地址不同，不能用新固件的调试地址操作旧板上程序。

## 后续

分别用两块板对应 ELF 做上板联调，重点验证云台共享 RTOS 的调度/IMU/USB、板间命令及 pitch/yaw 分工，再单独处理功率、跳跃准备及射击更新次数。结构融合不等于这些历史行为已修复。
