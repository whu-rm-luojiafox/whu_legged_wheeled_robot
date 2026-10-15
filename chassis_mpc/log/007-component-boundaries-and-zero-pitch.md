# 007 算法、控制器与功能边界；pitch 默认归零

日期：2026-10-16。

## 目的与分类依据

用户指出 balance 和 leg_math 主要做控制/数学计算，放在功能模块中不合适。本次按输入输出和依赖归类，而不是按文件是否带 chassis 前缀归类。

| 层 | 负责什么 | 当前例子 |
| --- | --- | --- |
| application | 硬件状态适配、运行顺序、模式选择、电机输出 | chassis_task、gimbal_task、INS_task |
| module | 带运行条件、接管规则和动作阶段的功能行为 | 跟随、停车、接地、越障、自救、功率接入 |
| components/controller | 数值控制计算，输入状态/参考，输出控制量 | PID、chassis_balance、独立 MPC |
| components/algorithm | 几何、矩阵、滤波等数值算法 | 五连杆正解/雅可比/VMC、LQR 增益拟合、Kalman |
| service / protocols / bsp / platform | 通信、数据格式、设备与板级接入 | CAN、板间协议、HAL 初始化 |

跟随和停车虽然包含数值计算，但它们还决定何时跟随、谁接管、何时捕获/释放目标，因此保留为功能模块。没有为每个小计算再建一层文件。

## 实际改动

1. `chassis_leg_math.c/.h` 从 `src/module/chassis/` 移到 `src/components/algorithm/`。
2. `chassis_balance.c/.h` 从 `src/module/chassis/` 移到 `src/components/controller/`。
3. 独立 `chassis_mpc/` 从功能目录整体移到 `src/components/controller/`；7 个文件内容哈希全部保持。同步 CMake 公共/私有 include、缓存生成脚本和独立 QP 仿真脚本中的路径，未重新生成缓存。
4. 更新固件显式源文件清单和 host 检查的源文件/include 路径，不保留旧位置的转发文件。
5. 移除内容为空、无引用的 `chassis_slip.c/.h` 以及构建清单项；当前没有打滑控制实现。
6. 删除仅有定义、全工程无引用的 `suspend_LQR`、`coordinate_PD`、`yaw_PD_test`、`alpha_dx`、`alpha_dv`。实际参与计算的跳跃参数和 `alpha_da` 保留。
7. 根 README 的分层、目录树、职责表与调试日志索引同步更新。

新位置的腿部数学、平衡控制、独立 MPC 不依赖 HAL、FreeRTOS 或 application 头文件；由任务把外设/业务状态转换为它们的数值输入。

## pitch 源码默认值变更

`src/components/controller/chassis_balance.h`：

```c
#define BALANCE_PITCH_DEFAULT_RAD 0.0f
```

从 0.0972 rad 改为 0，供 `tune_balance_pitch_rad` 的编译初值及参考初始化共同使用。此前用户认可的零偏移只有 RAM，本次才写入源码；新固件烧录后复位保持 pitch 零偏移。

对地腿角编译默认维持 **0.1072 rad**，未将此前 RAM 试调的 0.1172 一起固化。前馈默认关闭、moment=0，对侧腿权重=0。没有改 IMU 测量零点、PID/LQR 增益或使能逻辑。

本轮未烧录、未连接硬件，不将“源码已经为 0”表述为“板上旧固件复位也为 0”。

## 重新审视工程后仍需处理的边界

| 发现 | 代码证据及影响 | 建议顺序 |
| --- | --- | --- |
| 业务模块依赖整份任务状态 | contact/climb/recovery 头文件包含 `chassis_task.h`，任务头又导入驱动和云台类型，依赖面较大 | 后续涉及这些功能时，先明确所需状态，再收窄接口；不为搬目录一次性改写所有状态机 |
| 功率函数名称与实际职责不符 | `chassis_power_limit()` 计算 `target_limit`，实际乘的是 `Plimit/Wlimit`，且只修改速度参考；调用发生在力矩合成之后 | 单独做功率行为审查和板端验证，决定实现真实输出限功率还是改名为参考缩放。本轮仅修正文档，不暗改力矩 |
| 电机命令任务名容易误导 | `motor_cmd_task` 当前主要更新超电字段，底盘 CAN 实际发送仍在 `Motor_CMD_Send()` | 下次整理超电业务时再统一命名及线程入口，不与实际底盘发送混淆 |
| 共用状态不是同步快照 | CAN/UART 回调更新数据，底盘/云台任务共享读取；本次结构移动没有提供跨字段一致性 | 针对实测通信问题再选择快照或同步方式，不能靠目录移动声称解决抖动 |
| 部分逻辑仍是历史占位 | `Check_Jump_Preparation_Complete()` 对有效指针直接返回完成；`CONSTACTING_LEGS` switch 警告仍在 | 跳跃功能专项检查时补实际条件与回归，不在结构调整中悄悄改变触发行为 |
| 参数归属还未完全统一 | PID 默认宏在 chassis_config，跟随/平衡/机械参数在对应组件头/实现中 | 当前 README 给出明确位置；若以后支持多台车，再引入板型/车型配置，当前不增加泛化配置层 |

以上是实际代码审查结果与后续建议，未记录为本轮已修复。

## 验证

- `cmake --build --preset host`：完整主机构建通过。
- `ctest --preset host --output-on-failure`：**4/4 通过**，含跟随、运动/默认 pitch 归零、腿部数学及 MPC 独立 QP 对照。
- `cmake --build build/lqr-follow-review --parallel 2`：完整 LQR 固件链接通过。保留当前板上旧固件匹配的 `build/lqr/chassis_firmware.elf`。
- 新建独立 `build/mpc-structure-review`，按 mpc-probe preset 配置并构建：ARM MPC 测量固件链接通过。
- 原有 switch 和 NDEBUG 警告仍存在；目录调整不等于实车时序/闭环效果验证。

后续烧录整理后的 LQR 固件应使用与其匹配的 ELF，不能继续沿用旧地址写 RAM；本轮没有执行烧录。
