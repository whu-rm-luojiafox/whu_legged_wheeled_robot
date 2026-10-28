# balance_robot · 珞珈狐轮腿机器人

一份 CMake 工程，分别构建 **底盘板 chassis** 和 **云台板 gimbal** 固件。名称 `balance_robot` 表达整车工程；物理目录暂保留 `chassis_mpc`，避免已有工具的绝对路径失效。打开 [balance_robot.code-workspace](balance_robot.code-workspace) 可使用新的工作区显示名。

云台代码来自同级 `02Balance_gimbal_task`，已复制到本工程，构建不依赖同级目录。原工程未修改。实车编译统一使用 `chassis` 和 `gimbal` 两个入口；底盘使用 LQR。旧的 `lqr` 重复入口已移除，`host`、`mpc-probe` 仅供主动运行独立算法实验时使用。

## 选择、编译与烧录文件

固件使用 **CMake ≥ 3.21、Ninja、已授权 ARM Compiler 6**；本机以 ARMClang 6.7 验证。现有 AHRS/DSP 二进制库和启动汇编依赖 ARM 工具链，暂不支持直接切换为完整 GNU Arm 固件。

在工程根目录执行：

```powershell
$env:ARMCLANG_ROOT = 'D:/Keil5/ARM/ARMCLANG'

# 底盘板
cmake --preset chassis
cmake --build --preset chassis

# 云台板
cmake --preset gimbal
cmake --build --preset gimbal
```

| preset | 内容 | 烧录/调试文件 |
| --- | --- | --- |
| `chassis` | 底盘 LQR、轮腿控制、底盘侧 yaw 与发射控制 | `build/chassis/chassis_firmware.elf` |
| `gimbal` | 云台姿态估计、pitch、遥控/键鼠、自瞄通信及底盘指令生成 | `build/gimbal/gimbal_firmware.elf` |

固件同目录还会生成 `.hex`、`.bin`、`.map`。在烧录工具/EmberProbe 中选择**对应板的 ELF**；调试和变量读取也使用同一次构建的 ELF。日常 `build/` 只保留 `chassis/` 和 `gimbal/`；旧测试目录、历史 ELF、临时脚本和报告已清理。`host` 与 `mpc-probe` 只有显式运行对应 preset 时才会重新生成实验目录，不能作为整车运行固件烧录。

CMake 缓存变量 `ROBOT_BOARD=chassis|gimbal` 选择不同的**源码清单、main、外设配置和头文件目录**，并自动定义 `ROBOT_BOARD_CHASSIS=1` 或 `ROBOT_BOARD_GIMBAL=1`。不需要手工编辑宏；两个宏同时出现或都缺失会编译报错。`CHASSIS_FIRMWARE=lqr|mpc-probe` 用于底盘应用选择，云台不允许选择 MPC probe。兼容旧脚本的 CMake 构建目标名仍为 `chassis_firmware`，输出文件按板区分。

VS Code 的“运行生成任务”提供 `Balance robot: build chassis` 和 `Balance robot: build gimbal`；默认生成任务（Ctrl+Shift+B）编译底盘。

### 一个参数如何选择不同固件

`--preset gimbal` 读取 `CMakePresets.json`，预先设置 `ROBOT_BOARD=gimbal`、ARM 工具链和 `build/gimbal` 输出目录。也可以显式写出参数，例如：

```powershell
cmake --preset gimbal "-DROBOT_BOARD=gimbal"
cmake --build --preset gimbal
```

`-DROBOT_BOARD=...` 是 **CMake 配置变量**，在配置阶段使用；`cmake --build` 读取已生成的构建规则。每块板分别使用独立构建目录。

根 `CMakeLists.txt` 的固件分支执行 `include(cmake/Firmware.cmake)`。该文件先用 `set(ROBOT_BOARD ... CACHE STRING ...)` 声明可由命令行传入的缓存变量，再用 `if(ROBOT_BOARD STREQUAL "gimbal")` 选择：

| 配置 | 底盘 | 云台 |
| --- | --- | --- |
| 源码清单 | `cmake/ChassisSources.cmake` | `cmake/GimbalSources.cmake` |
| 板级头文件 | `platform/stm32f407/chassis/include` | `platform/stm32f407/gimbal/include` |
| 编译器宏 | `ROBOT_BOARD_CHASSIS=1` | `ROBOT_BOARD_GIMBAL=1` |
| 输出 | `chassis_firmware.elf` | `gimbal_firmware.elf` |

`add_executable()` 只加入选中板的源码，`target_include_directories()` 指定其头文件路径，`target_compile_definitions()` 才把板宏传给 C 编译器。因此同名的两份 `main.c`、中断回调和任务不会一起参与编译，公共文件由两块板分别编译进各自固件。

## 工程结构

```text
balance_robot/                  # 工程名；当前物理目录为 chassis_mpc
├── CMakeLists.txt               # host / firmware 总入口
├── CMakePresets.json            # chassis / gimbal；host / mpc-probe 为独立实验
├── balance_robot.code-workspace
├── cmake/
│   ├── Firmware.cmake           # 板选择、独立 include、宏与固件输出
│   ├── ChassisSources.cmake     # 底盘显式源码清单
│   ├── GimbalSources.cmake      # 云台显式源码清单
│   ├── RobotRuntime.cmake       # 共用 FreeRTOS 源码清单
│   ├── ChassisMpc.cmake         # 独立 MPC 库
│   └── toolchains/armclang.cmake
├── platform/stm32f407/
│   ├── chassis/
│   │   ├── src/ + include/      # 底盘 main、任务创建、外设配置
│   │   ├── cubemx/              # 底盘 CubeMX 参考
│   │   ├── mpc_probe_main.c
│   │   └── protocol_layout_checks.c
│   ├── gimbal/
│   │   ├── src/ + include/      # 云台 main、任务创建、外设与 USB 接入
│   │   └── cubemx/gimbal.ioc    # 云台 CubeMX 参考
│   ├── startup_stm32f407xx.s    # 共用启动汇编
│   ├── stm32f407ig.sct          # 共用 Flash/SRAM 布局
│   └── robot_link_layout_checks.c  # 两板均编译的协议 ABI 检查
├── src/
│   ├── chassis/
│   │   ├── application/        # 底盘任务与 chassis_config
│   │   ├── module/             # 跟随、停车、接地、越障、自救、功率
│   │   ├── components/         # LQR、腿部数学、平衡计算、独立 MPC
│   │   ├── bsp/                # 底盘特有硬件适配
│   │   ├── protocols/          # 底盘特有协议
│   │   └── service/            # 底盘 CAN 等服务
│   ├── gimbal/
│   │   ├── application/        # 云台任务、gimbal_config、自瞄、UART
│   │   ├── module/             # 遥控/键盘/yaw 指令与模式查询
│   │   ├── bsp/boards/         # 云台特有硬件适配
│   │   ├── components/         # 云台特有 IMU/EKF、算法、UI
│   │   └── IMPORT.json         # 导入来源、原始哈希与共享文件映射
│   └── common/
│       ├── include/            # 公共类型、声明与板选择宏校验
│       ├── application/        # RGB 灯任务
│       ├── components/         # 共用 PID、AHRS/DSP、FIFO、设备驱动
│       ├── protocols/          # 两板唯一的命令与 UART 帧定义
│       └── bsp/boards/         # 两板共用 BSP
├── third_party/                # CMSIS、FreeRTOS、HAL、USB、TinyMPC
├── tools/chassis/              # 模型、主机测试、仿真、MPC probe
├── tools/project/              # 构建隔离检查与日志轮换
├── docs/chassis-control.md     # 底盘启动、数据流、周期与调参入口
├── log/                        # 最近 3 次记录、索引和模板
└── build/                      # 构建产物与报告，Git 忽略
```

`src/chassis/application/gimbal_task.c` 是**底盘板的 yaw 执行任务**；`src/gimbal/application/gimbal_task.c` 是**云台板的任务**，二者不会一起编译。保留原硬件控制分工：云台板通过 UART 下发 yaw 参考和姿态，底盘侧执行 yaw 控制。

application 管任务顺序和接入，module 管操作与动作状态，controller 管控制计算，algorithm 管数学，BSP/platform 管硬件。模块公开头文件采用不透明状态声明；旧动作实现仍使用任务状态适配，尚未全部改为独立数值输入输出。

## 公共与板级边界

- 共用：PID（底盘版本兼容云台原计算）、基础类型、RGB 灯任务、相同 BSP/设备驱动、AHRS/DSP 库、CMSIS、FreeRTOS 10.3.1、启动汇编和链接布局。
- 分开：两板的 `main`、`FreeRTOSConfig.h`、外设初始化、不同版本的 IMU/BMI088 接口、姿态算法、`user_lib`、CAN 回调与控制任务。云台原 FreeRTOS 10.0.1/RVDS 移植到共享 10.3.1/GCC CM4F 端口，保留任务顺序、优先级、栈和 24 KiB RTOS 堆配置，仍需上板验证调度。
- HAL：底盘保持 `third_party/stm32_hal`；云台保留原版本 `third_party/stm32_hal_gimbal`，USB device 库放在 `third_party/stm32_usb_device`。两套 HAL 不会同时链接。

板间协议唯一来源是 [gim_cha_protocol.h](src/common/protocols/gim_cha_protocol.h) 与 [robot_link_frames.h](src/common/protocols/robot_link_frames.h)。命令负载 48 字节，包含帧头和原有对齐的命令帧 52 字节，反馈帧 8 字节；两板均检查尺寸、字段偏移和短枚举 ABI。保留旧字段别名供 UI/调试使用。当前仍是原固定长度串口协议，本次没有新增 CRC 或重设计数据同步。

## 从哪里读和改

| 内容 | 入口 |
| --- | --- |
| 底盘周期、传感器接入和电机输出 | `src/chassis/application/chassis_task.c`，见[底盘指南](docs/chassis-control.md) |
| 底盘 PID 默认值与在线调参声明 | `src/chassis/application/chassis_config.h`；`tune_*` 定义在任务顶部 |
| 平衡姿态参考、LQR 分量、质心前馈 | `src/chassis/components/controller/chassis_balance.c/.h` |
| LQR 增益拟合 | `chassis_lqr_gain_calculate(left_m, right_m, gain)`，无 HAL/任务依赖 |
| 云台任务、pitch PID、CAN 输出 | `src/gimbal/application/gimbal_task.c` |
| 云台控制参数与输入开关 | `src/gimbal/application/gimbal_config.h` |
| 遥控、键盘边沿、yaw 请求、底盘动作标志 | `src/gimbal/module/gimbal_command.c` |
| 自瞄/USB 与板间 UART | `src/gimbal/application/Self_aim.c`、`UART_task.c` |
| 任务创建、优先级和栈 | 对应 platform 下的 `src/freertos.c` |

底盘编译默认保持 pitch 参考 0 rad、对地腿角 0.1072 rad、前馈关闭；PID、LQR 系数和电机方向没有重新调参。自救返回是否接管，由任务显式重新做 VMC。旧 `chassis_power_limit()` 只缩放参考，保留历史调用顺序，不能视作最终电机限功率。跳跃准备占位判定、云台每拍两次射击更新和原 pitch 速度环 D 参数接线也保留，后续应分别做功能验证后修改。

## 验证与维护

主机验证需要原生 GCC/G++、Python、NumPy/SciPy，依赖见 `tools/chassis/mpc/requirements.txt`：

```powershell
python tools/chassis/mpc/run_host_checks.py
# 已配置后可直接执行
cmake --build --preset host
ctest --preset host --output-on-failure

# 两板构建后检查源码、include、宏和 ELF 是否隔离
python tools/project/check_board_builds.py
```

主机工具用于 MPC 独立 QP 对照；底盘独立单元测试文件及其构建入口已移除。MPC 当前为固定腿长 0.24 m、采样 4 ms 的独立实验；主机耗时不等于 F407 实时耗时。[工具说明](tools/chassis/README.md)包含更多模型约定。

日志只保留最近 **3 次**编号记录；每次新增后执行 `python tools/project/rotate_logs.py`，自动删除更旧记录并更新[日志索引](log/README.md)。`README.md` 和 `TEMPLATE.md` 永久保留。历史代码通过 Git 追溯，日志中的 RAM 读回不代表当前板上状态。
