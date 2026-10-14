# 轮腿底盘控制工程

统一入口是根目录 CMake。现有底盘业务代码、TinyMPC 求解模块和验证程序分别构建，编译产物统一放在 `build/`。

## 编译器与构建系统

CMake 管理源文件、依赖、编译参数与链接命令，仍然需要编译器。当前 TinyMPC/Eigen 无法用已有 ARMCC5 编译；改为 CMake 不会消除这个限制。

本工程采用 **CMake + Ninja + ARM Compiler 6（armclang / armasm / armlink）** 构建板端，电脑验证使用原生 GCC。当前工程不再包含 Keil IDE 工程文件，仍需要安装并授权 ARM Compiler 6 工具链。

暂不提供完整固件的 GNU Arm 构建配置：现有 `AHRS.lib`、DSP 库、启动汇编、散装加载文件和 RTOS 端口都与 ARM 工具链有关，不能仅切换一个编译器路径。已安装 GNU Arm 工具链不等于这项迁移已经完成。

## 构建目标

| 配置 | 用途 | 输出 |
| --- | --- | --- |
| `host` | double/float TinyMPC 与独立 QP 对照、失败保护、线性模型闭环回放 | `build/host/mpc_probe_*.exe`、`reports/comparison.json` |
| `lqr` | 原底盘应用的完整 ARMClang 固件，使用现有 LQR 控制路径 | `build/lqr/chassis_firmware.{elf,hex,bin,map}` |
| `mpc-probe` | 独立 MPC 板端测量固件，不启动底盘任务或电机外设 | `build/mpc-probe/chassis_firmware.{elf,hex,bin,map}` |

### 电脑验证

需要 Python、NumPy/SciPy、CMake、Ninja、原生 g++。从工程根目录执行：

```powershell
python tools/chassis/mpc/run_host_checks.py
# 编译器未自动发现时：
python tools/chassis/mpc/run_host_checks.py --cxx 'D:/your-mingw/bin/g++.exe'
```

脚本默认验证已提交的生成数据。修改模型或求解配置后，显式执行 `--regenerate`；模型参数变动后必须重新做数值验证。依赖列表见 `tools/chassis/mpc/requirements.txt`。

配置完成后也可以使用：

```powershell
cmake --build --preset host
ctest --preset host
```

### 板端固件

安装并授权 ARM Compiler 6；本机验证使用 6.7。PowerShell 示例：

```powershell
$env:ARMCLANG_ROOT = 'D:/Keil5/ARM/ARMCLANG'
cmake --preset lqr
cmake --build --preset lqr

cmake --preset mpc-probe
cmake --build --preset mpc-probe
```

两个固件拥有各自的启动代码与完整链接结果；分别使用独立构建目录。`mpc-probe` 的 `main` 在 SystemInit 后的默认 HSI 时钟下执行一次求解，结果保存在 `whu_mpc_probe_record`，随后等待中断。这个测试耗时不能当成 168 MHz、RTOS 并发下的耗时。`sequence == 1` 才表示测量已经完成。

探针保留 8 KiB 主栈、32 KiB C/C++ 堆；这是预留预算，实际峰值还需板端测量。现有启动文件不传配置时保持原有 8 KiB 堆。FreeRTOS 的 `heap_4` 与 C++ malloc/new 使用的堆分别管理。

## 文件职责

| 目录 / 文件 | 职责 |
| --- | --- |
| `src/application/` | RTOS 任务与应用编排，目前仍包含原底盘控制流程 |
| `src/bsp/` | 板载设备与通信驱动封装 |
| `src/components/` | 算法、控制器、设备、UI 组件 |
| `src/protocols/`、`src/service/` | 通信协议与共享服务 |
| `src/module/chassis/` | 接地、功率、恢复、越障、打滑模块 |
| `src/module/chassis/chassis_mpc/` | MPC 的 C 接口、结果检查、配置与生成缓存 |
| `third_party/` | STM32 HAL、CMSIS、FreeRTOS、TinyMPC 及其许可证 |
| `tools/chassis/` | 模型导出、缓存生成、电脑仿真与板端测试入口 |
| `platform/stm32f407/` | 启动汇编、链接布局、板级源文件/头文件、探针入口与协议布局检查 |
| `platform/stm32f407/cubemx/` | 保留的 CubeMX 引脚与时钟配置参考 |
| `cmake/` | MPC 库目标、显式固件源文件清单、工具链配置 |
| `build/` | 当前构建、日志、数值报告，不属于源代码 |

目录结构：

```text
chassis_mpc/
├── src/                 业务代码
├── platform/stm32f407/  MCU 启动、板级代码和配置
├── third_party/         第三方源码
├── tools/chassis/       模型导出和验证工具
├── cmake/               构建规则与工具链
├── build/               自动生成的构建产物
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

旧 Keil 工程、迁移备份、临时实验、过期调试配置和旧构建产物已移出当前工程，归档位于同级 `chassis_mpc_legacy_files_20261015/`，原始历史材料另有 `chassis_mpc_legacy_20261015.zip`。仍在使用的启动文件已迁入 `platform/stm32f407/`。

旧 MATLAB 算法回放脚本依赖已删除的 `TORQ_K` 和 `InverseJacobianCoefficient` 等定义，与当前固件不兼容，放在上述历史归档的 `historical_tools/lqr_replay/`，不属于当前验证入口。现用 MPC 模型导出工具和独立 QP 验证保留在 `tools/chassis/`。

CubeMX `.ioc` 保留原硬件配置，内部工具链字段属于历史生成设置；当前 CMake 构建不读取该字段。需要重新生成 HAL 时，先输出到独立目录，再将变更合入 `platform/`，避免生成旧工程结构。

新增板端源文件时更新 `cmake/FirmwareSources.cmake`。不使用递归 glob，以免将备份、电脑程序或多个 main 编进固件。MPC 库只公开 C 接口；Eigen 和生成缓存头只对库及诊断程序可见。

## 当前控制边界

目前 MPC 模型为固定双腿 0.24 m、10 状态、4 输入、4 ms 预测周期、8 个状态节点。模型输入中的虚拟腿力矩并不直接等于关节电机力矩。实车仍使用原 LQR 流程；独立 MPC 固件用于测量资源与求解结果。

后续实车接入的职责顺序应为：传感器快照 → 状态与零点适配 → 参考生成 → LQR/MPC 控制计算 → 支撑力/VMC 与功率限制 → 电机命令。求解器不读取 CAN、不发送电机命令、不创建 RTOS 任务。

`src/application/chassis_task.c` 的模式切换、反馈估计与控制流程仍有耦合，本次没有把这些算法视为已经重写。下一步拆分需要先确定状态符号、标定零点、腿长有效范围和离地行为，再用相同输入回放确认等价；不能把实验模型输出直接接到电机上。

## 验证范围

2026-10-15 本机验证结果：原生 GCC 8.1.0 通过 24/24 独立 QP 对照、5 组失败保护和 500 步线性闭环回放；ARMClang 6.7 的 `lqr`、`mpc-probe` 均完成完整链接并输出 ELF/HEX/BIN。9 项协议尺寸/偏移检查分别通过 ARMCC5 和 ARMClang 编译。

| 固件 | 链接报告 ROM 占用 | 链接报告 RW+ZI（含预留堆栈） |
| --- | --- | --- |
| `lqr` | 102156 bytes | 55472 bytes |
| `mpc-probe` | 254856 bytes | 46448 bytes |

电脑数值验证与完整固件链接属于软件检查。真实任务耗时、堆栈峰值、传感器适配、外设时序与实车闭环仍需要板端验证。旧代码中的非移植 include 大小写、未覆盖的 switch 等警告应逐项处理，不能通过链接成功推断已消除。

TinyMPC 上游版本、运行时补丁与代价函数约定见 [第三方说明](third_party/tinympc/README.WHU.md)。
