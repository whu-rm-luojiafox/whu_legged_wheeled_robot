# 000 烧录与调试基础修正

调试日期：2026-10-15。记录整理：2026-10-16。

## 现象

最初 Ozone 结构体无法展开，之后能展开但值为零、底盘没有反应。EmberProbe 后续日志出现 USB 写入失败、无法读取 DP IDR，以及一次暂停在异常捕获函数的记录。烧录校验通过只证明 Flash 写入内容，不证明任务已经正常运行。

## 工程改动

- `cmake/Firmware.cmake`：Release 固件保留 `-g` 调试信息；链接设置 `--datacompressor=off`，避免压缩 RW 加载区与运行 RAM 范围不一致影响变量写入校验。
- `platform/stm32f407/src/stm32f4xx_it.c`：增加 `fault_record`，保存异常状态寄存器及有效栈帧 PC/LR；校验栈地址和浮点扩展帧，异常后停留供读取。
- 调参标量、部分观察状态与 `uart_data` 放入独立 section，降低 ARMClang 全局合并对 ELF/DWARF 类型绑定的影响。此次整理仍保留原 section 名称。

## 后续结果与限制

恢复探针连接并重新运行后，读到了变化的姿态数据，用户确认正常使能。后续试调使用 `build/lqr/chassis_firmware.elf`，而非独立测量固件 `mpc-probe`。

初始异常的具体根因没有获得足够证据，不将增加异常记录称作修复 HardFault。`INS_angle` 曾被插件显示为未知类型，与 ELF 中存在完整数组类型是不同层面的问题。SVD 用于外设寄存器解释，不能补齐普通 C 变量的 DWARF 类型。

当前读回用的旧固件 SHA-256：`73590c9d5efd9dc1fb14d615ee7a2e833f9111dc92ae77feefef32fe8e116642`。这是本轮整理前的固件指纹；本轮在独立目录编译，没有覆盖其 ELF 或再次烧录。
