# 009 底盘与云台平级目录

日期：2026-10-21。

## 修改

按用户要求，将两个板作为平级业务域：

```text
src/
├── chassis/  # application、module、components、bsp、protocols、service
├── gimbal/   # application、module、components、bsp
└── common/   # 公共类型、PID、BSP、设备、库、协议与 RGB 任务

platform/stm32f407/
├── chassis/  # src、include、cubemx、底盘布局检查、MPC probe
└── gimbal/   # src、include、cubemx
```

启动汇编、链接布局与两板公共协议检查保留在 `platform/stm32f407/` 根目录。底盘原本位于 `src/` 根部的任务/驱动不再与整份云台目录并列；两块板都从公共目录显式取依赖，不包含另一块板目录。

`FirmwareSources.cmake` 改为 `ChassisSources.cmake`，与 `GimbalSources.cmake` 对应。迁移前后逐文件校验字节哈希，再更新 CMake 清单、头文件搜索路径、兼容头文件相对路径、MPC 生成/仿真脚本、主机检查及文档链接；控制参数和计算公式未修改。发现裁判系统的 `protocol.h` 两板原文件相同，一并放到 common，避免云台从底盘目录取公共头文件。

README 新增参数工作原理：`--preset` 读取预定义配置，`-DROBOT_BOARD=...` 设置 CMake 缓存变量，根 CMakeLists 引入 Firmware.cmake，再由条件分支选择源码、include、宏与输出。该参数在配置阶段使用；编译阶段执行已经生成的规则。

## 验证

- 底盘 `cmake --build --preset chassis`、云台 `cmake --build --preset gimbal` 完整编译链接通过；输出路径继续分别为 `build/chassis/chassis_firmware.elf`、`build/gimbal/gimbal_firmware.elf`。
- `cmake --build --preset host` 与 `ctest --preset host --output-on-failure`：**5/5 通过**。
- `cmake --build build/mpc-integration --parallel 2`：独立 ARM MPC probe 链接通过。
- `python tools/project/check_board_builds.py`：104/111 个翻译单元检查通过；加强为禁止引用另一板整个 `src` 与 `platform` 目录。
- 检查 `src` 只含 chassis/common/gimbal，platform 下两板目录平级；README、底盘指南和最新日志索引的链接有效，`git diff --check` 通过。
- 日志轮换后保留 007、008、009，模板和索引保留。

本轮未连接或烧录硬件，未覆盖 `build/lqr` 的旧 ELF（修改时间仍为 2026-10-15 22:25）。历史记录中的源码位置与 ELF 哈希描述对应轮次，不作为当前路径/产物。
