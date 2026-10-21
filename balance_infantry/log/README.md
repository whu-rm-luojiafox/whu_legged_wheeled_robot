# 最近改动记录

只保留最近 **3 次**编号记录；`README.md` 与 `TEMPLATE.md` 永久保留。旧代码与已提交的历史记录通过 Git 追溯。

新增 `NNN-标题.md` 后，在工程根目录执行：

```powershell
python tools/project/rotate_logs.py
# 只预览删除范围
python tools/project/rotate_logs.py --dry-run
```

脚本只处理 `log/` 内的三位编号 Markdown，更新下面索引。下一次编号取现有最大编号加一；复制 [TEMPLATE.md](TEMPLATE.md) 填写。

<!-- RECENT_LOGS_START -->
- [009 底盘与云台平级目录](009-peer-board-layout.md)
- [008 balance_robot 双板工程融合与模块整理](008-dual-board-integration.md)
- [007 算法、控制器与功能边界；pitch 默认归零](007-component-boundaries-and-zero-pitch.md)
<!-- RECENT_LOGS_END -->

当前工程结构、构建目标和烧录文件见[根 README](../README.md)，底盘详细流程见[控制指南](../docs/chassis-control.md)。

记录应区分源码默认、RAM 临时调参、编译结果与硬件实测。本次工程融合未烧录；底盘源码默认 pitch 参考 0 rad、对地腿角 0.1072 rad、质心前馈关闭。重新链接后使用与板上固件匹配的 ELF，历史 RAM 地址不能直接复用。
