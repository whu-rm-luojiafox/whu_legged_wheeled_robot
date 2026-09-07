# 把远程 main 同步到自己的 fan-branch

本文适用于本仓库，命令在 PowerShell 的仓库根目录执行。示例个人分支为 `fan-branch`，远程名为 `origin`。

## 1. 先理解同步的方向

PR 必须已经**合并到 main**；仅审核通过但未合并，不会更新 main。

```text
GitHub 上的 main
       │ git fetch origin
       ▼
本地记录的 origin/main
       │ 在 fan-branch 上执行 git merge origin/main
       ▼
自己的本地 fan-branch（保留自己的提交，加入 main 的更新）
       │ git push -u origin fan-branch
       ▼
GitHub 上的 fan-branch
```

- `main`：本地的主分支。
- `origin/main`：最近一次 fetch 获取到的远程主分支状态。
- `fan-branch`：自己的开发分支。
- `fetch` 下载提交并更新远程跟踪分支，不修改当前工作区；`merge` 才会把更新合并到当前分支。

直接合并 `origin/main` 即可，不需要先切换到本地 `main`。更新本地 `main` 也不会自动更新 `fan-branch`。

## 2. 本次忽略规则：先完成一次提交

根目录的 `.gitignore` 忽略 Keil 编译生成的对象文件、依赖文件、固件输出、编译报告，以及个人窗口布局、插件日志和本机 include 路径配置。

此次还将已跟踪且命中规则的文件从 Git 索引中移除，**本地文件仍保留**。因此 `git status` 中会出现一批暂存的 `deleted`，这是取消版本跟踪，不是删除电脑上的文件。

检查并提交本次改动，然后再按后面的日常流程同步：

```powershell
git branch --show-current
git diff --cached --stat
git diff --cached -- .gitignore docs/Git同步main到自己的分支.md
git commit -m "chore: ignore generated files and document branch sync"
```

本次规则和教程已加入暂存区；如果之后又编辑了这两个文件，提交前重新执行：

```powershell
git add -- .gitignore docs/Git同步main到自己的分支.md
```

必要的工程依赖仍保留：`Middlewares/` 中的 FreeRTOS 和 USB 库、`Drivers/`、`RTE/`、算法 `.lib`、`.uvprojx`、`.uvoptx`、`.ioc`、链接脚本及共享调试配置。这里忽略的是“编译中间文件”，工程里的“中间件源码”仍需要上传。

`.gitignore` 只自动忽略未跟踪文件，对已跟踪文件无效。以后新增规则时，如需取消某个已跟踪文件的跟踪，应使用明确路径：

```powershell
# 将路径换成已确认的生成文件；只从索引移除，本地文件保留
git rm --cached -- '实际文件路径'
```

规则随分支提交保存，目前修改发生在 `fan-branch`。推送它只会更新远程个人分支；以后若将此提交通过 PR 合并到 `main`，主分支也会采用这些规则。

## 3. 日常同步：工作区干净时

先停止编译，避免 IDE 在同步过程中改写文件。逐条执行命令；某条报错时，先处理，不要继续往下执行。

```powershell
# 查看分支与修改；只有分支信息、没有文件列表，表示工作区干净
git status --short --branch

# 进入自己的分支
git switch fan-branch

# 获取远程最新提交
git fetch origin

# 把 main 的更新合并到自己的分支
git merge origin/main

# 检查结果
git status
git log -5 --oneline --graph --decorate
```

如果出现提交信息编辑器，保留默认合并说明，保存并退出即可。出现 `Already up to date.` 表示当前分支已包含所获取的 main 更新。

合并后用 Keil 编译，必要时进行功能验证。确认后，如果需要同步 GitHub 上的个人分支，再执行：

```powershell
git push -u origin fan-branch
```

这个 push 不会更新远程 main。不要使用 `git push origin main` 来推送个人分支。

## 4. 日常同步：自己还有未提交的代码

如果代码已完成，优先在自己的分支上明确选择文件并提交，再执行第 3 节。如果代码尚未完成，可以先 stash。

```powershell
# 先确认当前分支，下面假设你已经在 fan-branch 上开发
git status --short --branch

# 暂存本地修改，-u 同时包含未跟踪文件
git stash push -u -m "fan-branch 同步 main 前保存开发修改"

# 立即查看刚创建的备份；记下它的完整提交哈希
git stash list
git rev-parse 'stash@{0}'

git fetch origin
git merge origin/main
```

确认 stash 输出为成功保存。如果提示 `No local changes to save`，说明没有新建 stash，此时 `stash@{0}` 可能是旧备份，不能把它当成本次修改恢复。

合并成功后，恢复刚才记录的那份备份。将下面的占位文字替换成实际哈希：

```powershell
git stash apply '刚才记录的完整哈希'
git status
```

`apply` 保留备份，恢复内容确认无误后，再用 `git stash list` 找到对应条目并按需删除。不要直接删除或恢复不确定的旧 stash。PowerShell 中应给 `stash@{0}` 这样的引用加单引号。

`-u` 不包含已经被忽略的编译文件。通常不需要 `-a`，它会连被忽略的产物一起保存。

如果开始时在别的分支，先在那里保存修改，再切到 `fan-branch` 同步；属于别的分支的开发修改，应回到那个分支后再恢复。

## 5. 常见报错怎么处理

### “Your local changes ... would be overwritten”

Git 发现未提交修改会被覆盖，合并或切分支尚未完成。这不是已经产生的合并冲突。

执行 `git status` 查看文件，先提交或 `git stash push -u` 保存，再重试。如果配置刚保存就又变了，停止编译并关闭相关 IDE 窗口或自动生成配置的插件。

### “untracked working tree files would be overwritten”

未跟踪文件也会被目标分支的同名文件覆盖。对于未被忽略的文件，使用带 `-u` 的 stash 保存。

忽略规则不能充当备份：切换到仍跟踪同名文件的旧分支时，本机忽略文件仍可能受到影响。重要的本机配置可额外复制到仓库外保存。

### 合并后出现 “CONFLICT”

```powershell
git status
git diff --name-only --diff-filter=U
```

打开冲突文件，根据实际代码保留双方需要的逻辑，删除 `<<<<<<<`、`=======`、`>>>>>>>` 标记。逐个确认后：

```powershell
# 替换成刚解决的实际文件路径
git add -- '已解决的文件路径'
git merge --continue
```

如果决定取消正在进行的合并：

```powershell
git merge --abort
```

如果冲突发生在 `git stash apply`，按同样方式编辑文件、再 `git add` 标记解决；这时没有正在进行的 merge，**不要执行 `git merge --continue` 或 `git merge --abort`**。stash 备份仍在，可用来核对原内容。

### 加了 ignore，为什么合并时还会遇到日志或配置冲突？

因为个人分支已取消跟踪，但远程 main 可能仍跟踪并修改这些文件，合并时可能出现 modify/delete 冲突。`.gitignore` 不会替你解决这种冲突。

先用 `git status` 确认文件。只有确定是应忽略的产物或本机配置时，先在仓库外备份有用内容，再用下面的命令保留取消跟踪的决定：

```powershell
git rm --cached -- '已确认应忽略的冲突文件路径'
```

若 Git 拒绝，先检查该文件的索引和工作区状态，不要直接加 `-f`。全部冲突解决后再 `git merge --continue`。对源码冲突，应正常审查代码。

如果上游重新加入了应忽略的文件，可检查：

```powershell
# 列出“仍被跟踪，但已匹配忽略规则”的文件
git ls-files -ci --exclude-standard
```

团队在 main 中统一忽略规则、取消产物跟踪，才能减少这类反复冲突。

### push 被拒绝，提示 non-fast-forward

远程 `fan-branch` 也有本地尚未包含的提交。工作区干净时，先同步自己的远程分支：

```powershell
git fetch origin
git merge origin/fan-branch
# 解决冲突并验证后
git push -u origin fan-branch
```

不要用强制推送跳过别人提交。

## 6. 验证忽略规则

```powershell
# 查看具体命中了哪条规则（文件还没生成也能检查规则）
git check-ignore -v -- 01Balance_chassis_task/MDK-ARM/.vscode/c_cpp_properties.json
git check-ignore -v -- 01Balance_chassis_task/MDK-ARM/chassis_task/adc.o

# 正常状态中不应再出现已取消跟踪的编译产物
git status --short
```

本教程采用合并流程，保留个人分支提交。`git reset --hard origin/main` 会重置个人分支并覆盖已跟踪文件，不应作为日常同步命令。
