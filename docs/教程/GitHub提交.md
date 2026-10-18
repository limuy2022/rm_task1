# GitHub Desktop：把代码和教程一起交给 B

本机真正的 Git 仓库是 `/Users/a1/Documents/RBM/github/rm_task1-repo`，远程是私有仓库 `limuy2022/rm_task1`。文件已经放进这里；不要再把整个 outputs、work 或 ZIP 拖进仓库。

## A 在 Mac 上提交

1. 打开 GitHub Desktop，左上选择 rm_task1。菜单 Repository → Show in Finder，确认打开的是上面的 rm_task1-repo。
2. 点 Changes。代码和教程都要提交：src、42-gimbal_serial、tools、docs、README、.gitignore 中本次实际出现的变更。没有改动的目录不会显示，正常。
3. 这次会出现很多红色删除：旧 docs/vision 中整份源码副本、旧 docs/当前任务 和阶段一旧教程已清理；新的独立教程在 docs/教程。**这些删除也要勾上**，否则 GitHub 仍留着重复旧文件。G 历史记录保留了你的本地修改。
4. 不选 .venv、build、install、log、ZIP、实验 bag/CSV/地图或现场原始照片。它们不是本次源码提交；正常会被 .gitignore 排除。
5. Summary 填 `接通跟踪与点云建图，整理调参和上机教程`，点 Commit to 当前分支。Commit 只是存到本机，还没有上传。
6. 点 Push origin。如果显示 Pull origin，先 Pull；有冲突就保留对话框和文件名再处理，**不要随便点整份覆盖一侧**。上传成功后网页刷新，确认最新提交和 docs/教程 已出现。

本次已整理文件，尚未代你点击 Commit/Push。如果界面显示 staged/unstaged 两类，二者都核对；整理本身没有改变你的分支。

## B 怎样拿到

B 先用自己的 GitHub 账号接受这个私有仓库的协作邀请。A 的 2223037417-beep 账号能看到，不等于 B 的账号也已获权。

B 在自己的 GitHub Desktop：已有本地 rm_task1 就 Fetch origin → Pull origin；没有就 File → Clone repository，填 `https://github.com/limuy2022/rm_task1.git`。下载后打开 docs/教程，当前先看云台跟踪.md 和上机调参检查清单.md；轮到建图/评分再打开对应独立教程。

去实验室时，A 另把最新版 `47-visual-tracking-20261018-r3.zip` 发给 B。B 用 Windows 上的 NoMachine 把它上传到 NUC，照云台跟踪教程解压/编译。这个 ZIP 由源码生成，不必提交进 Git；保留本机输出文件夹里的最新一个就够了。

## 如需自己重新打包

在个人电脑仓库根目录（不是公用 NUC）输入：

```bash
python3 tools/make_visual_tracking_bundle.py --output /tmp/47-visual-tracking-20261018-r3.zip
```

Windows 的 Python 命令可能是 py，输出位置换自己能找到的路径。工具只打包源码和教程，不启动 ROS 或硬件。生成后再传 NUC；不要把 Mac 上的 build/install 当 Linux 可执行文件发送。
