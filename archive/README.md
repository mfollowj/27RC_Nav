# 历史联调资料

`legacy_runs/` 保存原仓库 `logs/` 中已经提交的联调日志、实验脚本和旧服务样例。它们用于追溯当时的运行条件，保留原机器路径和测量内容。

当前运行从根目录 `real_*.sh`、`sim_*.sh`、`start_robot.sh` 进入，先读 [新人入口](../docs/README.md) 和 [脚本说明](../docs/scripts.md)。新的运行日志写入被 Git 忽略的 `logs/`，构建日志由 colcon 写入 `log/` 或 `third_party/log/`。

需要复用旧实验时，先根据当前源码、场地、地图及设备配置调整脚本，再在对应环境验证。验证后有长期用途的工具放入 `tools/` 或所属 ROS 包，并从当前文档链接。
