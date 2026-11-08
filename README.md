# XLeRobot 遥操作与数据采集

本仓库保存 XLeRobot、LeRobot 源码快照，以及本地遥操作、校准和采集脚本。克隆后包含完整源码，无需初始化子模块。

## 安装

建议在 Linux 上使用 Python 3.12。安装 uv 后，在仓库根目录执行：

```bash
uv venv --python 3.12
source activate.sh
uv pip install -r requirements-installed.txt
python scripts/check_install.py
```

`requirements-installed.txt` 记录了原机器的实际依赖版本，并使用相对路径安装本地插件。PyTorch/CUDA 相关包需要根据目标机器和可用的软件源调整。

## 使用

参见 [使用说明](使用说明.md)。原使用说明中的 `/home/antep/projects/Xlerobot` 请替换为当前克隆路径。

本仓库不包含虚拟环境、采集数据、日志、本机串口映射和机器人校准参数。连接新机器人时先核对串口角色并重新校准。

## 上游与许可证

- XLeRobot：https://github.com/Vector-Wangel/XLeRobot ，源版本与本地改动见 `UPSTREAM.md`。
- LeRobot：https://github.com/huggingface/lerobot ，源版本见 `UPSTREAM.md`。

上游源码保留各自目录内的 LICENSE，相关素材及依赖遵循原有许可。
