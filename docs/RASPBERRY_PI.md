# 树莓派迁移说明

本仓库保存 2026-11-11 上传时的工作区代码。当前已有 Mac 端手势 retarget 和实机控制实现；此次只整理并上传代码，没有在树莓派上部署或验证。

## 已保存的内容

- `bare_hand_teleop/`：追踪、优化 retarget、实测耦合路径、滤波、安全门控、HTTP 驱动。
- `scripts/run_sim.py`：当前优化 retarget 主入口；默认 `measured` 路径，实机默认发送 7 路电机计数。
- `config/sim_calib_right.json`：当前右手标定；相机或操作者变化后需重新标定。
- `mac_driver/`：包含 `motor_target`、保持/释放与 5–30 Hz 支持的当前服务；`pi_snapshot/` 保留此前 Pi 版本供比较。
- `vendor/yubi-pi_gmh15-sdk/models/`：左右手 URDF、STL 与 MuJoCo 模型；`models/hand_landmarker.task`：当前 MediaPipe 模型。
- `tests/`、`patches/`、`tools/`、厂家 Python 示例及 SDK PDF 文档。

没有上传 `.venv`、缓存、运行录制与日志、旧备份、SDK ZIP/DEB/RAR 安装包、重复 CAD 导出。这些文件保留在原 Mac。厂家 ARM64 SDK 与 Docker 镜像也不在仓库，迁移时需要单独保留原 Pi 上的镜像，或从 Mac 导出 `gmh15-teleop:3.9.0` 后导入目标 Pi。仓库里的 `Dockerfile.teleop` 依赖已有的厂家基础镜像，不能单独从零重建 SDK。

## 获取代码并检查环境

先在目标机器登录 GitHub（私有仓库需要访问权限），然后：

```sh
git clone https://github.com/limit-coding/gmh15-retarget.git
cd gmh15-retarget
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt -r mac_driver/requirements.txt
.venv/bin/python -m pytest -q tests
.venv/bin/python scripts/replay.py --synthetic --out runs/synthetic.csv
```

Mac 当前环境为 Python 3.12.11，numpy 2.5.4、OpenCV 5.0.0.93、MediaPipe 1.2.2、scipy 1.18.1、MuJoCo 3.17.0、matplotlib 3.11.2、pytest 9.2.0、pyserial 3.5。这是来源环境记录，不是 Pi 的已验证依赖锁定；安装前确认目标系统是 64 位 ARM64，并检查所用 Python 版本对应的 MediaPipe、MuJoCo 安装包是否可用。原 SDK 文档记录的运行环境为 Ubuntu 22.04 ARM64 / Python 3.10 容器，与主机视觉环境分开。

## 路线一：视觉留在 Mac，Pi 连接灵巧手

可先沿用 Mac 的视觉环境，将 USB 控制服务迁回 Pi。Pi 服务保持只监听本机，通过 SSH 转发后让 Mac 访问：

```sh
ssh -N -L 8765:127.0.0.1:8765 <你的Pi登录名和地址>
```

Mac 上从仓库根目录运行手势程序：

```sh
.venv/bin/python scripts/run_sim.py --camera 1 --side right \
  --hardware --i-have-checked-safety --url http://127.0.0.1:8765
```

相机编号需重新核对；不要同时运行占用 Mac 本机 8765 的服务。程序启动后仍需手动初始化和使能。

**协议必须匹配**：当前客户端默认使用 `motor_target`，保持/释放也依赖新服务。旧 Pi 快照不能直接当作完整兼容服务；迁移当前功能应使用本仓库 `mac_driver/` 服务代码，并在 Pi 上验证。`--joint-mode` 是旧的角度映射路径，只适合核对基础通信，不等同于当前完整手势控制。

## Pi 上验证当前服务的起点

以下仅作为待现场验证的启动模板；先确认 `gmh15-teleop:3.9.0` 镜像已存在、右手实际串口正确，并停止其他占用该串口的服务。在仓库根目录执行：

```sh
docker run --rm --init --name gmh15-pi \
  --cap-drop ALL --security-opt no-new-privileges \
  --device /dev/ttyACM1:/dev/ttyACM1 \
  -p 127.0.0.1:8765:8765 \
  -v "$PWD/mac_driver:/opt/teleop:ro" \
  -v "$PWD/vendor/yubi-pi_gmh15-sdk/models:/opt/models:ro" \
  --entrypoint python3 gmh15-teleop:3.9.0 \
  -u /opt/teleop/teleop_server.py --bind 0.0.0.0 --hands right \
  --right-port /dev/ttyACM1 --motor-only --no-render --no-camera
```

此模板为只读启动，不经过 Mac 的 USB 转发层。先在 `http://127.0.0.1:8765/`（或 SSH 转发后）确认身份、位置、故障等反馈。需要运动时，退出只读实例，再显式加 `--allow-motion`；服务不自动初始化或使能。先从默认 10 Hz 开始，验证通过后才考虑 `-e GMH15_CONTROL_HZ=30`。

## 路线二：视觉与控制都运行在 Pi

先完成上面的服务核对，再安装主机端视觉依赖并运行 `scripts/run_sim.py`。当前主入口会创建 OpenCV 窗口和 MuJoCo 渲染线程，需要可用的图形/渲染环境；尚无经过验证的 Pi 无桌面启动方式。Mac 的 `.command`、`scripts/start_all.sh` 和 `scripts/run_mac_hand.py` 使用 iTerm2、OrbStack 或 Mac USB 转发，应在 Pi 上改用 Python 入口。30 fps / 30 Hz 的整链路性能需要在 Pi 上重新测量。

## 验证范围

2026-11-11 在原 Mac 环境验证：主代码 64 项测试通过；历史接触补丁单独运行，50 项测试和 28 项子测试通过。此次没有更改 retarget 或硬件控制逻辑。

主代码测试从根目录执行 `python -m pytest -q tests`。历史接触补丁的测试需单独在其目录执行：`cd patches/teleop_grasp_contact && ../../.venv/bin/python -m pytest -q test_teleop_control.py`。不要将两套测试放进同一个 pytest 进程：它们导入同名的不同版本 `teleop_control`，会发生模块冲突。

迁移后核对相机左右手、标定、7 路电机顺序与限位，再进行空载运动与停止测试。Mac 的已有结果不能代替 Pi 的现场验证。
