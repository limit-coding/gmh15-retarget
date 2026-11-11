# 裸手视觉 → GMH15 retargeting（单手原型）

**GitHub 快照（2026-11-11）**：当前代码包含实测耦合的优化 retarget、7 路电机计数映射、保持/释放及 5–30 Hz 控制支持。树莓派迁移所需的入口、模型、SDK 外部依赖和验证步骤见 [树莓派迁移说明](docs/RASPBERRY_PI.md)；当前实机记录见 [Mac 驱动说明](mac_driver/README.md)。以下正文包含早期阶段记录，历史状态与参数不代表当前代码的全部行为。

**2026-11-07：Mac 本机直连，默认纯电机界面。** 双击 `启动灵巧手.command` 后访问 <http://127.0.0.1:8765/>，只读取电机与关节反馈，不启动摄像头、触觉采集或模型渲染；启动不自动初始化或使能。位置读取已验证，最新两次实机初始化均返回失败，因此电机运动测试尚未通过。详细记录和入口见 [Mac 驱动说明](mac_driver/README.md)。以下历史记录中的 Pi 部署、相机、触觉、SSH 隧道描述保留供参考。

目标链路（GELLO → Piper 不改动）：

```
操作者裸手 → RGB 相机 → MediaPipe 21 关键点 → 特征/标定 → 门控+滤波 → retarget → 安全守卫 → 驱动(dry-run / GMH15 HTTP)
```

当前状态：视觉追踪、映射、保护逻辑和离线回放已经实现，并用单元测试和合成数据验证过。
**没有接过相机实测，也没有驱动过真机。** 真机驱动默认关闭（见第 6 节）。

---

## 1. 工作区里有什么（`~/灵巧手`）

| 文件 | 内容 | 能确认什么 |
|---|---|---|
| `hand_client.py` | GMH15 teleop 服务器（`127.0.0.1:8765`，在 Pi 上）的 HTTP 客户端 | 协议：`GET /api/teleop/config`（token、`joint_names`、`limits_degrees`）、`GET /api/state`，以及 `POST /api/teleop`，动作有 `initialize / arm / heartbeat / target / preview / stop`；需要 `X-Teleop-Token` 和 Origin 头 |
| `auto_test_right.py` + 两份 `auto_test_full*.log` | 右手逐关节测试和半握拳测试，结果 PASS | **支持逐关节连续角度目标**（11 维命令，其中 7 个电机）；能读回 15 维关节角、7 个电机原始计数、故障列表和五指触觉力；心跳约 10 Hz，停掉心跳控制就会掉线；`stop` 后电机失能 |
| `patch_pip_passive.py` | 改服务器：四指 PIP 无电机，必须为 0 | 四指各只有 **1 个电机（MCP）**，PIP 没有独立电机，也没有反馈；拇指有 CMC/MCP/IP 共 3 个电机 |
| `probe_thumb.py` | 拇指 CMC 0/60/100° 拍照探测 | 曾经命令到 100°，但结果图不在工作区，**CMC 的运动方向（对掌还是外展）仍未确认** |
| `初始化后仿真姿态.jpg` | 服务器渲染的 "LIVE / 15 SDK joints" 模型 | SDK 侧有可视化模型（网格），文件不在本机 |
| `右手相机照片*.jpg`, `初始化后相机.jpg`, `probe/cam_now.jpg` | 服务器 `/media/camera/right.jpg` 的画面 | 镜头朝向天花板，像是装在机器人一侧。**不是用来拍操作者手的相机** |

日志里的关键数据：命令 30° 时实测 30.0–31.2°，45° 时实测 45.5–46.2°，全程无故障。

工作区之外（只读查看，没有修改）：
- `~/superminic/artifacts/startup64-comparison-20261106/gmh15/`：一个 GMH15 MuJoCo 仿真任务，模型 `robot.xml` 在远程集群路径上，本机没有。该模型有 15 个关节（`left_fingerN_joint1..3`），通过 tendon 传动驱动。**它和实机（四指 PIP 无电机、SDK 正运动学把 PIP 报为 0）是否一致，尚未核实。**
- `~/piper-drawing-references/`（piper_sdk、agx_arm_urdf）、`~/3D打印/gello_mechanical`：只有 Piper SDK 参考和 GELLO 机械件，**没有现在跑着的 GELLO→Piper 遥操作代码**。

本机环境：Apple M2 Mac，Python 3.12，numpy/opencv/scipy 已装，**mediapipe 未装**，dex-retargeting/pinocchio 也没装；摄像头有 FaceTime 内置相机和 iPhone 连续互通相机。

## 2. 厂家资料核对结果（2026-11-06 补充）

资料来源：
- 你下载到 `左手相关代码/` 的 SDK 文档（PDF）和 `3D drawing URDF.rar`（左右手两份 `.rar` 完全相同）。解压在 `vendor/vendor_urdf_rar/`。
- 从 `yubi-pi:~/gmh15-sdk` 只读拷贝的模型、teleop 源码和说明，放在 `vendor/yubi-pi_gmh15-sdk/`。厂家 URDF 和 Pi 上那份逐关节一致。

| 问题 | 结论 | 依据 |
|---|---|---|
| SDK/固件 | DexHandSDK 3.9.0（C++ 库 + `pxdex` Python 3.10 包），Pi 上跑在 Docker 镜像 `gmh15-sdk:3.9.0` 里。左手 SN H15L49G00062，右手 H15R49G00039 | Pi 上的说明文件 |
| 连续控制 | 支持：`setJointPositionsAngle`（11 个归一化角）、`setMotorTargetPosition`（7 路，0–10000）、速度模式、电流模式、最大速度 0–1000 | H15 API Reference (Python) |
| 驱动器与耦合 | **7 个电机，11 个“可调关节”**。每根手指的近端关节和远端关节**共用一个电机**（腱驱动），所以每根手指只有 1 个可控自由度；拇指 3 个关节分别对应电机 5/6/7 | 离线 SDK 逆解 `tools/ik_coupling_probe.py`：食指 (j1,j2)=(40,0)→2789，(0,60)→1999，(80,80)→9876 |
| “PIP 无电机” | 这是本项目 teleop 服务器的约定（`patch_pip_passive.py`），**不是厂家说法**。PIP 由同一根腱带动，厂家用 8 个磁编码器做反馈。实测反馈 PIP 一直是 0：是机构在空载时只弯 MCP，还是正解有问题，**待查** | SDK 文档 3.1/4 节 |
| URDF | 15 个转动关节，没有 `mimic`，即**没有写耦合关系**。四指：joint_0 0–80°、joint_1 0–81.6°、joint_2 0–76°；拇指：0–109.6°（旋转轴和其他关节不同）、0–87°、0–120°；指尖有触觉片 link | `DexH15_*_urdf.urdf` |
| 反馈 | 电机位置、磁编码器、电机速度/电流、故障码（含位置/速度超限、磁编断联）、触觉（327 点/手，约 10 Hz）、掌内相机 | API 文档 + Pi 说明 |
| teleop 服务器参数 | `SLEW_DEG_S=8`，`MAX_RATE_HZ=10`，arm 时把电机最大速度设成 100/1000；`WATCHDOG_S=0.6`，超时直接 `disableMotor`；**目标和反馈相差 >10° 就失能** | `teleop/teleop_control.py` |

### 由此得出的三个关键问题
1. **跟手性**：8°/s 和 100/1000 的速度上限都是我们自己的服务器设的，可以放宽，但要在真机上逐级测试。
2. **抓取会掉物**：手指被物体挡住、目标继续收紧时，跟踪误差超过 10° 就会失能，手随即松开。抓东西必须改这条规则，比如：接触后（触觉力或电流超过阈值）冻结目标、改成力控，或者只在空载时检查误差。这要改 Pi 上的服务器。补丁草案在 `patches/teleop_grasp_contact/`（用指尖触觉力判断接触，限制目标的超前量，本地 50 项测试通过）。**还没部署，部署要你确认。**
3. **捏合在当前范围内做不到**：按 URDF 正运动学（`tools/pinch_search.py`，用触觉片网格的质心作指尖），在已验证范围（指 ≤45°、拇指 ≤30°）内，拇指尖和食指尖最近也有 103 mm。
   - 让 PIP=0 时，最近 4.6 mm，此时食指 MCP 80°（到了极限）、拇指 CMC 83.5°、MCP 53.8°、IP 31.9°；
   - 让 PIP 也弯时，两指能对上，此时食指 MCP 66°/PIP 36°、拇指 CMC 83.5°/MCP 61°/IP 20°。
   - 这只是模型结果，候选值放在 `config/gmh15_right.pinch_candidate.json`，必须在真机上慢速逐步验证。

### 仍然缺的
- 正确的 `hardware_version` 字符串（离线逆解给空字符串时报 “Hardware version is invalid”，绝对数值不准）；
- 腱驱动下 MCP/PIP 的实际分配曲线（需要在真机上扫电机位置、读磁编码器）；
- 拇指 CMC 的实际动作方向（模型里是绕 −y 轴转，需要拍照确认）；
- `disableMotor` 后手指是保持还是松掉；
- 现有 GELLO/Piper 代码；拍操作者手的相机。
- `SDK for Ubuntu22.04/24.04 x86_64.zip` 和 `PaXini Hand Studio.deb` 还在下载（`.crdownload`），我还没看。

## 3. 可行性判断

- **连续控制可行**，有 7 个可控自由度：四指各 1（整根手指的闭合量），拇指 3。
- 所以把“人的整根手指弯曲（MCP+PIP+DIP）映射到这根手指的电机”这种做法，和硬件结构是一致的。
- 下一步可以把指尖/捏合优化放到 URDF 上做（`bare_hand_teleop/gmh15_model.py` 已有正运动学和指尖点），前提是先测出每根手指的 MCP/PIP 分配曲线。
- 降级方案（只能发预设手势时）：不需要。

## 4. 路线和代码结构

参考 AnyTeleop：单 RGB + MediaPipe，先走向量/弯曲映射，再加指尖优化。参考 DexPilot：拇指与食指距离小于阈值时，强制机器人指尖靠拢。

```
bare_hand_teleop/
  hand_types.py   HandObservation（21 点，world 单位米 + image）、HandFeatures
  tracking.py     MediaPipeTracker（Tasks HandLandmarker, VIDEO 模式）+ 左右手选择
  features.py     关节角特征：四指 MCP（在屈曲平面内投影，手指外展不会算成弯曲）/PIP/DIP、拇指 MCP/IP、
                  拇指对掌（拇指尖到小指根的距离/手尺度）、捏合距离（米和比例）。对镜像不变，左右手共用
  retarget.py     Calibration：每个操作者的 张开/握拳/捏合 参考值 → 0..1 闭合量
                  LinearCurlRetargeter：闭合量 → 7 个电机角（按范围线性映射）+ 可选捏合融合
                  （在实机上测出“拇指尖碰食指尖”的关节角，捏合时平滑融合过去）
  filters.py      One-Euro 滤波（静止时平滑，快动时低延迟）
  safety.py       TrackingGate：没有手/左右手不对/置信度低/帧过旧/尺度异常/整手跳变
                  TargetGuard：限位、逐关节速度限制、丢失时保持（不发新目标）、
                  恢复前要求连续 N 帧有效，恢复后限速从 25% 渐升到 100%
  pipeline.py     上面各步串起来，在线和离线回放共用
  drivers.py      DryRunDriver；GMH15HttpDriver（见第 6 节）
  overlay.py      画关键点、追踪状态、各指闭合条、捏合距离、7 路命令和实测值
  gmh15_spec.py   命令/反馈索引，“已验证范围”（指 0–45°，拇指 0–30°）和 URDF 极限
  gmh15_model.py  厂家 URDF 的 numpy 正运动学 + 指尖（触觉片）位置
tools/ik_coupling_probe.py  在 Pi 的 SDK 容器里离线跑逆解（不挂设备）
tools/pinch_search.py       在 URDF 上搜索捏合姿态
scripts/run_camera.py  实时原型（默认 dry-run）
scripts/replay.py      离线：回放录下的 JSONL，或跑合成轨迹 → CSV/图
tests/                 22 个单元测试（含用假服务器测 HTTP 驱动的安全行为）
```

替换点：追踪后端（任何输出 21 点的都行）、`Retargeter`（例如 dex-retargeting）、驱动，三者相互独立。

## 5. 使用

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
mkdir -p models && curl -L -o models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

实时（dry-run，不连硬件）：
```bash
python scripts/run_camera.py --side right --record runs/live.jsonl
```
标定（每个操作者、每种相机位置做一次）：张开平手按 `o`，握拳按 `f`，拇指尖碰食指尖按 `p`，然后按 `s` 保存到 `config/calib_right.json`。
相机不是面对操作者的自拍视角时，加 `--no-flip`，并举一下手，确认画面上的 `hand=Right/Left` 标签正确。

离线：
```bash
python scripts/replay.py --synthetic --out runs/synthetic.csv --plot runs/synthetic.png
python scripts/replay.py --jsonl runs/live.jsonl --calib config/calib_right.json --out runs/live.csv --plot runs/live.png
python -m pytest -q tests
```

## 6. 真机保护（已实现部分）

`GMH15HttpDriver` 默认**只读**：`connect()` 只发 GET，读取限位和当前位置；不带开关时永远不 POST。
要驱动真机，下面几条必须同时满足：`--driver gmh15-http --hardware --i-have-checked-safety`，并且在窗口里**手动按 `a`** 才会 arm。
arm 之前会检查：服务器 `last_init_result == 1`、没有故障、该侧没有被别的 owner 占用。

运行中的保护：
- 目标值先按“已验证范围”和服务器限位取交集再截断；PIP 强制为 0；
- 客户端逐关节速度限制；arm 后从实测位置开始限速逼近；
- 追踪丢失时不发新目标，但心跳照常发，手保持在最后一个目标（这样不会松手掉物）；
- 主循环卡住超过 0.3 s → 主动 `stop`（不会让心跳线程用旧目标把手“吊”住）；
- 通信失败超过 0.5 s、服务器显示 inactive 或 owner 变了、设备报故障 → 停止发送并 `stop`；
- 退出时，只要本程序 arm 过就发 `stop`；不会去停别人的会话。

以上逻辑都用假服务器测过（`tests/test_http_driver.py`），**没有在真服务器上跑过**。

### 第一次上真机前的检查清单
1. 把 Pi 上 `~/gmh15-sdk/teleop/` 拷过来核对：slew 速度、心跳超时、`stop` 和掉心跳后电机的行为、`limits_degrees`；
2. 确认拇指 CMC 方向，再决定 `thumb_opp` 映射的正负和范围；
3. 手里不拿东西，用 `--max-rate 30` 先跑；全程有人在 teleop 网页上随时能按停止；
4. 逐项测：张开/握拳、单指、捏合、手移出画面（应当保持）、移回（应当平滑接回）、拔网线/关 Pi 服务（应当 stop）。

## 7. 已做的验证
- `pytest`：22 项通过（新增 URDF 结构和正运动学检查），覆盖特征角度复原、镜像不变、外展不误判为弯曲、映射端点和限位、捏合融合、门控各种原因、速度限制、丢失保持和恢复、长间隔不跳变、驱动的只读/arm 前置条件/截断/卡死/掉线/通信超时。
- 合成轨迹回放（`runs/synthetic.png`）：张开→握拳→张开→捏合→握拳（中途丢 0.6 s）→张开。命令最大变化率不超过 90°/s；丢失期间保持不动；恢复后平滑接回。
- 未做：真实相机上的 MediaPipe 精度/延迟、真机响应、跟手延迟。这些需要你在现场跑。

## 8. 后续
1. **相机实测**：在 Mac 上跑 `run_camera.py`，录几段 JSONL（张开握拳、单指、捏合、快速动作），离线调 One-Euro 参数和标定。
2. **使用姿势验证（必须做）**：操作者握着 GELLO 的时候，录正面、斜上方、侧面三个机位，统计 TRACKING 占比和 `low_score/no_hand` 原因；重点看 GELLO 握把挡住手掌、手腕翻转、双手互相遮挡。很可能需要每只手一台相机（俯视斜前方），或者改 GELLO 握把，让手指露出来。
3. **捏合实标**：用 teleop 网页把拇指尖和食指尖凑到一起，读关节角，写进 `config/gmh15_right.json` 的 `pinch_pose_deg`，然后测试捏取薄片和小物体。触觉力已经能读到，可以作为“接触”的判据记录下来。
4. **指尖优化**：拿到和实机一致的 URDF（7 主动 + 耦合关系）后，用 dex-retargeting 的 `DexPilot`/`vector` 优化器实现一个 `Retargeter` 子类直接替换，并比较 Mingrui Yu 文中的手型/指尖/捏合目标。**dex-retargeting 目前不支持 GMH15，需要自己写配置。**
5. 双手：起两个 Pipeline/驱动（`side=left/right`），左右手标签要按相机朝向核对；再接入数据采集（与 GELLO 的时间戳对齐）。

## 9. 仿真阶段：基于 URDF 的优化 retarget（2026-11-06）

参考了三篇工作：
- AnyTeleop / dex-retargeting：在 URDF 上做向量 retarget；
- DexPilot：捏合时把指尖对拉近；
- Mingrui Yu 等：在指尖目标之外，再加一个弱的手型先验。

没有直接调用 dex-retargeting：它导入时需要 torch，而且只支持线性的 `mimic` 耦合；GMH15 是腱驱动，耦合很可能是分段的。

- **变量（7 个）**：四根手指的闭合量 `s∈[0,1]`，加拇指的 3 个关节角。
- **手指路径 `FINGER_PATHS`**：把闭合量 `s` 换算成 (MCP, PIP, DIP)。
  - 默认 `mcp_first`：先弯 MCP 到 80°，再弯 PIP/DIP。这个假设和真手日志一致（命令 MCP 45° 时，编码器正解得到的 PIP 是 0°），但只验证到 45°；
  - 对照方案 `synergy`：三个关节按比例一起弯；
  - 测出真实曲线后替换。
- **代价函数**：在人手和机器手的手掌坐标系里比较（x：小指根→食指根，y：手指方向，z = x×y），人手向量乘以中指长度比 α。
  - 四指：指根→指尖向量；拇指：食指根→拇指尖向量；
  - 拇指→食指/中指的指尖向量：人手距离小于 2 cm 时，目标变成 6 mm 的短向量（DexPilot），2–5 cm 之间平滑过渡；
  - 拇指 MCP/IP 的弱手型先验；时间平滑项。
- **求解**：每根手指先在整条路径上做网格全局搜索，拇指也从一张粗网格里选起点，再用 L-BFGS-B 加解析梯度细化。暖启动平均约 5 ms。全局起点是必要的：只用局部暖启动时，手指握拳后会卡在弯曲状态（见 `runs/sim_replay.png` 的修复前后对比）。
- **仿真**：MuJoCo 纯运动学，模型是 Pi 上转换好的 `hand.xml`。自写的正运动学和 MuJoCo 的指尖位置一致到 0.1 mm。渲染放在单独线程里，约 140 ms/帧。
- **镜像自检**：检查手指是否朝“右手掌心”一侧弯曲，界面上显示 chirality OK 或 MIRRORED?。

合成轨迹的结果（`tools/sim_replay.py`）：
- 张开、握拳、各指开合都能跟上；
- 人手捏合（<20 mm）时，机器人两个指垫质心的距离中位数 9.3 mm，最大 11.1 mm，其余手指保持张开；
- 丢失时保持，恢复后平滑接回。

**真人手结果（Mac 内置相机，`runs/sim_live2.jsonl`，49 s，701 帧跟踪）**：
- 张开/握拳、单指（29–40 s 依次弯四指）、捏合都能跟上；人手捏合 <20 mm 时机器人指垫距离中位数 10.1 mm，最大 13.2 mm；求解约 2 ms/帧；
- 发现并修复：左右手标签方向写反了（画面翻转后 MediaPipe 把右手标成 Left，699/701 帧），见 `tracking.py`；
- **张开标定**：放松张开的手原始闭合度约 0.11–0.19（食指 MCP ≈ 30°）。在 `run_sim.py` 里张开手按 `o`、握拳按 `f`，结果保存到 `config/sim_calib_right.json`，下次自动加载（`--no-calib` 忽略）。闭合度重映射 `(s-open)/(fist-open)`；对食指/中指，人手越接近捏合，重映射越弱，捏合精度不受影响。现有的 `config/sim_calib_right.json` 是从这次录制里提取的；
- **拇指**：握拳时拇指被遮挡，求解代价有多个极小值。加了迟滞：全局解比沿用上一帧拇指的解便宜 40% 以上才切换，拇指总变化量降低约 20%。剩下的大跳变出现在快速握拳/张开时（每帧 24°，就是 360°/s 限速），是真实运动；不同张开段之间 CMC 的差别（1°–44°）也对应人手拇指离掌面距离的真实差别（17–32 mm）；
- 帧率只有 15 fps（光线偏暗时内置相机会降帧）。
- 评估脚本：`tools/eval_recording.py <jsonl> [--calib ...]`。

运行（Mac，第一次会请求相机权限）：
```bash
.venv/bin/python scripts/run_sim.py                          # 内置相机，自拍视角
.venv/bin/python scripts/run_sim.py --camera 1 --no-flip     # 外接相机，不是自拍视角时加 --no-flip
.venv/bin/python scripts/run_sim.py --record runs/sim_live.jsonl
.venv/bin/python tools/sim_replay.py --jsonl runs/sim_live.jsonl --mirror-world   # 录制时画面镜像过，就加 --mirror-world
.venv/bin/python tools/sim_snapshots.py                      # 生成 张开/握拳/捏合 的渲染对比图
```
相关代码：`bare_hand_teleop/opt_retarget.py`、`sim_pipeline.py`、`sim.py`；`scripts/run_sim.py`；测试在 `tests/test_opt_retarget.py`（全部 32 项通过）。

**平滑（`filters.MotionShaper`）**：安全层之后再加一级二阶跟随：速度 ≤120°/s、加速度 ≤600°/s²，并提前制动。在 `sim_live2` 上，每帧最大变化从 24.6° 降到 8.2°，加速度 p99 = 600°/s²，平均滞后 2.5°。

**真机模式**：`run_sim.py --hardware --i-have-checked-safety`。需要先开 SSH 隧道 `ssh -N -L 8765:127.0.0.1:8765 yubi-pi`，默认 URL 是 `http://localhost:8765`（服务器要求 Origin 与访问地址一致）。启动时不使能；按 `a` 使能，`d` 失能，`q` 退出（同时失能）。发出去的角度限制在已验证范围内：四指 MCP 0–45°，PIP 0°，拇指 0–30°。仿真画面显示的就是实际发出的值，真手还会受服务器端 8°/s 缓动的限制。使能前服务器要求位置初始化成功（`last_init_result == 1`）。

这一阶段的输出 `cmd11` 是 SDK 的 11 关节顺序，**包含四指的 PIP**，只用来驱动仿真。现在的 teleop 服务器仍然强制 PIP=0，两边还没接上。

## 7. 工作空间仿真（2026-11-06）

`tools/workspace_analysis.py` 用厂家 URDF 做正运动学，并在 MuJoCo 里检查碰撞（原模型关掉了碰撞，脚本里打开），比较 4 种情况：A 把 15 个关节都当成独立的；B/C 是 7 电机，每根手指 1 个电机经腱带动，分两种耦合猜测；D 是当前服务器的限位。输出放在 `runs/workspace/`：`viewer.html`（交互 3D，可用 `python3 -m http.server --directory runs/workspace` 打开）、`overview.png`、`summary.json`。

结论（只基于模型）：四指 3 个关节转轴互相平行，**没有外展关节**，所以指尖只能在一个平面里动，左右可动量是 0。电机耦合后，每个指尖只能沿一条曲线走。拇指和食指、中指能对捏，能不能对正要看腱耦合曲线。拇指碰不到小指（最近 27 mm），无名指只能勉强碰到。手指张开/并拢和手内旋转，任何情况下都做不到。在当前服务器限位下，所有捏取动作都做不到。
