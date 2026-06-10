# SpaceMouse Marvin Teleop

这个 demo 用 SpaceMouse 遥操作 Marvin 机械臂，默认控制左臂 `A`。控制链路是：

SpaceMouse 增量输入 -> 累积 TCP 目标位姿 -> `Marvin_Kine.ik()` -> `Concise_Marvin_Robot.set_joint_position_cmd()` 位置模式下发 7 轴关节目标。

## 文件

- `config.py`：机器人 IP、控制周期、轴映射、速度/加速度比例、工作空间限制、`libspnav` 路径。
- `spacemouse_input.py`：基于 `libspnav` 的 SpaceMouse 非阻塞读取。
- `spacemouse_teleop.py`：Marvin 位置模式遥操作主循环。
- `joint_drag_debug.py`：从 SDK `case7` 拆出来的独立关节拖拽调试脚本，支持 `Ctrl+C`、按钮松开延时退出、总超时自动关闭。
- `ccs_m6_40.MvKDCfg`：本 demo 默认运动学配置文件。

## 依赖

Python 侧需要：

```bash
pip install numpy
```

SpaceMouse 侧需要 `spacenavd` 和 `libspnav`。Ubuntu 上常见安装方式：

```bash
sudo apt install spacenavd libspnav-dev
sudo systemctl enable --now spacenavd
```

如果系统库名找不到，可以把 `libspnav.so.0.4` 放到当前目录，或运行前指定：

```bash
export LIBSPNAV_PATH=/path/to/libspnav.so.0.4
```

## 运行

先在仓库根目录做静态检查：

```bash
python3 -m py_compile spacemouse_demo/*.py
```

测试 SpaceMouse 输入：

```bash
python3 spacemouse_demo/spacemouse_input.py
```

连接机器人后运行遥操作：

```bash
python3 spacemouse_demo/spacemouse_teleop.py --ip 192.168.1.190 --arm A --print-mouse
```

只想临时进入关节拖拽调试时，可以运行：

```bash
python3 spacemouse_demo/joint_drag_debug.py --ip 192.168.1.190 --arm A
```

默认行为：

- 按住末端按钮开始拖拽。
- 松开按钮不会退出，可以反复按住继续拖拽。
- 总运行 `300` 秒后自动关闭。
- 退出时会额外等待 `1` 秒，再退出拖拽并下使能。
- `Ctrl+C` 可立即退出。

也可以按需调整：

```bash
python3 spacemouse_demo/joint_drag_debug.py --timeout 120 --exit-delay 1
```

也可以通过环境变量覆盖默认值：

```bash
MARVIN_ROBOT_IP=192.168.1.190 MARVIN_ARM=A python3 spacemouse_demo/spacemouse_teleop.py
```

## 安全默认值

- 默认 `A` 臂，关节位置模式，`VEL_RATIO=10`，`ACC_RATIO=10`。
- 启动后先订阅当前反馈关节，用 FK 对齐当前 TCP，再开始累积 SpaceMouse 增量。
- 必须按住 SpaceMouse 左键（默认 button `0`）才会累积位姿增量并下发运动指令。
- IK 无解、目标超工作空间、关节超限、奇异或订阅失败时，本周期跳过下发。
- 工作空间超限时不会把目标裁剪到边界，而是停止本周期运动指令。
- 默认只打开平移轴，旋转轴在 `config.py` 的 `AXIS_ENABLE` 里关闭；确认方向后再逐步打开。
- 第一次有机械臂测试时，先用很低速度，只推动单轴，确认轴向和比例正确后再扩大动作。
- `Ctrl+C` 会停止 SpaceMouse、下使能当前手臂并释放机器人连接。
