# Kuavo-5 右手高举操作手册

本文记录在 WSL 控制 Kuavo-5 执行一次“右手高举”动作的完整命令。命令分为只读检查、一次性动作文件适配、真实运动、监控和中断五部分。

## 1. 已验证的设备信息

- WSL 控制端：`192.168.26.11`
- 机器人下位机：`192.168.26.1`
- 机器人上位机/相机：`192.168.26.12`
- ROS master：`http://192.168.26.1:11311`
- 下位机账户：`lab`
- 下位机密码：三个空格，然后按 Enter
- 下位机仓库版本：`1.4.4`
- ROS 参数 `/robot_version`：`55`
- 动作服务：`/execute_arm_action`
- 软件中断服务：`/interrupt_arm_traj`

这些地址可能随网络配置变化。每次执行前都要重新检查。

## 2. 在 WSL 检查网络

```bash
ip -4 -brief address
ip route get 192.168.26.1
ping -c 2 -W 1 192.168.26.1
ping -c 2 -W 1 192.168.26.12
```

预期结果：

- WSL 有一个 `192.168.26.x/24` 地址；
- 到 `192.168.26.1` 的路由使用该网卡；
- 两个地址均无丢包。

检查 SSH 和 ROS master 端口：

```bash
nc -zvw2 192.168.26.1 22
nc -zvw2 192.168.26.1 11311
nc -zvw2 192.168.26.12 22
```

## 3. 在 WSL 检查 ROS 控制链路

```bash
cd /home/source/LeTools-Learning
source /opt/ros/noetic/setup.bash

export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.11

./tools/check_kuavo_arm_control.sh \
  --master http://192.168.26.1:11311 \
  --ip 192.168.26.11
```

必须看到：

```text
RESULT ROS GRAPH READY
```

脚本只读取 ROS 图，不发布控制指令。它会确认：

- `/sensors_data_raw` 有状态发布者；
- `/kuavo_arm_traj` 有控制器订阅者；
- `/kuavo_arm_target_poses` 有控制器订阅者。

## 4. 登录机器人下位机

```bash
ssh lab@192.168.26.1
```

出现密码提示后，输入三个空格，再按 Enter。

登录后执行：

```bash
cd /home/lab/kuavo-ros-opensource
source devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
```

检查软件和机器人版本：

```bash
git describe --tags --always
rosparam get /robot_version
```

本次成功执行时分别返回：

```text
1.4.4
55
```

如果 `/robot_version` 不是 `55`，不要继续使用本文生成的适配动作。

## 5. 执行前只读状态检查

检查机器人步态：

```bash
rosservice call /humanoid_get_current_gait_name "{}"
```

必须返回：

```text
success: True
gait_name: "stance"
```

检查机械臂是否空闲：

```bash
timeout 3 rostopic echo -n 1 /humanoid/mpc/is_arm_executing
```

必须返回：

```text
data: False
```

检查机械臂控制模式：

```bash
rosservice call /humanoid_get_arm_ctrl_mode 0
```

本次动作前返回 `mode: 1`，表示自动摆臂模式。动作服务会按内部流程切换控制模式。

检查状态和控制话题：

```bash
rostopic info /sensors_data_raw
rostopic info /kuavo_arm_traj
rostopic info /kuavo_arm_target_poses
```

检查是否启动碰撞检测节点：

```bash
rosnode list | grep collision
```

本次执行时没有返回碰撞检测节点。因此执行前必须人工确认右臂侧面和上方无人员、桌子、墙面或其他障碍物，并安排人员守在急停附近。

## 6. 一次性适配旧动作文件

原始动作文件：

```text
/home/lab/.config/lejuconfig/action_files/右手高举.tact
```

该文件是旧格式：

- 缺少 `robotType`；
- 每帧有 28 个通道；
- 当前 Kuavo-5 控制器要求 29 个通道。

下列命令保留原文件，生成适配副本：

```bash
python3 - <<'PY'
import copy
import json
from pathlib import Path

source = Path("/home/lab/.config/lejuconfig/action_files/右手高举.tact")
target = Path("/home/lab/.config/lejuconfig/action_files/right_hand_raise_checked.tact")

data = json.loads(source.read_text(encoding="utf-8"))

# 控制器源码的兼容表允许 robotType 52 动作运行在 robot_version 55。
data["robotType"] = 52

for index, frame in enumerate(data["frames"]):
    servos = frame["servos"]
    attributes = frame["attribute"]

    if len(servos) == 28:
        # 新增通道在整个动作中保持中性值 0。
        servos.append(0)

    if "29" not in attributes:
        # 目标始终为零，复用相邻零通道的插值属性。
        attributes["29"] = copy.deepcopy(attributes["28"])

    if len(servos) != 29:
        raise ValueError(f"frame {index}: expected 29 servos, got {len(servos)}")

target.write_text(
    json.dumps(data, ensure_ascii=False, separators=(",", ":")),
    encoding="utf-8",
)
print(target)
PY
```

检查适配结果：

```bash
python3 - <<'PY'
import json

path = "/home/lab/.config/lejuconfig/action_files/right_hand_raise_checked.tact"
data = json.load(open(path, encoding="utf-8"))

print("robotType:", data["robotType"])
print("frame count:", len(data["frames"]))
print("servo counts:", [len(frame["servos"]) for frame in data["frames"]])
print("keyframes:", [frame["keyframe"] for frame in data["frames"]])
print("finish:", data["finish"])
PY
```

预期输出：

```text
robotType: 52
frame count: 4
servo counts: [29, 29, 29, 29]
keyframes: [0, 100, 200, 300]
finish: 300
```

该适配只需执行一次。文件已经存在且上述检查通过时，可直接进入下一步。

## 7. 实际动作内容

动作使用双臂 14 维角度，单位为度：

```text
初始姿态：
左臂  [20, 0, 0, -30, 0, 0, 0]
右臂  [20, 0, 0, -30, 0, 0, 0]

右手高举姿态：
左臂  [20, 0, 0, -30, 0, 0, 0]
右臂  [-55, -85, -20, -45, -90, 0, 10]
```

动作过程约 3 秒：移动到高举姿态、短暂保持、返回初始姿态。

## 8. 最后人工确认

在发送真实运动命令前，确认：

1. 机器人处于稳定站立状态；
2. 右臂侧面和上方无障碍物；
3. 没有人接触机器人；
4. 有人能够立即操作硬件急停；
5. `gait_name` 仍为 `stance`；
6. `is_arm_executing` 仍为 `False`。

再次执行即时检查：

```bash
rosservice call /humanoid_get_current_gait_name "{}"
timeout 3 rostopic echo -n 1 /humanoid/mpc/is_arm_executing
```

## 9. 发送真实运动命令

下面的命令会让机器人真实运动：

```bash
rosservice call /execute_arm_action \
  "action_name: 'right_hand_raise_checked'"
```

成功时返回：

```text
success: True
message: "Action executed successfully"
```

服务返回成功表示控制器接受并执行了动作。仍应通过现场观察和关节反馈确认实际动作。

## 10. 监控动作

可在第二个 SSH 终端中执行：

```bash
cd /home/lab/kuavo-ros-opensource
source devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311

rostopic echo /humanoid/mpc/is_arm_executing
```

动作期间应出现 `data: True`，完成后恢复为 `data: False`。

查看实时关节反馈：

```bash
rostopic echo /sensors_data_raw/joint_data/joint_q
```

## 11. 软件中断与硬件急停

需要停止轨迹时，在第二个 SSH 终端执行：

```bash
rosservice call /interrupt_arm_traj "{}"
```

如果出现碰撞风险、身体失稳、异常抖动或软件中断没有及时生效，应由现场人员使用硬件急停。

## 12. 动作后检查

```bash
rosservice call /humanoid_get_current_gait_name "{}"
timeout 3 rostopic echo -n 1 /humanoid/mpc/is_arm_executing
timeout 5 rostopic echo -n 1 /sensors_data_raw/joint_data/joint_q
rosservice call /humanoid_get_arm_ctrl_mode 0
```

本次成功执行后的结果为：

```text
gait_name: "stance"
is_arm_executing: False
arm control mode: 1
```

完成后退出 SSH：

```bash
exit
```

## 13. 常见失败信息

### 无法连接 ROS master

```text
ERROR: Unable to communicate with master!
```

检查 WSL 地址、路由、网线、`ROS_MASTER_URI` 和机器人控制程序。

### 缺少 robotType

```text
文件合法性检查失败: 缺少必需字段: robotType
```

说明使用了旧动作文件。执行第 6 节的一次性适配。

### 需要 29 通道

```text
servos数组长度应为29，实际为28
attribute缺少键: ['29']
```

说明旧动作尚未补齐 Kuavo-5 当前格式要求的中性通道。

### 正在执行其他动作

```text
另一个动作正在执行中，请等待当前动作完成后再试
```

等待 `/humanoid/mpc/is_arm_executing` 变为 `False`。如需终止当前轨迹，使用 `/interrupt_arm_traj`。
