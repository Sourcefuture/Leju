# Kuavo-5 摄像头与双臂 14 维控制手册

本文只整理两项已经在当前 Kuavo-5 真机上核对过的能力：

1. 从 WSL 查看、订阅和保存头部及腕部相机数据；
2. 使用 14 维关节数组规划和执行双臂动作。

本文可离线阅读。所有 ROS、相机和真机控制命令都必须等机器人网络恢复后运行。

仿真、笛卡尔相对移动、RGB-D物体坐标和14维IK转换见[Kuavo-5 仿真与现实坐标转14维手册](kuavo5_simulation_and_cartesian_conversion_manual.md)。

## 1. 已知设备和网络结构

上次真机联调时使用的地址如下：

| 设备 | 地址 | 作用 |
|---|---|---|
| Kuavo-5 下位机、ROS Master、SSH | `192.168.26.1` | 机械臂控制和机器人状态 |
| 相机主机 | `192.168.26.12` | 头部、左腕、右腕相机驱动 |
| WSL | `192.168.26.11` | 查看图像和发起远程控制；地址可能随网络配置改变 |

网络恢复后先在 WSL 检查地址：

```bash
ip -4 -brief address
ip route get 192.168.26.1
ping -c 2 -W 1 192.168.26.1
ping -c 2 -W 1 192.168.26.12
```

在需要直接使用 ROS 图像工具的每个 WSL 终端中执行：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.11
```

如果 WSL 地址已经改变，应把 `ROS_IP` 替换为当前与机器人同网段的实际地址。

验证 ROS Master：

```bash
rostopic list
rostopic info /sensors_data_raw
```

## 2. 调用三个相机

### 2.1 相机 Topic 总表

三个相机分别为头部 `cam_h`、左腕 `cam_l` 和右腕 `cam_r`。

| 相机 | RGB 压缩图 | 彩色内参 | 深度图 | 深度内参 |
|---|---|---|---|---|
| 头部 | `/cam_h/color/image_raw/compressed` | `/cam_h/color/camera_info` | `/cam_h/depth/image_raw` | `/cam_h/depth/camera_info` |
| 左腕 | `/cam_l/color/image_raw/compressed` | `/cam_l/color/camera_info` | `/cam_l/depth/image_rect_raw` | `/cam_l/depth/camera_info` |
| 右腕 | `/cam_r/color/image_raw/compressed` | `/cam_r/color/camera_info` | `/cam_r/aligned_depth_to_color/image_raw` | `/cam_r/aligned_depth_to_color/camera_info` |

上次真机实测结果：

- 头部和左腕 RGB 均为 `848×480`，约 31 Hz；
- RGB 类型为 `sensor_msgs/CompressedImage`，内容为逐帧 JPEG；
- 头部与左腕消息格式为 `rgb8; jpeg compressed bgr8`；
- 右腕节点已注册，但当时 10 秒内没有收到新帧；
- 原始深度类型为 `sensor_msgs/Image`；已保存样本是 `16UC1`、`uint16`、毫米单位，数值 0 表示无效深度。

Topic 出现在 `rostopic list` 中只说明它已向 Master 注册，不保证仍在持续发送帧。应使用 `rostopic hz` 检查实时性。

### 2.2 直接打开 GUI

单独查看头部 RGB：

```bash
rqt_image_view /cam_h/color/image_raw/compressed
```

查看左腕或右腕 RGB：

```bash
rqt_image_view /cam_l/color/image_raw/compressed
rqt_image_view /cam_r/color/image_raw/compressed
```

查看深度图：

```bash
rqt_image_view /cam_h/depth/image_raw
rqt_image_view /cam_l/depth/image_rect_raw
rqt_image_view /cam_r/aligned_depth_to_color/image_raw
```

三个命令可分别放在三个终端中运行。WSL 需要启用 WSLg 或配置可用的 X Server；可以先检查：

```bash
echo "$DISPLAY"
```

### 2.3 使用项目三画面查看器

项目工具：[view_kuavo_cameras.py](../tools/view_kuavo_cameras.py)

同时查看三个 RGB 画面：

```bash
cd /home/source/LeTools-Learning
/usr/bin/python3 tools/view_kuavo_cameras.py --camera all
```

只查看一路：

```bash
/usr/bin/python3 tools/view_kuavo_cameras.py --camera head
/usr/bin/python3 tools/view_kuavo_cameras.py --camera left
/usr/bin/python3 tools/view_kuavo_cameras.py --camera right
```

窗口中的绿色 `age` 表示最近收到了新帧，红色表示画面已过期，`WAITING FOR FRAMES` 表示尚未收到图像。按 `q` 或 Esc 退出。该程序只订阅图像，不发布机器人控制消息。

### 2.4 检查格式、帧率、带宽和内参

以头部相机为例：

```bash
rostopic type /cam_h/color/image_raw/compressed
rostopic echo --noarr -n 1 /cam_h/color/image_raw/compressed
rostopic echo -n 1 /cam_h/color/camera_info
rostopic hz /cam_h/color/image_raw/compressed
rostopic bw /cam_h/color/image_raw/compressed

rostopic type /cam_h/depth/image_raw
rostopic echo --noarr -n 1 /cam_h/depth/image_raw
rostopic hz /cam_h/depth/image_raw
```

检查右腕相机是否真的有帧：

```bash
timeout 10 rostopic hz /cam_r/color/image_raw/compressed
timeout 10 rostopic hz /cam_r/aligned_depth_to_color/image_raw
```

若只看到 publisher 名称但结果为 `no new messages`，说明驱动节点存在，但当前没有图像传到 WSL。

### 2.5 在 Python 中读取 RGB

最小订阅代码：

```python
#!/usr/bin/python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


def on_image(msg):
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        return
    print(
        "frame=", msg.header.frame_id,
        "stamp=", msg.header.stamp.to_sec(),
        "shape=", bgr.shape,
    )
    cv2.imshow("head", bgr)
    cv2.waitKey(1)


rospy.init_node("kuavo_head_camera_reader", anonymous=True)
subscriber = rospy.Subscriber(
    "/cam_h/color/image_raw/compressed",
    CompressedImage,
    on_image,
    queue_size=1,
    buff_size=2**22,
    tcp_nodelay=True,
)
rospy.spin()
```

`cv2.imdecode(..., cv2.IMREAD_COLOR)` 返回的数组顺序为 BGR。若送入通常按 RGB 训练的模型，应转换：

```python
rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
```

实时程序建议使用 `queue_size=1`，优先处理最新图像，避免算法落后于现场画面。

### 2.6 保存一组 RGB-D 数据

采集工具：[capture_kuavo_rgbd.py](../tools/capture_kuavo_rgbd.py)

保存头部 RGB、原始深度和内参到宿主机 `E:\Data\Picture`：

```bash
cd /home/source/LeTools-Learning
/usr/bin/python3 tools/capture_kuavo_rgbd.py \
  --camera cam_h \
  --output /mnt/e/Data/Picture/head_test
```

左腕和右腕：

```bash
/usr/bin/python3 tools/capture_kuavo_rgbd.py \
  --camera cam_l \
  --output /mnt/e/Data/Picture/left_test

/usr/bin/python3 tools/capture_kuavo_rgbd.py \
  --camera cam_r \
  --output /mnt/e/Data/Picture/right_test \
  --timeout 10
```

每次成功采集会生成：

```text
<输出目录>/
├── rgb.jpg             # OpenCV BGR JPEG
├── depth_mm.npy        # uint16 原始深度，毫米
├── depth_preview.png   # 仅供人眼查看的伪彩色图
└── camera.json         # topic、时间戳、内参、FOV、格式和深度统计
```

`depth_preview.png` 做过按帧拉伸和着色，不能用于测距。几何计算应读取 `depth_mm.npy`。

采集脚本依次接收 RGB、深度和 CameraInfo，并非硬件同步。应检查 `camera.json` 中的 `stamp_delta_seconds`；快速运动时不能忽略两帧时间差。

### 2.7 已实测的视场角

FOV 由 `CameraInfo.K/P` 和图像宽高计算，不是独立控制参数：

```text
头部彩色：水平 92.54°，垂直 61.22°
左腕彩色：水平 88.30°，垂直 57.59°
左腕深度：水平 89.26°，垂直 58.38°
```

这些数值描述上次连接时的有效针孔模型。更换分辨率、裁剪方式或相机标定后，应重新读取 `CameraInfo`，不要写死旧参数。

## 3. 使用 14 维数组控制双臂

### 3.1 数组顺序与单位

控制 Topic：

```text
/kuavo_arm_target_poses
```

消息类型：

```text
kuavo_msgs/armTargetPoses
```

14 个值的固定顺序为：

```text
values = [
  left_j1, left_j2, left_j3, left_j4, left_j5, left_j6, left_j7,
  right_j1, right_j2, right_j3, right_j4, right_j5, right_j6, right_j7
]
```

目标 `values` 使用角度制。Kuavo-5 的实时反馈来自：

```python
arm14_rad = msg.joint_data.joint_q[13:27]
left_rad = arm14_rad[:7]
right_rad = arm14_rad[7:]
```

反馈是弧度，不能未经转换直接作为目标数组发布：

```python
import math
arm14_deg = [math.degrees(value) for value in arm14_rad]
```

14 维数组只控制双臂关节，不包含头部、腰部、手指或夹爪。

### 3.2 已定义的标准姿态

| 名称 | 14维角度数组 |
|---|---|
| 双臂放下 `arms_down` | `[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]` |
| 准备姿态 `ready` | `[20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]` |
| 双臂平举、手心向下 | `[0, 15, 70, -70, 90, 0, -20, 0, -15, -70, -70, -90, 0, 20]` |
| 双臂向前、肘部约90°、手心向下 | `[0, 0, 0, -90, -84, 0, 0, 0, 0, 0, -90, 84, 0, 0]` |

这些是当前项目的命名预设，不代表所有现场环境都没有碰撞风险。执行前仍需观察机器人周围空间。

### 3.3 推荐方式：WSL Python 经 SSH 远程控制

入口工具：[kuavo_remote_arm.py](../tools/kuavo_remote_arm.py)

它会自动：

1. SSH 连接机器人；
2. 上传最新版 [kuavo_arm_controller.py](../tools/kuavo_arm_controller.py) 到机器人 `/tmp`；
3. 加载机器人 ROS 工作空间；
4. 设置正确的 ROS Master 和远端 ROS IP；
5. 读取当前反馈、规划或执行动作；
6. 将 JSON 结果返回 WSL。

由于当前 Miniforge 环境没有 Paramiko，应使用系统 Python：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py state
```

脚本会在终端安全提示 SSH 密码。若希望在当前终端只输入一次：

```bash
read -rsp "Robot SSH password: " KUAVO_SSH_PASSWORD
echo
export KUAVO_SSH_PASSWORD
```

密码不会显示，也不会被写入命令历史。不要把密码硬编码进 Python 文件。

### 3.4 执行动作前检查

只读检查远端 ROS 连接：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py doctor
```

应确认输出满足：

```json
{
  "target_topic_tcpros_connections": 2,
  "ready": true
}
```

当前真机曾出现 ROS Master 已列出订阅者，但新发布者约 18 秒后才建立 TCPROS 连接。控制器最多等待 30 秒，因此运行 `--execute` 后可能先等待一段时间再运动。

读取当前双臂状态：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py state
```

输出包含：

```text
arm14_rad       当前反馈，弧度
arm14_deg       当前反馈，角度
left_eef        左手 FK 位置和四元数
right_eef       右手 FK 位置和四元数
feedback_stamp  反馈时间戳
```

### 3.5 传入任意 14 维数组

以“双臂平举、手心向下”为例。第一步只规划，不运动：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4
```

检查 JSON 中至少以下字段：

```text
execute_requested        应为 false
status                   应为 planned
pre_state.arm14_deg      执行前真实反馈
target_arm14_deg         即将使用的目标
largest_joint_change_deg 最大单关节变化
actual_duration_s        控制器最终采用的持续时间
```

确认数组、现场空间和持续时间后，在同一命令末尾显式增加 `--execute`：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4 \
  --execute
```

实际持续时间不会短于 `--duration-s`。控制器还按照最大关节速度约 25°/s 自动延长大幅度动作，避免从当前状态直接快速跳到目标。

若只想改变一只手臂，任意数组接口仍必须提供完整 14 维。另一只手臂应填入刚读取的实际角度，不能随意填零。标准姿态接口可以自动替你保持另一侧：

```bash
# 只规划左臂进入屈肘姿态，右臂采用实时反馈保持
/usr/bin/python3 tools/kuavo_remote_arm.py \
  pose elbows_90_palms_down --arm left --duration-s 4

# 确认后执行
/usr/bin/python3 tools/kuavo_remote_arm.py \
  pose elbows_90_palms_down --arm left --duration-s 4 --execute
```

### 3.6 使用标准姿态名称

查看全部预设：

```bash
python3 tools/kuavo_arm_controller.py list-poses
```

远程规划和执行：

```bash
# 双臂放下，只规划
/usr/bin/python3 tools/kuavo_remote_arm.py pose arms_down --arm both

# 双臂放下，真实执行
/usr/bin/python3 tools/kuavo_remote_arm.py \
  pose arms_down --arm both --duration-s 4 --execute

# 双臂平举，真实执行
/usr/bin/python3 tools/kuavo_remote_arm.py \
  pose arms_horizontal_palms_down --arm both --duration-s 4 --execute
```

### 3.7 控制器内部执行过程

一次带 `--execute` 的 14 维命令执行以下流程：

1. 从 `/sensors_data_raw` 读取一份新鲜反馈；
2. 检查 `/humanoid/mpc/is_arm_executing`，避免与另一条轨迹重叠；
3. 等待 `/kuavo_arm_target_poses` 的真实 TCPROS 连接；
4. 调用 `/arm_traj_change_mode`，将手臂切换为外部控制模式 `2`；
5. 根据当前状态、目标和速度上限确定动作时长；
6. 向 `/kuavo_arm_target_poses` 发布一次目标；
7. 等待动作结束，再读取一份真实反馈；
8. 比较目标误差并记录本次操作。

远端状态记录在：

```text
/tmp/kuavo5_arm_state/latest_state.json
/tmp/kuavo5_arm_state/history.jsonl
```

`latest_state.json` 保存最近反馈；`history.jsonl` 追加保存每次规划、执行或失败记录。下一次动作会重新读取机器人反馈，不会把旧目标误当作当前状态。

### 3.8 底层 ROS 命令

以下命令必须在已经安装 `kuavo_msgs` 且加载机器人工作空间的环境运行。当前推荐在机器人端运行；WSL 尚未安装该自定义消息包。

```bash
cd /home/lab/kuavo-ros-opensource
source devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
```

切换外部手臂控制模式：

```bash
rosservice call /arm_traj_change_mode "control_mode: 2"
```

直接发布 14 维目标：

```bash
rostopic pub -1 /kuavo_arm_target_poses kuavo_msgs/armTargetPoses \
  "times: [4.0]
   values: [0, 15, 70, -70, 90, 0, -20, 0, -15, -70, -70, -90, 0, 20]
   frame: 0"
```

字段含义：

```text
times[0]   到达这一组目标所用时间，秒
values     14 维角度目标
frame      当前单目标命令使用 0
```

底层命令不会自动读取当前状态、检查最大角度变化、等待上一条动作结束或记录执行结果，所以日常开发优先使用 `kuavo_remote_arm.py`。

### 3.9 在机器人 Python 环境直接调用

当 Python 程序本身运行在已经加载机器人 ROS 工作空间的环境时：

```python
from kuavo_arm_controller import KuavoArmController


controller = KuavoArmController(
    robot="5",
    state_dir="/tmp/kuavo5_arm_state",
)

target_deg = [
    0, 15, 70, -70, 90, 0, -20,
    0, -15, -70, -70, -90, 0, 20,
]

# 先生成计划，不运动
plan = controller.move_to_joint_target(
    target_deg,
    duration_s=4.0,
    execute=False,
)
print(plan)

# 明确需要真机动作时才调用
result = controller.move_to_joint_target(
    target_deg,
    duration_s=4.0,
    execute=True,
)
print(result)
```

将脚本复制到机器人 `/tmp` 后，可用：

```bash
export PYTHONPATH=/tmp:$PYTHONPATH
```

然后再导入 `kuavo_arm_controller`。

## 4. 常见故障

### 4.1 当前处于离线状态

可能表现为：

```text
Unable to communicate with master
No route to host
Connection timed out
```

先恢复 WSL 与 `192.168.26.1/192.168.26.12` 的同网段连接，再检查 `ROS_MASTER_URI` 和 `ROS_IP`。离线时仍可查看本手册、检查脚本语法和使用 MuJoCo，但不能取得真机相机帧或状态。

### 4.2 相机 Topic 存在但没有画面

使用：

```bash
timeout 10 rostopic hz <topic>
```

若无新消息，应检查相机主机 `192.168.26.12`、相机驱动和物理连接。不要仅根据 `rostopic info` 判断相机正常。

### 4.3 `/kuavo_arm_target_poses has no subscriber`

先运行：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py doctor
```

当前脚本已明确设置机器人端 `ROS_IP=192.168.26.1`，并把真实 TCPROS 连接等待时间增加到 30 秒。应等待 `ready=true`；Master 中显示 subscriber 但连接数为 0 时不能发布动作。

### 4.4 规划正常但没有执行

不带 `--execute` 是预期的只规划模式。检查输出中的：

```text
execute_requested: false
status: planned
```

只有确认计划后显式添加 `--execute` 才会运动。

### 4.5 单位错误

- `/sensors_data_raw` 的手臂反馈按弧度读取；
- `/kuavo_arm_target_poses.values` 使用角度；
- 深度样本 `depth_mm.npy` 使用毫米；
- 不要在没有单位转换的情况下混用这些数组。

## 5. 本地文件索引

| 文件 | 作用 |
|---|---|
| `tools/view_kuavo_cameras.py` | 三路 RGB 实时查看器，只读 |
| `tools/capture_kuavo_rgbd.py` | 保存一组 RGB、深度、内参和元数据，只读 |
| `tools/kuavo_remote_arm.py` | WSL 经 SSH 调用机器人端控制器 |
| `tools/kuavo_arm_controller.py` | 14维状态控制、标准姿态、FK/IK 和历史记录 |
| `tools/check_kuavo_arm_control.sh` | 只读检查机械臂 ROS 图 |
| `docs/kuavo_interfaces_and_video.md` | 相机格式、内参、FOV 和深度数据详细分析 |
| `docs/kuavo5_real_robot_api_guide.md` | 真机相机、机械臂、笛卡尔移动和 TF 综合记录 |
