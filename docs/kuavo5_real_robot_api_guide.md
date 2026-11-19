# Kuavo-5 真机控制、三相机、笛卡尔步进与 TF 接口

本文整理 2026-09-21 至 2026-09-22 在当前 Kuavo-5 真机上实际核对过的接口。机器人 ROS Master 为 `192.168.26.1:11311`，WSL 当前直连地址为 `192.168.26.11`，相机主机为 `192.168.26.12`。

## 0. WSL 连接环境

每个新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.11
```

验证：

```bash
rostopic list
rostopic info /sensors_data_raw
rostopic info /kuavo_arm_target_poses
```

当前 WSL 的系统 ROS 没有安装 `kuavo_msgs` 生成后的 Python 模块，所以标准图像和 TF 工具可直接在 WSL 运行；`kuavo_arm_controller.py` 应在已经 source 乐聚工作空间的机器人下位机运行，或者先在 WSL 编译 `kuavo_msgs`。

机器人端环境：

```bash
ssh lab@192.168.26.1
# 密码为三个空格

cd /home/lab/kuavo-ros-opensource
source devel/setup.bash
export ROS_MASTER_URI=http://kuavo_master:11311
```

### 0.1 从 WSL 用 Python 经 SSH 自动执行

如果不想每次登录机器人、复制文件、再手动输入机器人端命令，可以直接在 WSL 运行项目中的 SSH 启动器。它会完成以下工作：

1. 连接 `lab@192.168.26.1`；
2. 将本地最新版 `tools/kuavo_arm_controller.py` 上传到机器人 `/tmp`；
3. 在机器人上加载 `/home/lab/kuavo-ros-opensource/devel/setup.bash`；
4. 执行状态查询、轨迹规划或明确授权的运动命令；
5. 将机器人端 JSON 输出和退出状态返回 WSL。

当前 WSL 的 Miniforge Python 没有 Paramiko，因此使用已安装 Paramiko 的系统 Python：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py state
```

执行动作前可运行只读连接诊断：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py doctor
```

只有输出同时满足 `"ready": true` 和 `"target_topic_tcpros_connections"` 大于 `0`，远端临时发布节点才真正连接到了机械臂控制订阅者。当前真机实测新发布者用了约 18 秒才建立两条 TCPROS 连接，因此诊断和动作控制默认最多等待 30 秒。动作控制器先等连接，连接成功后才切换机械臂控制模式；连接失败不会改变控制模式。

SSH 启动器还会为机器人端临时节点设置 `ROS_IP=192.168.26.1`，使节点 URI 明确指向机器人的有线网卡地址。可通过 `doctor --wait-s 45` 临时延长诊断时间。

脚本会安全地提示输入 SSH 密码。输入三个空格后按 Enter；终端不会显示密码字符。也可以在当前终端只输入一次密码，避免后续重复提示，同时不把密码写入脚本或 shell 历史：

```bash
read -rsp "Robot SSH password: " KUAVO_SSH_PASSWORD
echo
export KUAVO_SSH_PASSWORD
```

14 维目标的只规划调用如下。默认不运动：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4
```

检查返回的当前状态、目标值、最大关节变化和实际采用的运动时长后，才在同一条命令末尾增加 `--execute`：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4 \
  --execute
```

标准姿态和基于当前状态的笛卡尔步进使用同一入口：

```bash
# 列出标准姿态
/usr/bin/python3 tools/kuavo_remote_arm.py list-poses

# 规划双臂平举、手心向下
/usr/bin/python3 tools/kuavo_remote_arm.py pose arms_horizontal_palms_down --arm both

# 规划左手沿机器人坐标系向前移动 2 cm
/usr/bin/python3 tools/kuavo_remote_arm.py move \
  --arm left --direction forward --step-m 0.02 --duration-s 2
```

以上 `pose` 和 `move` 示例也只有显式增加 `--execute` 才会发布运动命令。每次调用都会先读取机器人的实时关节反馈；笛卡尔步进从这份反馈对应的末端位姿开始求解，不会假定机器人仍处于上一次目标状态。状态和历史记录保存在机器人 `/tmp/kuavo5_arm_state/`。

SSH 启动器的帮助：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py --help
/usr/bin/python3 tools/kuavo_remote_arm.py move --help
```

这里的 Python 脚本负责从 WSL 自动编排操作，ROS 控制进程仍由 SSH 在机器人上启动，因为当前 WSL 尚未安装 `kuavo_msgs`。如果以后在 WSL 编译好同版本的 `kuavo_msgs` 并正确配置 ROS 网络，就可以直接从 WSL 发布 ROS 消息，不再需要 SSH。

### 0.2 左右手指收紧、张开与自定义发送（2026-10-16）

统一入口为 `tools/kuavo_remote_hand.py`，通过 SSH 在机器人上发布
`/control_robot_hand_position`，读取 `/dexhand/state`。本地使用 `/usr/bin/python3`，
不需要配置 WSL 的 ROS 或上传临时脚本。用户名、地址及密码输入方式与上节一致。

```bash
# 只读查询左右手当前反馈
/usr/bin/python3 tools/kuavo_remote_hand.py state

# 收紧左手、右手或双手；level 是位置参数，不代表握力
/usr/bin/python3 tools/kuavo_remote_hand.py close --hand left --level 80 --execute
/usr/bin/python3 tools/kuavo_remote_hand.py close --hand right --level 80 --execute
/usr/bin/python3 tools/kuavo_remote_hand.py close --hand both --level 80 --execute

# 张开左手、右手或双手
/usr/bin/python3 tools/kuavo_remote_hand.py open --hand left --execute
/usr/bin/python3 tools/kuavo_remote_hand.py open --hand right --execute
/usr/bin/python3 tools/kuavo_remote_hand.py open --hand both --execute

# 单手自定义六维参数
/usr/bin/python3 tools/kuavo_remote_hand.py send --hand left \
  --values 94 80 80 80 80 80 --duration-s 2 --execute

# 双手分别指定参数
/usr/bin/python3 tools/kuavo_remote_hand.py send --hand both \
  --left-values 94 80 80 80 80 80 \
  --right-values 94 0 0 0 0 0 --execute
```

六维顺序为 `[拇指弯曲, 拇指侧摆, 食指, 中指, 无名指, 小指]`，数值范围为
`0..100`，`0` 张开、`100` 收紧，使用的是本机已验证的兼容手指接口。
这与 14 维机械臂角度参数是两个接口。
拇指顺序已核对真机安装的 `hardware_plant/linker_hand_mapping.h`：
`index 0 = ThumbFlexion`，`index 1 = ThumbAbduction`；此前对这两项的口头说明应以此处为准。

`close` 默认将拇指弯曲和其余四指收至 `80`，`open` 将这五个弯曲通道设为 `0`；两者默认保留拇指侧摆通道
的实时反馈，可加 `--thumb-rotation 0` 等参数显式指定。单手调用时，另一手的六通道
由实时反馈填充。脚本默认用两秒逐步发送；`--duration-s` 可调整为 `0.5..30` 秒。
省略 `--execute` 时只查询反馈并生成目标，不创建手指命令发布者。

后续程序可以直接调用 Python API：

```python
from tools.kuavo_remote_hand import KuavoRemoteHand

with KuavoRemoteHand() as hands:  # 密码从 KUAVO_SSH_PASSWORD 或终端提示读取
    result = hands.close("left", level=80, execute=True)
    print(result["post_state"]["left"])
    # hands.open("right", execute=True)
    # hands.send("left", values=[94, 80, 80, 80, 80, 80], execute=True)
    # hands.send("both", left_values=[94, 80, 80, 80, 80, 80],
    #            right_values=[94, 0, 0, 0, 0, 0], execute=True)
```

查询、预览及执行结果均保存在本机 `outputs/robot_control/kuavo5_hands/`，包含
`latest_state.json`、`latest_result.json`、`history.jsonl` 及逐次动作记录。
`--state-dir` 可更改目录。每次操作使用新鲜反馈，不从保存文件恢复姿态。
返回值的 `status=executed` 只表示所选手的通道反馈与目标偏差不超过 10 个位置单位，
不证明抓牢了包裹；`grasp_verified` 保持为 `false`。`feedback_mismatch` 返回非零退出码，
不会自动加大收紧幅度。首次连接若 SSH 主机密钥尚未保存在本机，可在子命令前加
`--accept-new-host-key` 来信任当前机器人密钥。

### 0.3 保存当前手臂位置，并按名称回到预定位置

当前统一调用包含四个点位：15:48 保存的快递点与右臂秤上方点，以及此前 15:15 保存的
左臂中央点和右臂框上方点。单位为米，坐标系为 `base_link`：

| 点位名 | 手臂 | 用户指定含义 | FK 末端坐标 `(x,y,z)` |
| --- | --- | --- | --- |
| `left_parcel_approx` | 左臂 | 近似快递位置 | `(0.545945, 0.361226, 0.077637)` |
| `left_scale_hover` | 左臂 | 秤上方；来源为历史桌面中央附近点 | `(0.606066, -0.005026, 0.215790)` |
| `right_scale_hover` | 右臂 | 秤上方 | `(0.615951, -0.002710, 0.255767)` |
| `right_bin_hover` | 右臂 | 快递框上方；采用调整后确认版本 | `(0.497143, -0.465319, 0.352152)` |

旧文件 `left_table_center_pose.json` 和 `right_bin_drop_pose.json` 原样保留；统一格式的新文件
分别为 `left_scale_hover.json` 和 `right_bin_hover.json`，其中七个目标关节角与旧记录完全一致。
左臂旧记录的原始含义是“桌面中央附近”，按用户随后说明标为秤上方近似点，未经精确秤面标定。
四点索引为 `outputs/robot_control/pose_library/tabletop_saved_poses.json`。

文件位于 `outputs/robot_control/pose_library/<点位名>.json`。保存的是各自实测的七个关节角、
末端位姿、机器人版本及原始双臂反馈；不会自动恢复当时的手指开合状态。

```bash
# 本地列出保存点位，不连接机器人
/usr/bin/python3 tools/kuavo_remote_arm.py list-saved-poses

# 只预览左臂回到近似快递位置
/usr/bin/python3 tools/kuavo_remote_arm.py goto-pose left_parcel_approx

# 实际回到点位；另一个手臂的七维目标由届时的实时反馈填充
/usr/bin/python3 tools/kuavo_remote_arm.py goto-pose left_parcel_approx \
  --duration-s 6 --execute
/usr/bin/python3 tools/kuavo_remote_arm.py goto-pose right_scale_hover \
  --duration-s 6 --execute

# 旧点位：左臂秤上方，以及右臂快递框上方
/usr/bin/python3 tools/kuavo_remote_arm.py goto-pose left_scale_hover \
  --duration-s 6 --execute
/usr/bin/python3 tools/kuavo_remote_arm.py goto-pose right_bin_hover \
  --duration-s 6 --execute

# 手动调整后保存新的单臂点位；此命令只记录，不移动
/usr/bin/python3 tools/kuavo_remote_arm.py save-pose left_parcel_approx \
  --arm left --description "近似快递位置"
/usr/bin/python3 tools/kuavo_remote_arm.py save-pose right_scale_hover \
  --arm right --description "秤上方"
```

省略 `--execute` 只生成回放计划。回放默认限制最大关节变化速度为 `5°/s`，
实际采用时长取请求时长和速度限制所需时长中的较大值。
复用同一名称保存时会保留旧文件到点位目录下的 `.versions/`。
回放前后实测状态和动作记录同时保存在机器人端 `/tmp/kuavo5_arm_state/`
与本机 `outputs/robot_control/saved_pose_replay/`。

回放是关节插值恢复姿态，不包含桌面、包裹或双臂之间的避障规划，也不保证中途末端走直线。
从桌面低位切换前，可先用已有 `move --direction up` 小步抬高；应留出物品及桌沿的间隙。
机器人底座、躯干或桌子相对位置变化后，需要重新记录点位。

## 1. 直接使用 14 维控制双臂

### 1.1 数组含义

`/kuavo_arm_target_poses` 使用角度制：

```text
values = [
  left_j1, left_j2, left_j3, left_j4, left_j5, left_j6, left_j7,
  right_j1, right_j2, right_j3, right_j4, right_j5, right_j6, right_j7
]
```

Kuavo-5 的反馈来自 `/sensors_data_raw`：

```python
arm14_rad = msg.joint_data.joint_q[13:27]
left_rad = arm14_rad[:7]
right_rad = arm14_rad[7:]
```

反馈单位为弧度；`armTargetPoses.values` 使用度。不要把反馈数组未经转换直接发布到目标话题。

### 1.2 推荐的状态控制器

将脚本复制到机器人：

```bash
scp tools/kuavo_arm_controller.py lab@192.168.26.1:/tmp/
```

在机器人端先规划，不运动：

```bash
python3 /tmp/kuavo_arm_controller.py \
  --robot 5 \
  --state-dir /tmp/kuavo5_arm_state \
  joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4
```

输出会包含当前反馈、目标 14 维数组、自动计算的安全时长和最大关节变化。只有增加 `--execute` 才会真实运动：

```bash
python3 /tmp/kuavo_arm_controller.py \
  --robot 5 \
  --state-dir /tmp/kuavo5_arm_state \
  joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4 \
  --execute
```

控制器实际执行以下步骤：

1. 从 `/sensors_data_raw` 读取新鲜的当前状态；
2. 确认 `/humanoid/mpc/is_arm_executing` 没有其他动作；
3. 调用 `/arm_traj_change_mode` 切换到外部机械臂控制模式 `2`；
4. 根据最大关节变化和 25°/s 限制延长时间；
5. 发布一条 `/kuavo_arm_target_poses`；
6. 等待结束并再次读取反馈；
7. 保存 `latest_state.json` 和 `history.jsonl`。

实机当前已确认的依赖接口：

| 名称 | 类型 | 作用 |
|---|---|---|
| `/sensors_data_raw` | `kuavo_msgs/sensorsData` | 读取当前关节反馈 |
| `/humanoid/mpc/is_arm_executing` | `std_msgs/Bool` | 判断机械臂是否正在执行 |
| `/arm_traj_change_mode` | `kuavo_msgs/changeArmCtrlMode` | 切换外部控制模式 |
| `/ik/fk_srv` | `kuavo_msgs/fkSrv` | 14 维关节角到双手末端位姿 |
| `/ik/two_arm_hand_pose_cmd_srv` | `kuavo_msgs/twoArmHandPoseCmdSrv` | 双手末端目标到 14 维关节角 |
| `/kuavo_arm_target_poses` | `kuavo_msgs/armTargetPoses` | 发布角度制双臂目标 |

Python 调用：

```python
# 机器人 shell 先执行：export PYTHONPATH=/tmp:$PYTHONPATH
from kuavo_arm_controller import KuavoArmController

arm = KuavoArmController(robot="5", state_dir="/tmp/kuavo5_arm_state")

target_deg = [
    0, 15, 70, -70, 90, 0, -20,
    0, -15, -70, -70, -90, 0, 20,
]

plan = arm.move_to_joint_target(target_deg, duration_s=4, execute=False)
# 人工检查 plan 后：
result = arm.move_to_joint_target(target_deg, duration_s=4, execute=True)
```

### 1.3 最底层 ROS 发布方式

机器人端可以直接调用：

```bash
rosservice call /arm_traj_change_mode "control_mode: 2"

rostopic pub -1 /kuavo_arm_target_poses kuavo_msgs/armTargetPoses \
  "times: [4.0]
   values: [0, 15, 70, -70, 90, 0, -20, 0, -15, -70, -70, -90, 0, 20]
   frame: 0"
```

这条发布命令会直接产生真实运动。它不包含状态重读、目标误差检查和历史记录，所以重复开发时优先使用状态控制器。

## 2. 三个相机 API

这里的“三个相机”指头部 `cam_h`、左腕 `cam_l`、右腕 `cam_r`。

| 相机 | RGB 压缩图 | 彩色内参 | 深度图 | 深度内参 | 2026-09-22 实测 |
|---|---|---|---|---|---|
| 头部 | `/cam_h/color/image_raw/compressed` | `/cam_h/color/camera_info` | `/cam_h/depth/image_raw` | `/cam_h/depth/camera_info` | RGB 约 31.4 Hz |
| 左腕 | `/cam_l/color/image_raw/compressed` | `/cam_l/color/camera_info` | `/cam_l/depth/image_rect_raw` | `/cam_l/depth/camera_info` | RGB 约 31.3 Hz |
| 右腕 | `/cam_r/color/image_raw/compressed` | `/cam_r/color/camera_info` | `/cam_r/aligned_depth_to_color/image_raw` | `/cam_r/aligned_depth_to_color/camera_info` | 节点已登记，但 10 秒无新帧 |

三路 RGB 消息类型都是 `sensor_msgs/CompressedImage`。已收到的头部和左腕消息格式均为：

```text
rgb8; jpeg compressed bgr8
```

分辨率均为 848×480。实时读到的有效 FOV：

```text
头部彩色：水平 92.54°，垂直 61.22°
左腕彩色：水平 88.30°，垂直 57.59°
左腕深度：水平 89.26°，垂直 58.38°
```

### 2.1 用 GUI 直接查看

查看单路：

```bash
rqt_image_view /cam_h/color/image_raw/compressed
rqt_image_view /cam_l/color/image_raw/compressed
rqt_image_view /cam_r/color/image_raw/compressed
```

每个命令放在独立终端，即可同时开三个窗口。

项目新增的三画面查看器会将三路画面并排显示，并标记等待帧或过期帧：

```bash
/usr/bin/python3 tools/view_kuavo_cameras.py --camera all
```

只看某一路：

```bash
/usr/bin/python3 tools/view_kuavo_cameras.py --camera head
/usr/bin/python3 tools/view_kuavo_cameras.py --camera left
/usr/bin/python3 tools/view_kuavo_cameras.py --camera right
```

按 `q` 或 Esc 退出。该工具只订阅图像，不发布任何控制消息。

### 2.2 查看格式、帧率和带宽

```bash
rostopic echo --noarr -n 1 /cam_h/color/image_raw/compressed
rostopic echo -n 1 /cam_h/color/camera_info
rostopic hz /cam_h/color/image_raw/compressed
rostopic bw /cam_h/color/image_raw/compressed
```

将 `cam_h` 改成 `cam_l` 或 `cam_r` 即可查询腕部相机。检查右腕时不要只看 `rostopic info` 中有没有 publisher，还应运行：

```bash
timeout 10 rostopic hz /cam_r/color/image_raw/compressed
```

当前结果是 `no new messages`，说明右腕驱动节点存在，但没有图像数据到达。

### 2.3 Python 订阅方式

```python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

def callback(msg):
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    print(msg.header.frame_id, msg.header.stamp.to_sec(), bgr.shape)

rospy.init_node("camera_reader")
sub = rospy.Subscriber(
    "/cam_h/color/image_raw/compressed",
    CompressedImage,
    callback,
    queue_size=1,
    buff_size=2**22,
    tcp_nodelay=True,
)
rospy.spin()
```

### 2.4 保存 RGB-D

采集工具现在支持四个前缀，其中本任务使用 `cam_h/cam_l/cam_r`：

```bash
/usr/bin/python3 tools/capture_kuavo_rgbd.py \
  --camera cam_h \
  --output /mnt/e/Data/Picture/head_test
```

左腕会默认使用 `/cam_l/depth/image_rect_raw`；右腕会默认使用对齐深度 `/cam_r/aligned_depth_to_color/image_raw`。右腕当前无帧时脚本会超时退出，不应无限等待。

## 3. 平移参数转换为 14 维数组

对应脚本：

```text
tools/kuavo_arm_controller.py
tools/kuavo_cartesian_step_to_arm14.py   # 兼容入口
```

它处理的是末端执行器的六方向笛卡尔增量，不是把二维像素直接变成关节角。方向定义在 `waist_yaw_link`：

```text
forward  = +X
backward = -X
left     = +Y
right    = -Y
up       = +Z
down     = -Z
```

转换流程：

```text
/sensors_data_raw 的 joint_q[13:27]
        ↓ 当前双臂 14 维弧度
/ik/fk_srv
        ↓ 当前左右末端 xyz + quaternion
选定手臂 xyz 增加 Δx/Δy/Δz，姿态保持不变
        ↓
/ik/two_arm_hand_pose_cmd_srv
        ↓ 目标 q_arm[14]（弧度）
转换成度
        ↓
/kuavo_arm_target_poses
```

每次最多移动 0.05 m。未选中的手臂使用当前反馈和当前末端姿态，不会被重置成某个预设值。

先求解并查看生成的 14 维数组，不运动：

```bash
python3 /tmp/kuavo_arm_controller.py \
  --robot 5 \
  --state-dir /tmp/kuavo5_arm_state \
  move --arm left --direction forward --step-m 0.02 --duration-s 2
```

输出中的关键字段：

```text
pre_state.arm14_deg
ik.current_xyz_m
ik.requested_xyz_m
ik.achieved_xyz_m
ik.position_error_mm
ik.target_arm14_deg
status
```

实际执行只是在相同命令最后增加 `--execute`。下一次命令会重新读取真机反馈，再以该反馈求解，不依赖旧 JSON 作为 IK 初值。

Python API：

```python
arm.move_relative(
    arm="left",
    direction="forward",
    step_m=0.02,
    duration_s=2.0,
    execute=False,
)
```

## 4. TF 树相关 API

### 4.1 ROS 标准接口

```text
/tf         tf2_msgs/TFMessage，动态变换
/tf_static  tf2_msgs/TFMessage，静态变换
```

命令行查询：

```bash
rosrun tf tf_echo base_link zarm_l7_end_effector
rosrun tf tf_echo base_link zarm_r7_end_effector
rosrun rqt_tf_tree rqt_tf_tree
```

`tf_echo base_link camera_frame` 表示查询把 `camera_frame` 坐标转换到 `base_link` 所需的变换。

项目只读工具：

```bash
/usr/bin/python3 tools/kuavo_tf_inspector.py list --check-cameras

/usr/bin/python3 tools/kuavo_tf_inspector.py lookup \
  --target base_link \
  --source cam_h_color_optical_frame

/usr/bin/python3 tools/kuavo_tf_inspector.py lookup \
  --target base_link \
  --source cam_h_color_optical_frame \
  --point 0.10 0.05 0.80
```

最后一个命令把相机坐标系中的 `[0.10, 0.05, 0.80]` 米转换到 `base_link`。

当前 WSL 中 `kuavo_master` 被解析到 `198.18.0.58`，而机器人直连地址是 `192.168.26.1`。因此 WSL 可以收到位于 `192.168.26.12` 的相机 TF，却可能收不到发布 URI 为 `http://kuavo_master:...` 的机器人本体 TF。需要在 WSL 修正 `kuavo_master -> 192.168.26.1` 的主机名解析，或把该工具复制到机器人端运行。这个网络问题会让 WSL 报告 `base_link does not exist`；在机器人本机查询可以看到 `base_link`，但仍然确认相机树与机器人树彼此断开。

Python 标准 API：

```python
import rospy
import tf2_ros

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)

transform = buffer.lookup_transform(
    "base_link",                    # target：输出坐标系
    "cam_h_color_optical_frame",    # source：输入坐标系
    rospy.Time(0),                   # 最新变换
    rospy.Duration(2.0),
)
```

### 4.2 当前真机 TF 实测结果

机器人本体树存在：

```text
odom
└── base_link
    └── waist_yaw_link
        ├── zhead_1_link -> zhead_2_link -> head_camera_base -> head_camera_depth
        ├── zarm_l1_link ... zarm_l7_link -> zarm_l7_end_effector
        └── zarm_r1_link ... zarm_r7_link -> zarm_r7_end_effector
```

相机驱动另外发布了内部树：

```text
cam_h_link -> cam_h_depth_frame -> cam_h_color_frame -> cam_h_color_optical_frame
cam_l_link -> cam_l_depth_frame / cam_l_color_frame -> optical frames
cam_r_link -> ...  # 当前没有收到右腕 TF/图像帧
```

实测 `base_link -> cam_h_color_optical_frame` 查询失败，原因是机器人本体树和相机驱动树没有连接。左腕同样没有 `zarm_l7_end_effector -> cam_l_link`；右腕当前连相机内部帧也没有收到。

因此现在还不能仅靠 TF 将检测点可靠地从相机转换到机械臂坐标。必须补齐并校准三个安装外参：

```text
head_camera_depth 或 head_camera_base -> cam_h_link
zarm_l7_end_effector                  -> cam_l_link
zarm_r7_end_effector                  -> cam_r_link
```

头部安装外参是固定在随头部转动的机械链上的静态变换；腕部安装外参是末端到相机的静态手眼变换，而 `base_link` 到末端由动态关节 TF 自动提供。

官方仓库包含 `src/Camera_Calibration` 三相机棋盘标定流程，但示例使用 `/head_camera`、`/left_wrist_camera`、`/right_wrist_camera` 命名；当前真机是 `/cam_h`、`/cam_l`、`/cam_r`，使用前必须修改或 remap topic 和 frame。仓库中的示例标定数值不应直接套用到当前机器人。

标定得到真实的平移和四元数后，才可发布桥接 TF：

```bash
rosrun tf2_ros static_transform_publisher \
  X Y Z QX QY QZ QW \
  zarm_l7_end_effector cam_l_link
```

这里的 `X..QW` 必须来自当前机器人标定。桥接后先验证：

```bash
rosrun tf tf_echo base_link cam_l_color_optical_frame
```

只有该查询连续成功且相机移动方向、轴方向和实际位置检查一致后，才能把视觉三维点交给机械臂 IK。
