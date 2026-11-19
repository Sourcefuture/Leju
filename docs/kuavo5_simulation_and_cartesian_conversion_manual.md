# Kuavo-5 仿真与现实坐标转 14 维手册

新增称重工作台的启动、GUI双终端操作和秤台验证见[Kuavo-5称重场景仿真启动流程](kuavo5_weighing_simulation.md)。

本文整理当前项目中与以下工作有关的本地实现：

1. 使用官方 Kuavo-5.2 MJCF 做离线姿态、关节数组和笛卡尔移动验证；
2. 一边输入命令，一边在 MuJoCo GUI 中观察动作；
3. 将机器人坐标系中的末端相对位移转换成双臂 14 维关节目标；
4. 从 RGB-D 物体坐标走向机器人坐标和机械臂目标所需的完整链路；
5. 当前仿真和现实坐标转换尚未覆盖的部分。

本文可在机器人离线时使用。所有带 MuJoCo 后端的命令只修改本地仿真状态，不会连接或控制真机。

相机Topic、RGB-D采集和直接14维真机控制见[Kuavo-5 摄像头与双臂14维控制手册](kuavo5_camera_and_arm14_manual.md)。

## 1. 当前本地仿真环境

当前已确认存在：

```text
MuJoCo Python: 3.13.0

.sim/kuavo-ros-opensource/
└── src/kuavo_assets/models/biped_s52/xml/scene.xml

.sim/kuavo-data-challenge-branch/
└── src/data_challenge_simulator/models/biped_s45/xml/scene1.xml
```

项目脚本固定使用 `letools` 环境：

```text
/home/source/miniforge3/envs/letools/bin/python
```

快速检查：

```bash
cd /home/source/LeTools-Learning

/home/source/miniforge3/envs/letools/bin/python -c \
  'import mujoco; print(mujoco.__version__)'

test -f .sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52/xml/scene.xml \
  && echo "Kuavo-5.2 scene OK"
```

GUI 需要 WSLg 或有效的 X Server：

```bash
echo "$DISPLAY"
```

无 GUI 时仍可使用 EGL 生成视频、图片和 JSON 报告。

## 2. 三种仿真使用方式

### 2.1 姿态和任意 14 维数组的离屏视频

入口：[run_kuavo_mujoco_demo.sh](../tools/run_kuavo_mujoco_demo.sh)

生成默认标准姿态视频：

```bash
cd /home/source/LeTools-Learning
tools/run_kuavo_mujoco_demo.sh
```

默认输出：

```text
outputs/kuavo_sim/kuavo5_arm_pose_demo.mp4
outputs/kuavo_sim/kuavo5_arm_pose_demo_poses.png
outputs/kuavo_sim/kuavo5_arm_pose_demo.json
```

视频依次显示双臂放下、准备姿态、双臂平举、屈肘约90°和重新放下。JSON报告记录模型、14维顺序、帧率以及是否发生关节限位裁剪。

验证任意14维数组：

```bash
tools/run_kuavo_mujoco_demo.sh \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20
```

该命令把角度转换为弧度并写入以下关节：

```text
zarm_l1_joint ... zarm_l7_joint
zarm_r1_joint ... zarm_r7_joint
```

如果输入超出MJCF中的关节限位，视频脚本会裁剪，并在报告的 `joint_limit_clips` 中记录请求值和裁剪值。仿真中的裁剪不能替代真机安全检查。

### 2.2 打开循环播放的 MuJoCo 窗口

```bash
tools/run_kuavo_mujoco_demo.sh --live
```

这会使用 GLFW 打开 WSLg 窗口，循环播放预设动作。它适合快速查看关节数组对应的大致姿态，但不是交互控制器，也不会保存新的真机状态。

### 2.3 状态连续的命令行控制和实时 GUI

这一方式使用同一套状态控制接口连续测试多条命令。

终端A启动GUI：

```bash
cd /home/source/LeTools-Learning
tools/run_kuavo_live_viewer.sh
```

终端B初始化仿真到准备姿态：

```bash
cd /home/source/LeTools-Learning
tools/kuavo_simctl.sh \
  --reset-sim \
  pose ready --duration-s 2 --execute
```

然后继续输入动作：

```bash
# 左臂前移 2 cm
tools/kuavo_simctl.sh move \
  --arm left --direction forward --step-m 0.02 \
  --duration-s 1.5 --execute

# 左臂上移 2 cm
tools/kuavo_simctl.sh move \
  --arm left --direction up --step-m 0.02 \
  --duration-s 1.5 --execute

# 右臂向机器人左侧移动 3 cm
tools/kuavo_simctl.sh move \
  --arm right --direction left --step-m 0.03 \
  --duration-s 1.5 --execute
```

GUI监视：

```text
outputs/robot_control/kuavo5_mujoco/history.jsonl
outputs/robot_control/kuavo5_mujoco/latest_state.json
```

控制器执行一条动作后将目标写入历史，GUI读取新增记录并平滑播放。只有状态为 `executed` 的记录会产生动画；不带 `--execute` 的计划不会移动GUI模型。

GUI启动时会读取 `latest_state.json` 显示当前状态，但不会重播启动前已经存在的全部历史。

### 2.4 使用独立状态目录

多个实验应使用不同状态目录，避免上一组动作影响下一组：

```bash
tools/run_kuavo_live_viewer.sh \
  --state-dir /tmp/kuavo_test_a

tools/kuavo_simctl.sh \
  --state-dir /tmp/kuavo_test_a \
  --reset-sim pose ready --execute

tools/kuavo_simctl.sh \
  --state-dir /tmp/kuavo_test_a \
  move --arm left --direction forward --step-m 0.02 --execute
```

查看器和控制器必须使用同一个 `--state-dir`。

`--reset-sim` 表示忽略保存的旧关节状态，从模型Home状态重新开始。后续不带该参数的命令会恢复上一条仿真执行后的14维状态。

## 3. 批量验证六方向坐标转换

入口：[test_kuavo_cartesian_sim.py](../tools/test_kuavo_cartesian_sim.py)

测试左右臂各自的前、后、左、右、上、下六个方向：

```bash
/home/source/miniforge3/envs/letools/bin/python \
  tools/test_kuavo_cartesian_sim.py \
  --step-m 0.02
```

测试5 cm单步：

```bash
/home/source/miniforge3/envs/letools/bin/python \
  tools/test_kuavo_cartesian_sim.py \
  --step-m 0.05 \
  --output outputs/kuavo_sim/cartesian_conversion_test_50mm
```

每次测试先进入 `ready`，然后连续执行12项移动，并检查：

- IK是否接受目标；
- 末端目标和求解结果的误差；
- 执行后末端误差；
- 上一条动作的后状态是否等于下一条动作的前状态；
- 左右臂所有方向是否都能完成。

已有离线结果：

```text
20 mm：12/12通过，最大末端误差约0.0164 mm
50 mm：12/12通过，最大末端误差约0.0185 mm
相邻动作状态连续性误差：0°
```

输出包括：

```text
outputs/kuavo_sim/cartesian_conversion_test.json
outputs/kuavo_sim/cartesian_conversion_test.mp4
outputs/kuavo_sim/cartesian_conversion_test_state/
```

这些误差是MuJoCo运动学求解误差，不代表真机关节跟踪误差、结构变形或相机定位误差。

## 4. 零件分拣视觉场景

入口：[prepare_kuavo5_lingjian_scene.py](../tools/prepare_kuavo5_lingjian_scene.py)

该脚本把官方挑战任务1房间、桌面、传送带和物体，与官方Kuavo-5.2视觉模型组合：

```bash
python tools/prepare_kuavo5_lingjian_scene.py
```

生成：

```text
outputs/kuavo_sim/scenes/biped_s52_visual.xml
outputs/kuavo_sim/scenes/lingjian_task1_kuavo52_visual.xml
```

生成姿态回放视频：

```bash
tools/run_kuavo_mujoco_demo.sh \
  --scene outputs/kuavo_sim/scenes/lingjian_task1_kuavo52_visual.xml \
  --output outputs/kuavo_sim/kuavo52_lingjian_pose_demo.mp4
```

实时打开该场景：

```bash
tools/run_kuavo_mujoco_demo.sh \
  --scene outputs/kuavo_sim/scenes/lingjian_task1_kuavo52_visual.xml \
  --live
```

这是视觉和运动学组合场景，当前不等价于完整Kuavo-5 ROS仿真。它没有完成以下闭环：

- MPC/WBC动力学控制；
- `/simulator/init/reset/start/success`服务；
- 三路真机等价RGB-D Topic；
- 夹爪物理抓取和任务评分；
- LingBot-VLA-V2 checkpoint推理。

官方data-challenge分支当前提供的ROS控制和评分主体是 `biped_s45`，即Kuavo 4.5。仅把视觉模型替换为Kuavo-5.2，不会自动让控制器、关节索引和评分逻辑兼容Kuavo-5。

## 5. 现实坐标转14维脚本

### 5.1 文件关系

| 文件 | 作用 |
|---|---|
| [kuavo_arm_controller.py](../tools/kuavo_arm_controller.py) | 主状态控制器；真机与MuJoCo共享的命令入口 |
| [kuavo_cartesian_step_to_arm14.py](../tools/kuavo_cartesian_step_to_arm14.py) | 兼容入口；把六方向相对位移交给主控制器 |
| [kuavo_mujoco_controller.py](../tools/kuavo_mujoco_controller.py) | 官方Kuavo-5.2 MJCF的FK、IK和仿真执行后端 |
| [kuavo_remote_arm.py](../tools/kuavo_remote_arm.py) | WSL经SSH调用机器人端真实ROS控制器 |

核心实现位于 `KuavoArmController.move_relative()`。兼容脚本没有另一套独立算法，目的是保证旧命令也使用实时反馈、同一IK流程和同一历史记录。

### 5.2 当前“现实坐标”的准确含义

当前脚本处理的是机器人腰部参考系 `waist_yaw_link` 中的末端相对平移：

| direction | 位移 |
|---|---|
| `forward` | `+X`，机器人前方 |
| `backward` | `-X`，机器人后方 |
| `left` | `+Y`，机器人左侧 |
| `right` | `-Y`，机器人右侧 |
| `up` | `+Z`，上方 |
| `down` | `-Z`，下方 |

单位为米，单步范围强制限制为：

```text
0 < step_m <= 0.05
```

“左”和“右”始终是机器人整体坐标方向。例如右臂执行 `left`，末端向机器人左侧移动，通常表现为向身体内侧靠近。

当前接口不是以下几种转换：

- 不是二维图像像素直接变成14维关节角；
- 不是房间或地图中的全局坐标；
- 不是任意绝对XYZ目标接口；
- 不改变手掌姿态或指定新的四元数；
- 不控制夹爪或手指。

### 5.3 真机转换流程

```text
/sensors_data_raw
    │ joint_q[13:27]，14维弧度
    ▼
/ik/fk_srv
    │ 当前左右手 xyz + quaternion
    ▼
选定手 xyz 加上 Δx/Δy/Δz
    │ 选定手姿态保持不变
    │ 另一只手位置和姿态保持当前值
    ▼
/ik/two_arm_hand_pose_cmd_srv
    │ 当前14维反馈作为IK初值 q0
    ▼
q_arm[14]，弧度
    ▼ radians → degrees
target_arm14_deg[14]
    ▼ 只有 execute=True 时
/kuavo_arm_target_poses
```

真机IK请求使用：

- `joint_angles_as_q0=True`，当前关节反馈作为初值；
- 位置容差默认5 mm；
- 当前手掌四元数保持不变；
- 双臂一起送入IK，未选择的一侧保持当前末端位姿；
- IK结果必须成功、长度为14且位置误差不超过阈值。

在发布前，程序再次读取一份实时关节状态。如果求解期间机器人关节变化超过1°，它会基于新状态重新做一次FK和IK。若重新求解失败，本次动作记为 `rejected_after_state_change`，不会发布。

### 5.4 MuJoCo转换流程

MuJoCo后端继承同一个 `move_relative()` 状态机，但替换了FK、IK和执行方法：

- 14维状态来自官方模型的当前 `qpos`；
- 左右末端参考 `zarm_l7_link`、`zarm_r7_link`；
- 根据Kuavo-5.2参数增加0.193 m手部参考偏移；
- 使用保持位置和方向的6D阻尼最小二乘IK；
- 每次最多300次迭代；
- 每步关节增量限制为0.08 rad；
- 每轮按MJCF关节上下限裁剪；
- 规划阶段求得目标后恢复原仿真状态，只有 `--execute` 才改变保存的状态。

仿真固定基座时，模型世界轴与机器人前、左、上方向对齐，因此六方向命令与真机约定一致。

### 5.5 真机调用

通过SSH远程规划左手前移2 cm，不运动：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py move \
  --arm left \
  --direction forward \
  --step-m 0.02 \
  --duration-s 2
```

关键输出：

```text
pre_state.arm14_deg       当前真实14维反馈
ik.current_xyz_m          当前末端坐标
ik.requested_xyz_m        请求的末端坐标
ik.achieved_xyz_m         IK解对应的末端坐标
ik.position_error_mm      IK位置误差
ik.target_arm14_deg       转换得到的14维角度数组
status                    planned/rejected
```

确认 `ik.accepted=true`、误差、目标数组和现场空间后才执行：

```bash
/usr/bin/python3 tools/kuavo_remote_arm.py move \
  --arm left \
  --direction forward \
  --step-m 0.02 \
  --duration-s 2 \
  --execute
```

每条SSH命令会创建新的远端ROS节点。上次实测新发布者建立TCPROS连接约需18秒，控制器最多等待30秒后才发布动作。

兼容入口若复制到机器人并加载好ROS环境，也可调用：

```bash
python3 /tmp/kuavo_cartesian_step_to_arm14.py \
  --robot 5 \
  --arm left \
  --direction up \
  --step-m 0.02 \
  --duration-s 2
```

末尾增加 `--execute` 才会控制真机。

### 5.6 MuJoCo调用

只求解并查看转换出的14维数组：

```bash
tools/kuavo_simctl.sh \
  move --arm left --direction forward --step-m 0.02
```

在仿真中执行并保存状态：

```bash
tools/kuavo_simctl.sh \
  move --arm left --direction forward --step-m 0.02 \
  --duration-s 1.5 --execute
```

直接使用兼容脚本：

```bash
/home/source/miniforge3/envs/letools/bin/python \
  tools/kuavo_cartesian_step_to_arm14.py \
  --backend mujoco \
  --reset-sim \
  --arm left \
  --direction forward \
  --step-m 0.02
```

### 5.7 Python API

真机机器人端：

```python
from kuavo_arm_controller import KuavoArmController


arm = KuavoArmController(
    robot="5",
    state_dir="/tmp/kuavo5_arm_state",
)

plan = arm.move_relative(
    arm="left",
    direction="forward",
    step_m=0.02,
    duration_s=2.0,
    execute=False,
)

if plan["status"] == "planned" and plan["ik"]["accepted"]:
    print(plan["ik"]["target_arm14_deg"])
```

MuJoCo：

```python
from tools.kuavo_mujoco_controller import MuJoCoKuavoArmController


sim = MuJoCoKuavoArmController(
    state_dir="/tmp/kuavo_python_sim",
    resume=False,
)
sim.ready(duration_s=1.0, execute=True)

plan = sim.move_relative(
    arm="left",
    direction="forward",
    step_m=0.02,
    duration_s=1.0,
    execute=False,
)
print(plan["ik"]["target_arm14_deg"])
```

## 6. 一次离线转换示例

本文生成前重新运行了以下离线测试：

```bash
tools/kuavo_simctl.sh \
  --reset-sim --state-dir /tmp/kuavo_doc_validation \
  pose ready --duration-s 1 --execute

tools/kuavo_simctl.sh \
  --state-dir /tmp/kuavo_doc_validation \
  move --arm left --direction forward --step-m 0.02
```

主要结果：

```text
当前左手 xyz：[-0.02312, 0.25270, 0.65620] m
请求左手 xyz：[-0.00312, 0.25270, 0.65620] m
IK位置误差：约0.0156 mm

目标14维角度约为：
[18.6900, 0, 0, -31.7872, 0, 0, 3.0965,
 20.0000, 0, 0, -30.0000, 0, 0, 0]
```

右臂14维部分保持准备姿态不变。该数组只对应这次仿真初始状态，不能复制后直接用于任意真机状态。

## 7. 从相机物体坐标到14维还缺少什么

### 7.1 几何链路

相机像素 `(u,v)` 和深度 `d_mm` 先通过相机内参恢复光学坐标：

```python
Z = d_mm / 1000.0
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
p_camera = [X, Y, Z]
```

ROS光学坐标通常为：

```text
+X：图像右侧
+Y：图像下方
+Z：相机前方
```

然后必须使用标定外参：

```text
p_waist = R_waist_camera × p_camera + t_waist_camera
```

得到物体中心后，还要生成抓取目标：

```text
p_hover = p_object + 安全高度偏移
目标手掌四元数 = 根据抓取方向设置
```

最后才把完整末端位姿交给IK，生成14维目标。

### 7.2 当前TF限制

上次真机检查发现相机驱动TF树与机器人本体TF树没有连接：

```text
base_link -> cam_h_color_optical_frame              查询失败
zarm_l7_end_effector -> cam_l_link                  查询失败
zarm_r7_end_effector -> cam_r_link                  未获得有效链路
```

缺少的固定变换至少包括：

```text
机器人头部相机安装坐标 -> cam_h_link
左手末端安装坐标       -> cam_l_link
右手末端安装坐标       -> cam_r_link
```

在完成手眼标定并发布静态TF前，不能把图像检测结果直接当成 `waist_yaw_link` 坐标控制真机。

只读检查：

```bash
/usr/bin/python3 tools/kuavo_tf_inspector.py list --check-cameras

/usr/bin/python3 tools/kuavo_tf_inspector.py lookup \
  --target base_link \
  --source cam_h_color_optical_frame
```

### 7.3 三包裹坐标估计脚本

入口：[analyze_package_targets.py](../tools/analyze_package_targets.py)

它针对之前采集的桌面三包裹画面，完成：

1. 用场景特定HSV阈值找两个紫色包裹和一个黑色包裹；
2. 在目标掩膜内使用原始毫米深度反投影；
3. 拟合桌面平面估计相机滚转和俯仰；
4. 使用给定的相机原点和安装方向近似变换到 `waist_yaw_link`；
5. 在物体上方增加默认15 cm安全高度；
6. 输出标注图和估计坐标。

调用：

```bash
python tools/analyze_package_targets.py \
  /mnt/e/Data/Picture/<某次RGB-D采集目录>
```

输出：

```text
targets.json
targets_annotated.jpg
```

默认近似相机原点：

```text
[0.094, 0.044, 0.569] m in waist_yaw_link
```

该脚本是针对一次固定桌面、颜色和视角的分析工具。由于真实TF链断开，旋转由桌面拟合、平移和偏航来自安装假设，其 `hover_waist_m_est` 只能用于保守规划和校验，不能未经标定直接驱动机械臂。

## 8. 当前功能边界和下一步

当前已完成：

- 14维数组到Kuavo-5.2 MJCF关节的映射；
- 四个标准姿态仿真；
- 左右手六方向、最多5 cm的相对末端平移；
- 真机FK/IK服务与MuJoCo本地IK两套后端；
- 每次从当前状态求解，并记录执行前后状态；
- GUI实时回放和离屏视频；
- 固定桌面三包裹的RGB-D坐标估计原型。

当前未完成：

- 命令行形式的任意绝对 `x,y,z` 目标；
- 末端姿态、欧拉角或四元数控制接口；
- 相机到机器人基座的可靠标定和完整TF树；
- 碰撞检测、桌面约束和自碰撞规划；
- 夹爪抓取闭环；
- Kuavo-5完整ROS动力学和任务评分仿真；
- LingBot-VLA-V2训练checkpoint的仿真推理闭环。

建议的实现顺序：

1. 完成头部和腕部相机外参标定并补齐TF；
2. 在控制器中新增绝对末端位姿接口，输入 `xyz + quaternion`；
3. 先在MuJoCo验证可达性、关节限位和连续性；
4. 增加桌面、机器人本体和双臂碰撞检查；
5. 真机先执行物体上方较高的hover点，再逐步降低；
6. 最后加入夹爪和视觉反馈闭环。

## 9. 本地文件索引

| 文件 | 内容 |
|---|---|
| `tools/kuavo_arm_controller.py` | 真机/仿真统一状态控制和六方向移动 |
| `tools/kuavo_cartesian_step_to_arm14.py` | 笛卡尔相对步进转14维兼容入口 |
| `tools/kuavo_mujoco_controller.py` | Kuavo-5.2 MuJoCo FK、IK和执行后端 |
| `tools/kuavo_mujoco_demo.py` | 标准姿态、任意14维数组视频和循环GUI |
| `tools/kuavo_mujoco_live_viewer.py` | 监听历史记录的实时GUI |
| `tools/kuavo_simctl.sh` | 固定使用 `letools` 环境的仿真命令入口 |
| `tools/run_kuavo_live_viewer.sh` | 实时GUI入口 |
| `tools/test_kuavo_cartesian_sim.py` | 左右臂六方向批量测试 |
| `tools/prepare_kuavo5_lingjian_scene.py` | 组合任务1视觉场景与Kuavo-5.2模型 |
| `tools/analyze_package_targets.py` | 固定桌面三包裹RGB-D位置估计原型 |
| `docs/kuavo_simulation_smoke_test.md` | 已完成仿真验证和完整闭环边界 |
| `docs/kuavo_stateful_arm_control.md` | 状态控制器原始使用说明 |
| `docs/kuavo5_camera_and_arm14_manual.md` | 摄像头和14维真机控制手册 |
