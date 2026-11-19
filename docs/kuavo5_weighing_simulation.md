# Kuavo-5 称重场景仿真启动流程

本场景使用此前仿真的同一份官方 Kuavo-5.2 模型 `biped_s52`，可在 WSL 离线运行。它提供称重工作台、三种质量的物体、秤盘和放置区，并继续支持原有14维控制和六方向末端移动。

手臂GUI使用运动学回放；秤台另外使用重力与接触动力学检验重量。当前尚未实现机器人自主抓取、放物、松手、读取重量的完整闭环。Kuavo-5.2模型也不意味着已核对所有Kuavo-5硬件变体的精确尺寸和惯量。

![Kuavo-5.2称重工作台](../outputs/kuavo_sim/weighing/weighing_overview.png)

## 第一步 检查环境

在WSL终端进入项目：

```bash
cd /home/source/LeTools-Learning
/home/source/miniforge3/envs/letools/bin/python -c 'import mujoco, cv2, numpy; print(mujoco.__version__)'
```

当前已核对的MuJoCo版本为 `3.13.0`。启动脚本固定调用上述 `letools` Python，因此不依赖终端是否已执行 `conda activate letools`。

机器人资源必须存在：

```text
.sim/kuavo-ros-opensource/src/kuavo_assets/models/biped_s52/
├── xml/biped_s52.xml
└── meshes/
```

GUI需要WSLg显示环境：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

这里不需要启动ROS Master、Docker、SSH、真实机器人或模型推理服务，也不需要联网下载模型。

## 第二步 生成称重场景

```bash
bash tools/kuavo_weighing.sh build
```

它读取本地官方MJCF，生成场景副本，保留原有机器人关节、网格和14维映射。默认三个物体放在桌面左侧，秤盘为空。

| 元素 | 默认配置 |
|---|---|
| 机器人 | 官方Kuavo-5.2，与之前使用的模型相同 |
| 桌面 | 长0.76 m、宽1.24 m、高0.76 m |
| 秤盘 | 0.30 × 0.26 m，上表面高0.844 m |
| 蓝色物体A | 0.2 kg |
| 橙色物体B | 0.5 kg |
| 紫色物体C | 1.0 kg |
| 绿色放置区 | 机器人右侧，称重后放置位置示意 |
| 黄色标记 | 秤上方参考点 `(0.60, 0, 1.02)` m；可达性尚未验证 |

生成文件：

```text
outputs/kuavo_sim/weighing/
├── scene.xml             # 带Kuavo-5.2的完整视觉场景
├── scale_bench.xml       # 仅秤台和物体的接触动力学测试场景
└── scene_info.json       # 物体质量、参考点和坐标系说明
```

上述坐标使用MuJoCo世界系，Z=0为地面。它们不是机器人 `waist_yaw_link` 下的绝对目标，不能直接传入真机IK。

## 第三步 打开GUI

在终端A运行：

```bash
cd /home/source/LeTools-Learning
bash tools/kuavo_weighing.sh gui
```

窗口默认使用 `weighing_overview` 固定相机，可同时看到机器人和桌面。

俯视称重台：

```bash
bash tools/kuavo_weighing.sh gui --camera weighing_top
```

使用原有可自由调整视角的相机：

```bash
bash tools/kuavo_weighing.sh gui --camera ''
```

关闭窗口或在终端A按Ctrl+C退出。只需保留一个同状态目录的查看器即可。

## 第四步 在另一终端控制手臂

终端B执行以下命令，不要关闭终端A：

```bash
cd /home/source/LeTools-Learning

# 首次测试恢复模型初始状态，再进入准备姿态
bash tools/kuavo_weighing.sh ctl --reset-sim \
  pose ready --duration-s 2 --execute

# 查看当前仿真关节状态
bash tools/kuavo_weighing.sh ctl state

# 只计算左手前移2 cm对应的14维角度
bash tools/kuavo_weighing.sh ctl move \
  --arm left --direction forward --step-m 0.02 --duration-s 2

# 在仿真中执行同样的移动，GUI将播放动作
bash tools/kuavo_weighing.sh ctl move \
  --arm left --direction forward --step-m 0.02 --duration-s 2 --execute

# 再上移2 cm
bash tools/kuavo_weighing.sh ctl move \
  --arm left --direction up --step-m 0.02 --duration-s 2 --execute
```

`ctl` 已固定指定 `--backend mujoco`、称重场景和独立状态目录，不会发送真机消息。`--execute` 在这里的含义是提交本地仿真目标。

支持 `left/right` 两只手及 `forward/backward/left/right/up/down` 六个方向。单步为 `(0, 0.05]` m。从 `ready` 开始测试；双臂伸直的零位可能无法稳定求解某些保持手掌姿态的位移。

直接传入14维目标的规划示例：

```bash
bash tools/kuavo_weighing.sh ctl joints \
  --values 20 0 0 -30 0 0 0 20 0 0 -30 0 0 0 \
  --duration-s 2
```

添加 `--execute` 可在仿真中执行。数组顺序是左臂J1至J7、右臂J1至J7，单位为度。

## 第五步 观察连续状态

GUI与控制器共享：

```text
outputs/robot_control/kuavo5_weighing/latest_state.json
outputs/robot_control/kuavo5_weighing/history.jsonl
```

控制器从最新仿真关节状态计算，完成后记录目标和末端误差；GUI根据新追加的 `executed` 记录按动作时长播放。命令行计算可能先结束，窗口随后完成插值动画。

`planned` 或 `rejected` 记录不会让模型移动。GUI打开时显示保存的最新状态，只播放打开之后的新动作；重新打开可以对齐当前状态。一次只从一个控制终端发动作，避免多进程覆盖同一状态文件。

当前状态文件只恢复机器人14维关节。桌面物体的位置取自场景初始配置；GUI回放不会驱动物体接触动力学，也不会刷新电子秤数字。

如果控制命令输出 `executed` 但窗口没有变化，关闭旧窗口并重新运行 `bash tools/kuavo_weighing.sh gui`，以加载新版同步代码。窗口左上角显示 `MOVING/IDLE`、动作名称、状态目录和左右臂角度；GUI终端收到动作时打印 `playing`，完成时打印 `finished`。空闲时窗口还会核对 `latest_state.json`，补上遗漏的状态更新。

前移2 cm在全身视角下变化较小。需要明显的姿态验证时，可执行 `ctl pose elbows_90_palms_down --arm both --duration-s 4 --execute`，观察双肘弯曲，再执行 `ctl pose ready --arm both --duration-s 4 --execute` 返回准备姿态。

## 第六步 验证称重

执行独立秤台物理测试：

```bash
bash tools/kuavo_weighing.sh scale-test
```

测试使用与完整场景相同的桌面、秤盘和物体配置，逐个将物体初始化到秤盘上方3 mm，让其在重力下接触、稳定。它不依赖机器人平衡控制，也不宣称由机械臂完成放置。

秤盘使用 `scale_load_site` 上的三维力传感器 `scale_force`，测量支撑力，单位N。算法先测空盘去皮，然后对每种物体运行2秒动力学，取最后0.5秒均值：

```text
mass_kg = abs(mean(force_z) - tare_force_z) / abs(gravity_z)
```

结果保存在：

```text
outputs/kuavo_sim/weighing/scale_validation.json
```

当前验证结果：

| 用例 | 实测仿真质量 |
|---|---:|
| A | 约0.200 kg |
| B | 约0.500 kg |
| C | 约1.000 kg |
| 空盘 | 0 kg |

每项要求质量误差和末段标准差均小于0.005 kg，当前4项通过。该模型是理想秤台，尚未模拟实际电子秤的噪声、通信协议和滤波延迟。

## 第七步 改变初始摆放

要在完整GUI场景中看到0.5 kg物体位于秤盘上，先关闭GUI，执行：

```bash
bash tools/kuavo_weighing.sh build --on-scale b
bash tools/kuavo_weighing.sh gui
```

可选择 `empty`、`a`、`b`、`c`。此操作修改初始场景，便于观察布局；不是机械臂搬运演示。`scale-test` 始终独立测试全部四种情况。

恢复空盘：

```bash
bash tools/kuavo_weighing.sh build --on-scale empty
```

重新生成场景不会清空手臂历史。若需要同时重置手臂，再执行 `ctl --reset-sim pose ready --execute`。

## 无GUI时生成预览

```bash
bash tools/kuavo_weighing.sh render
```

输出：

```text
outputs/kuavo_sim/weighing/weighing_overview.png
outputs/kuavo_sim/weighing/weighing_top.png
```

通过VS Code或Windows文件管理器打开即可。生成称重场景中的标准姿态视频也可沿用旧工具：

```bash
bash tools/run_kuavo_mujoco_demo.sh \
  --scene outputs/kuavo_sim/weighing/scene.xml \
  --output outputs/kuavo_sim/weighing/arm_demo.mp4
```

该视频工具输出姿态回放，不检查桌面碰撞，也不产生称重读数。

## 获取仿真摄像头画面

场景支持机器人头部、左右腕部、腰部以及桌面俯视、外部全景相机。生成器新增的头部和腕部相机采用近似安装位置及60°垂直FOV，尚未对应真机标定数据。

已有旧场景时先重新生成，保持物体在秤上可指定 `--on-scale b` 等选项：

```bash
bash tools/kuavo_weighing.sh build
bash tools/kuavo_weighing.sh camera --list
```

实时查看头部图像：

```bash
bash tools/kuavo_weighing.sh camera --camera head --live
```

同时查看头部、左腕和右腕：

```bash
bash tools/kuavo_weighing.sh camera --camera all --live
```

参数映射：

| 参数 | MJCF相机 |
|---|---|
| `head` | `sim_head` |
| `left` | `sim_wrist_l` |
| `right` | `sim_wrist_r` |
| `waist` | `waist_camera`，官方模型原有相机 |
| `top` | `weighing_top` |
| `overview` | `weighing_overview` |
| `all` | 并列显示或分别采集头部、左腕和右腕 |

窗口通过Tk和Pillow显示，不依赖OpenCV的GUI支持。保持相机窗口开启，在另一终端使用 `ctl` 发出仿真动作，相机跟随同一状态目录中的14维状态和新增执行记录。腕部图像会随手臂变化；手臂放下时可能看到地面或机器人本体。默认显示目标帧率为15 Hz，实际帧率受渲染速度影响。

按 `s` 保存当前RGB-D，按 `q` 或Esc关闭窗口。采集进程与机器人GUI分别播放历史，画面时间存在轮询及渲染延迟，不能认为它们严格同步。

直接保存三路快照：

```bash
bash tools/kuavo_weighing.sh camera --camera all
```

每路默认生成：

```text
outputs/kuavo_sim/weighing/captures/head/     # 或left、right
├── rgb.png             # RGB图像，848×480
├── depth_m.npy         # float32光学Z深度，单位米
├── depth_preview.png   # 伪彩色深度预览
└── camera.json         # 内参K、FOV、相机到世界的变换、当前关节状态
```

命令行快照会覆盖相同目录中的上一组文件；实时窗口按 `s` 保存到时间戳子目录。RGB和深度由同一次场景状态渲染，对齐到同一像素网格。

通过 `--output` 指定保存目录，例如：

```bash
bash tools/kuavo_weighing.sh camera --camera head \
  --output /mnt/e/Data/Picture/sim_weighing
```

Python读取接口：

```python
import os
os.environ["MUJOCO_GL"] = "egl"  # 必须放在首次import mujoco之前

from tools.kuavo_sim_camera import KuavoSimCamera

camera = KuavoSimCamera()
try:
    camera.refresh()
    rgb, depth_m, info = camera.capture("head")
    print(rgb.shape, rgb.dtype)          # (480,848,3), uint8, RGB
    print(depth_m.shape, depth_m.dtype)  # (480,848), float32, 米
    K = info["K"]
    world_from_camera = info["T_world_camera_optical"]
finally:
    camera.close()
```

循环运行 `refresh()` 和 `capture()` 即可将图像数组交给检测代码。`T_world_camera_optical` 将相机光学坐标（右、下、前）转换到MuJoCo世界坐标；腕部相机的变换随关节状态更新。背景像素可能返回远裁剪面深度，使用 `camera.json` 的 `far_m` 剔除；深度是沿光轴的Z值，不是到相机原点的欧氏距离。

这里直接使用MuJoCo渲染接口，尚未向ROS发布 `/cam_h/...` 等话题。真机的 `view_kuavo_cameras.py` 依赖ROS订阅，应使用上述仿真入口查看本场景。

## 当前能力与后续实现

| 工作 | 状态 |
|---|---|
| 同一Kuavo-5.2模型加载、称重桌面与物体布局 | 已实现并完成离屏渲染 |
| 14维与六方向移动在新场景中运行 | 已验证准备姿态和左手前移2 cm |
| 秤台去皮及0.2、0.5、1.0 kg接触测量 | 4项用例通过，含空载回零 |
| 机器人抓住、搬运、松开放物 | 尚未实现 |
| 全身平衡、负载动力学、碰撞规划 | 尚未实现 |
| GUI机器人动作与秤台接触读数同步 | 尚未实现 |

当前机器人与称重道具未启用相互接触碰撞；手臂回放直接设置关节状态。即使画面中手穿过桌面，也不能据此判断真实动作可执行。黄色hover标记仅定义任务参考点，尚未完成可达性和无碰撞路径验证。

后续开发应先核对秤盘和待取物体是否处于手臂可达范围，再统一世界坐标与机器人腰部坐标；随后补齐抓取执行器、接触与碰撞规划、全身控制和称重稳定判定。旧MuJoCo后端的 `position_m` 实际来自世界坐标，历史记录中继承的 `reference_frame=waist_yaw_link` 标签并未执行坐标转换，不能直接拿该绝对数值控制真机。

## 文件入口

- [称重场景统一入口](../tools/kuavo_weighing.sh)
- [场景生成器](../tools/prepare_kuavo5_weighing_scene.py)：桌面尺寸、秤位置、物体质量等参数。
- [预览及秤台物理测试](../tools/kuavo_weighing_scene.py)
- [原有仿真与坐标转换手册](kuavo5_simulation_and_cartesian_conversion_manual.md)

场景XML中的mesh路径由生成器根据当前目录写入绝对路径；迁移电脑或移动项目后重新执行 `build` 即可。
