# Kuavo-5 相对桌面坐标称重归纳仿真

本地已实现四个已知桌角机器人坐标和头部 RGB 图像驱动的三件物品称重入框流程。使用官方 Kuavo-5.2 `biped_s52` 模型，双手在任务中保持向下，逐件重新观察，输出并记录左臂七关节和右臂七关节组成的 14 维关节角。

当前交付是带可见定位标记、固定厚度袋捆代理的运动学仿真。抓取和下落采用辅助附着，称重采用独立接触物理模拟后显示到 LCD，再从 RGB 识别读数。真实手指闭合、全身动力学和真机执行不属于已经通过验证的结果。

## 最新多条件验收

本轮的场景、运行结果与录像统一从[多条件验收入口](../outputs/kuavo_sim/acceptance_20261014/index.html)查看。页面仅把通过全部列出检查的运行标为通过，并区分完整流程、部分动作和未执行的拒绝案例；旧演示保留在下节，不代表本轮已完成的检查。

验收视频采用 H.264/YUV420 MP4，可通过本地验收 HTML 页面播放；编码不改变动作阶段或时间比例，原始输入 PNG 保持独立。

完整通过的验收录像在画面上标注 `2x simulation time`，即原始仿真时间的两倍速；全部动作阶段仍保留。左侧总览用于检查 CAD 姿态，关闭阴影以降低 CPU 渲染开销；头部 RGB 保留原有光照，右下角实际输入快照不改动。GUI `--replay --speed 1` 可按原始时间播放。

本轮交付16组条件录像，其中9组完整三件流程通过所列检查，每组63段动作；另有1个无抓手驱动的实际动力学对照。部分流程和拒绝案例的录像范围单独标注，不计作通过。[全部录像逐帧检查](../outputs/kuavo_sim/acceptance_20261014/delivery_artifact_check.json)和[逐项交付审查](../outputs/kuavo_sim/acceptance_20261014/completion_audit.json)提供机器可读证据。

人工调整、示教、四点确认及内侧参考点推算的方法见本文“手动调整场景”和“人工记录四个角点”。本轮运行输入的四角仍是明示的仿真假设值，没有现场人工触达记录。

[独立视觉测试](../outputs/kuavo_sim/acceptance_20261014/vision_validation_summary.json)包含 40 组未使用的随机种子试验，均落在 15 mm 定位门槛内，最大机器人坐标误差 13.18 mm。条件包括桌子平移/旋转、物品平移、视角变化、1 mm 标定噪声、1.5%小遮挡和声明的镜头畸变；对象仍是带定位标记桌面、固定尺寸代理。这些结果不表示已经训练 VLA 或能够识别任意软袋。

[真实动力学对照](../outputs/kuavo_sim/acceptance_20261014/deep_gap30/physics_no_grasp_rotor.mp4)采用浮动机器人、电机力矩和自由刚体包裹。五段机械臂动作跟踪后包裹未离开原位，说明 14 维两臂动作不包含所需的抓手闭合动作。主流程录像采用辅助抓取，只验证声明范围内的定位、运动规划、称重显示识别和释放沉降。固定准备姿态由仿真重置得到，尚未证明真机可从站立安全过渡至此。

## 历史演示结果

完整回放：[称重入框视频](../outputs/kuavo_sim/planar_rgb_weighing/replay.mp4)。使用本地 GUI 从第一阶段回放：

```bash
cd /home/source/LeTools-Learning
bash tools/kuavo_tabletop.sh gui \
  --scene outputs/kuavo_sim/planar_rgb_weighing/scene.xml \
  --state-dir outputs/robot_control/kuavo5_planar_rgb_weighing_demo \
  --replay
```

GUI 需要 WSLg 或可用的 X Server。`--speed 2` 可以加速观察。没有 `--replay` 时，GUI 显示当前状态，并等待该状态目录中新追加的动作记录。控制器和 GUI 必须使用相同的 `--scene` 和 `--state-dir`。

头部摄像头窗口：

```bash
bash tools/kuavo_tabletop.sh camera \
  --scene outputs/kuavo_sim/planar_rgb_weighing/scene.xml \
  --state-dir outputs/robot_control/kuavo5_planar_rgb_weighing_demo \
  --camera head --live
```

窗口中 `s` 保存 RGB 和深度，`q` 退出。任务定位使用 RGB；深度和真实相机矩阵没有传给定位算法。任务采集的头部图像、每件的识别结果和 LCD 图像保存在状态目录的 `observations/` 和 `scale_images/`。

## 运行一次新的实验

保留已经验证的演示状态，选择新的状态目录运行：

```bash
KUAVO_TEST_SCENE=outputs/kuavo_sim/planar_rgb_weighing/scene.xml
KUAVO_TEST_CORNERS=outputs/kuavo_sim/planar_rgb_weighing/assumed_manual_corners.json
KUAVO_TEST_STATE=outputs/robot_control/kuavo5_rgb_trial_01

bash tools/kuavo_tabletop.sh closed-loop \
  --scene "$KUAVO_TEST_SCENE" --corners "$KUAVO_TEST_CORNERS" \
  --state-dir "$KUAVO_TEST_STATE" --execute
```

另一个终端可以先启动 GUI，并传入同一个新状态目录。省略 `--execute` 会预检全部路线并保存 `task_preflight.json`，不会发布动作记录。仿真控制器直接更新模型状态，GUI 按记录的轨迹时间播放，因此规划计算与 GUI 播放可能有时间差。

加入相机滚转和 RGB 干扰：

```bash
bash tools/kuavo_tabletop.sh closed-loop \
  --scene "$KUAVO_TEST_SCENE" --corners "$KUAVO_TEST_CORNERS" \
  --state-dir outputs/robot_control/kuavo5_rgb_trial_noise \
  --execute --noise-level 2 --camera-roll-deg 10 --seed 337
```

`--noise-level 2` 增加亮度、伽马、颜色增益、噪声、模糊和 JPEG 干扰；它同时影响定位和秤读数图像。头部使用预设的多组偏航和俯仰角取景，算法只能接收到 RGB。

称重读取失败时，包裹留在秤上，已完成的动作和状态保留。可以用相同目录重试：

```bash
bash tools/kuavo_tabletop.sh closed-loop \
  --scene "$KUAVO_TEST_SCENE" --corners "$KUAVO_TEST_CORNERS" \
  --state-dir "$KUAVO_TEST_STATE" --execute --resume
```

恢复流程跳过已经入框的物品，从新图像确认秤上的物品位置，再执行称重和右臂入框阶段。记录仍处于持物状态时会要求人工恢复；不会自行假定物品已经松手。重做整轮实验应选择新的状态目录。

## 四点标定格式

桌面原点 A 是机器人视角的近左角，X 向右，Y 远离机器人，单位米。A、B、C、D 分别对应 `[0,0]`、`[1.2,0]`、`[1.2,0.6]`、`[0,0.6]`。当前低桌示例：

```json
{
  "reference_frame": "mujoco_world",
  "clearance_m": 0,
  "corners_robot": {
    "A": [0.22, 0.6, 0.76],
    "B": [0.22, -0.6, 0.76],
    "C": [0.82, -0.6, 0.76],
    "D": [0.82, 0.6, 0.76]
  }
}
```

这些数值是用户允许的“假设四个坐标已经知道”的仿真输入，不是伪造的人工触达记录。四个角全部可达并不是已经验证的事实，远角可能超出机械臂工作区。如果输入点在桌面上方同一高度，填写相应 `clearance_m`。拟合残差超过 5 mm，或尺寸与 120×60 cm 矩形明显不符，程序拒绝运行。

也可以指定 `reference_frame: "waist_yaw_link"`。脚本使用机器人模型的腰部变换，将这四个坐标转换为内部 MuJoCo 世界坐标；示例见 [腰部坐标输入](../outputs/kuavo_sim/planar_rgb_weighing/assumed_corners_waist.json)。不要将世界坐标数值标记成腰部坐标。真实使用时，标定点、运动学模型和实际工具点必须对应同一参考系。

桌子或机器人身体工作姿态改变后，应重新提供四点坐标。相机视角改变时，程序从每帧图像重算桌面映射，无需读取相机真实外参。

## 图像到关节角的计算

第一步从 RGB 中识别桌面参考标记，求像素到桌面米制坐标的单应矩阵。第二步在规整到桌面坐标的图像中识别白、绿、黑袋捆以及秤和框的位置。至少三张图像在 15 mm 范围内一致后才接受该件观测。

第三步用四点拟合的平面变换计算目标机器人 XYZ：`[1, table_x, table_y] × affine + normal × height`。第四步从当前 14 维关节状态求逆运动学，保持两只手的工作四元数固定。14 维数组是关节角，不是十四个空间坐标，也不是对四个角的关节角做线性插值。

求解采用阻尼雅可比、多组初值和带关节限位的优化。接近限位时，通过冗余自由度避让；运输直线不可行时，尝试相对桌面方向的有限绕行。每件的完整路线必须通过预检，再执行该件动作。终点要求 3 mm 位置误差和 1° 姿态误差，关节速度按分段时间限制到 25°/s。

框壁在规划模型中增加 8 mm 裕量。预检检查机械臂与桌面、秤、袋捆和框，机器人自身碰撞，以及所持包裹碰撞；关节插值的四分之一、二分之一和四分之三位置也要通过。图像误差可能使相邻袋捆出现小量假重叠，程序仅允许受限的已有接触垂直退出，不允许增加接触或横向穿越。

每次动作都会保存当前关节角、末端位置、固定姿态、物品流程状态和动作记录。下一件重新采集图像，并从当前模型关节状态开始求解。

## 场景和控制条件

主演示保持 120×60 cm 桌面、左侧袋捆、中央秤、右侧框，桌高暂定 0.76 m，近桌沿位置暂定 0.22 m。袋捆用 300×200×12 mm 的刚性代理表示；框外尺寸 400×300×230 mm、壁厚暂定 10 mm；秤尺寸 230×170×20 mm。

低桌实验采用固定准备姿态，骨盆世界位置约为 `[0.1182, 0, 0.7423]`。左右腿的髋俯仰、膝、踝俯仰约为 `[-49.53°, 96.50°, -46.96°]`，脚部相对原位置前移 30 mm。该准备姿态由仿真重置设置，手臂 14 维命令不包含腿部控制。脚底接触和静态重心条件经过检查，全身动力学、摩擦和执行器力矩没有因此得到验证。

手部控制点是在原局部 `z=-0.193 m` 手参考点基础上，沿向下掌法向偏移 45 mm 的代理 TCP。它用于这个固定手型仿真，不等于已经校准的真机手掌中心或真实夹持点。左手固定工作偏航为 -10°，右手为 +20°，任务内均保持向下。

主要高度参数相对桌面计算：

| 阶段 | TCP 高度 |
|---|---:|
| 左臂接近物品 | 120 mm |
| 袋捆接触代理 | 12 mm |
| 放到秤上 | 32 mm |
| 一般抬起和秤上方 | 140 mm |
| 右臂运输准备 | 260 mm |
| 框上方 | 285 mm |
| 框上释放 | 282 mm |

入框后的放置采用辅助下落和 12 mm 层高记录，填充物使用静态颗粒代理。当前不能用此结果证明软袋形变、真实指爪接触抓取或刚体落入填充物后的动力学。

## 可见标记与现场适用条件

实验使用 `DICT_4X4_50`，黑色标记边长 60 mm，包含白边的纸片为 75 mm。桌面 ID 0～3 位于距四角各 50 mm 的中心位置，ID 4、5 位于近边和远边的中间、距边 50 mm。打印和安装必须保持约定尺寸、位置和方向。

可直接使用 [A4 标记打印文件](../outputs/kuavo_sim/planar_rgb_weighing/markers_A4.pdf)，选择实际尺寸 100% 打印，并用尺核对 50 mm 校验线。重新生成：`/home/source/miniforge3/envs/letools/bin/python tools/export_kuavo_markers.py`。它生成向量黑色方格，第一页桌面标记，第二页器具标记。

如果标记被物品遮挡，可以移动到空区，并通过 `--marker-layout markers.json` 提供已知的印刷布置，例如 `{"table_centers_m":{"0":[0.05,0.05],"1":[1.15,0.05],"2":[1.15,0.55],"3":[0.05,0.55],"4":[0.4,0.05],"5":[0.4,0.55],"6":[0.7,0.55]}}`。这些数值是标记相对桌面的安装位置，不是物品或相机真值。新增标记可以用 `tools/export_kuavo_markers.py --table-ids 0 1 2 3 4 5 6` 生成。避免所有可见标记挤在同一条边上；框或手臂遮住参考点时，程序会继续取景或拒绝动作。

秤使用 ID 10、12，框使用 ID 11、13。它们模拟与器具刚性连接的延伸底座标记，位于桌面参考平面。秤标记相对中心为 `[±0.06,-0.14] m`，框标记为 `[±0.09,-0.25] m`。真实安装时必须随器具一起移动；将独立纸片贴在桌布上，不能自动得到移动后器具的新位置。现场是否允许这种标记，需要按比赛规则确认。

当前稳健指标来自带标记场景。不能将它解释为无标记、任意堆叠、任意物体旋转或任意包裹厚度的通用识别。实际秤的屏幕颜色、字体、反光和分辨率也需要重新验证；这里的 LCD 是已知七段字体的绿底显示。

## 测试结果与证据

| 测试 | 已观察到的结果 |
|---|---|
| 主流程 | 三件各 21 阶段，RGB 读数 371.5、829.3、64.8 g，全部入框 |
| 轨迹独立审查 | 4,405 采样，姿态最大约 0.244°，终点误差最大约 0.361 mm |
| 仿真真实几何评分 | 2,643 采样通过，穿入容差 2 mm |
| 静态支撑必要条件 | 含载荷与头部取景变化，最小重心投影余量约 21.97 mm |
| 常规扰动定位 | 100 组中 98 组全部目标在 15 mm 内，2 组拒绝，0 组超阈值错误接受 |
| 更强扰动定位 | 100 组中 87 组在 15 mm 内，13 组拒绝，0 组超阈值错误接受 |
| 随机 LCD 读数 | 50 组中 47 组正确，3 组拒绝，0 组错误读数，最多 10 帧 |
| 极端视角多帧定位 | 100 组全部目标在 15 mm 内，平均 4.79 帧；偏航 ±18°、俯仰 5～30°、滚转 ±30° |

轨迹终点的运动学误差不代表相机定位或真机落点达到亚毫米精度。定位扰动包括头部偏航 ±12°、俯仰 5～25°、光学滚转 ±20°、颜色和噪声、JPEG，另有桌面平移 ±20 mm、旋转 ±5°、物品偏移及四点标定误差 ±2 mm。桌面变化的每个测试都重新给定了对应的四点坐标。更强组还包含约 5% 图像遮挡和额外径向图像畸变。

极端单帧测试曾出现 16.3 mm 误差：接近正视桌面时，自动焦距估计不稳定，错误的高度修正放大了偏差。现在对这种图像跳过不可靠的相机高度推断，保留平面投影。原案例得到修正，正式演示图像结果不变；新随机种子的 200 组极端单帧中仍有一例约 15.3 mm。因此单帧成功不等于允许执行，运行流程坚持多帧一致性。以上 100 组极端多帧结果不能解释为对所有现场条件的保证。

读数测试随机设置不同质量并加入图像扰动，算法只能看到 RGB。另一完整实验改变桌面位置、旋转 3°、加入标定误差与相机滚转、图像噪声，并把私有物理载荷改成 591.7、1284.0、223.6 g；三件读数正确。该结果说明读数来自图像，不是按颜色返回固定质量。

另一次测试将袋捆中心的桌面 X 从 0.25 m 改回 0.19 m。原参考点被遮挡时，程序拒绝了动作；改用可见的三角参考布置，并按图像中的袋捆边界调整左臂停靠点后，三件全部完成。该轨迹通过 4,680 个运动学采样和 2,808 个真实几何采样，最大终点误差约 0.419 mm、姿态误差约 0.271°，静态重心余量约 22.1 mm。这说明停靠点和视觉参考可见性也会影响效果，不能只检查目标物体是否出现在图像中。

证据：[轨迹审查](../outputs/kuavo_sim/planar_rgb_weighing/final_trajectory_audit.json)、[真实几何评分](../outputs/kuavo_sim/planar_rgb_weighing/final_actual_contacts.json)、[静态支撑检查](../outputs/kuavo_sim/planar_rgb_weighing/final_stance_score.json)、[完整状态记录](../outputs/robot_control/kuavo5_planar_rgb_weighing_demo/closed_loop_result.json)。扰动与失败案例保存在 `outputs/kuavo_sim/optimization/`。

## 代码入口

| 文件 | 功能 |
|---|---|
| [kuavo_table_vision.py](../tools/kuavo_table_vision.py) | 纯 RGB 定位、四点平面拟合、多帧一致性、秤上物品重新识别 |
| [kuavo_planar_task.py](../tools/kuavo_planar_task.py) | 固定掌向下 IK、抓取点搜索、运输绕行、逐阶段状态记录 |
| [kuavo_planar_closed_loop.py](../tools/kuavo_planar_closed_loop.py) | 重采头部图像、逐件执行、读数失败后的恢复 |
| [kuavo_scale_vision.py](../tools/kuavo_scale_vision.py) | 七段字符注册识别和读数一致性 |
| [kuavo_sim_scale_environment.py](../tools/kuavo_sim_scale_environment.py) | 私有称重物理与 LCD 相机环境，输出给任务的是 RGB 识别结果 |
| [kuavo_planar_collision.py](../tools/kuavo_planar_collision.py) | 从四点和 RGB 重建碰撞障碍，不读取场景物体真值作定位 |
| [prepare_kuavo_fixed_crouch.py](../tools/prepare_kuavo_fixed_crouch.py) | 生成独立固定蹲姿实验，检查腿部限位和脚位 |
| [audit_kuavo_planar_task.py](../tools/audit_kuavo_planar_task.py) | 对已记录轨迹做独立运动学审查 |
| [score_kuavo_sim_contacts.py](../tools/score_kuavo_sim_contacts.py) | 仅用于隔离评分的真实场景几何检查 |

实际机器人摄像头和 14 维接口的已有说明见 [摄像头与双臂控制手册](kuavo5_camera_and_arm14_manual.md)。本流程全部运行在本地仿真，接入真实机器人前需要验证实际工具点、模型零位、手指控制和全身控制。

## 2026-10-14：多条件验收与人工示教

本次版本的验收入口是 `outputs/kuavo_sim/acceptance_20261014/index.html`。每组场景单独保存 XML、配置、四点输入、控制记录、原始 RGB、回放与审查结果。入口分别显示完成、停止、拒绝和检查结果；只出现 `executed` 不能代替物理审查。

主流程仍使用辅助抓取来测试视觉到关节角的转换。秤读数来自独立接触物理产生的 LCD 图像；入框后的释放使用刚体重力与接触沉降。填充物是静态椭球，已经放入的物品在后续沉降计算中固定，不包含软袋或颗粒动力学。另一段 `physics_no_grasp_rotor.mp4` 使用浮动机器人、电机力矩和自由刚体包裹，记录未加入抓手驱动时的失败结果。

新视频同时显示第三人称场景、记录头部角度下的仿真视图，以及控制器实际采用的原始 RGB。右下角图像是在对应路线规划或称重时采集的快照，不是实时视频。

### WSL 显示与 EGL 提示

`failed to open /dev/dri/renderD128: Permission denied` 表示当前进程无法访问该渲染设备；不能仅凭这行判断任务失败。现在控制器会打印阶段进度、异常和退出码。CPU 渲染可通过以下前缀选择：

```bash
KUAVO_SOFTWARE_RENDER=1 bash tools/kuavo_tabletop.sh camera \
  --scene outputs/kuavo_sim/acceptance_20261014/baseline28/scene.xml \
  --state-dir outputs/robot_control/acceptance_20261014_baseline28_v4 \
  --camera head --live
```

相机实时窗口只渲染 RGB，按 `s` 才采集深度。`ARB_clip_control unavailable` 是深度读取精度提示；主流程定位使用 RGB。GUI/Tk 窗口仍需要 WSLg 或 X Server。脚本也允许显式设置 `KUAVO_RENDER_BACKEND=glfw` 或 `MUJOCO_GL`；选择 `osmesa` 前须已有对应库。

### 手动调整场景

```bash
cd /home/source/LeTools-Learning
KUAVO_SOFTWARE_RENDER=1 bash tools/kuavo_tabletop.sh scene-editor \
  --source-scene outputs/kuavo_sim/acceptance_20261014/prepared_deep/scene.xml \
  --config outputs/kuavo_sim/acceptance_20261014/my_scene.json \
  --output outputs/kuavo_sim/acceptance_20261014/my_scene
```

在俯视图拖动物品；右侧调整桌子位置和偏航、机器人位置和偏航、头部观察角度、相机滚转/FOV、近侧桌布垂落长度。点击 `Save config and build scene` 保存配置并重建；点击 `Preview head RGB` 看这个布置的头部画面。机器人与桌子的间距同时受骨盆前移和脚的位置影响，不能只看桌子世界 X 坐标。具体距离见每组 `placement_report.json`。

场景编辑不向真机发送动作。改变机器人或桌子的相对位置后，重新提供标定坐标，并使用新的状态目录。已有 GUI 不会自动重新加载修改后的 XML，需要关闭后重新打开。

### 人工记录四个角点

A、B、C、D 依次是近左、近右、远右、远左，桌面 XY 分别为 `[0,0]`、`[1.2,0]`、`[1.2,0.6]`、`[0,0.6]`。角点坐标和工具点必须使用同一参考系，单位米。

方式一：在头部 RGB 上点选角落，再输入人工测量得到的机器人 XYZ：

```bash
bash tools/kuavo_tabletop.sh calibration \
  --scene outputs/kuavo_sim/acceptance_20261014/baseline28/scene.xml \
  --state-dir outputs/robot_control/my_manual_calibration \
  gui --image /absolute/path/to/head_rgb.png
```

选择角点，在图像上点击，输入 XYZ，再按 `Confirm and record point`。四点完成后按 `Fit and export four corners`。像素点击作为对应图像的标定证据保存；运行时桌面单应矩阵仍从可见定位标记重新求解。不能把输入 XYZ 的确认理解为程序已经验证了真实接触位置。

方式二：移动仿真手臂，由操作者观察 GUI，再记录当前 TCP：

```bash
# 使用同一个 SCENE 和 CAL_STATE，先打开 GUI；另一个终端示教。
SCENE=outputs/kuavo_sim/acceptance_20261014/baseline28/scene.xml
CAL_STATE=outputs/robot_control/my_manual_calibration

bash tools/kuavo_tabletop.sh planar --scene "$SCENE" --state-dir "$CAL_STATE" init

# 机器人世界坐标：X 前，Y 左，Z 上；一次最多 5 厘米。
bash tools/kuavo_tabletop.sh planar --scene "$SCENE" --state-dir "$CAL_STATE" \
  jog --arm left --delta 0.01 0 0 --execute

# 只有操作者判定到位后，才明确使用 --confirm。
bash tools/kuavo_tabletop.sh calibration --scene "$SCENE" --state-dir "$CAL_STATE" \
  record-tcp A --arm left --clearance-m 0.05 --confirm
```

按上述方式记录 B、C、D，选择对应手臂。四点使用同一离桌高度，例如全部为 5 厘米；这需要实际确认，而不是只看一张图猜高度。最后导出：

```bash
bash tools/kuavo_tabletop.sh calibration --scene "$SCENE" --state-dir "$CAL_STATE" \
  fit --output "$CAL_STATE/manual_corners.json"
```

记录包含 TCP、14 维角度、确认标记、时间和修改历史。修改任意点位后必须重新拟合并导出；运行器检查人工记录摘要，拒绝过期导出文件或来自其他场景的人工会话。远角不一定在固定站位的机械臂工作区内，不要把示例坐标当成四角均已触达的证明。手动 jog 用于本地示教，未标定时不具备完整的图像重建避障条件。

### 远角不可触达时：四个可达参考点

可以在桌面上量出四个内侧参考点的位置，分别用手臂确认并记录 TCP，然后按 120×60 厘米尺寸推算外角。示例参考点为 R1 `[0.20,0.08]`、R2 `[0.85,0.08]`、R3 `[0.85,0.35]`、R4 `[0.20,0.35]`；先检查这些点实际可达且没有物品挡住。

```bash
# 手臂已经在 R1 上方同一离桌高度，操作者确认后记录。
bash tools/kuavo_tabletop.sh calibration --scene "$SCENE" --state-dir "$CAL_STATE" \
  set-reference R1 --table-xy 0.20 0.08 --arm left --clearance-m 0.05 --confirm

# 依次示教并记录 R2、R3、R4，再拟合。
bash tools/kuavo_tabletop.sh calibration --scene "$SCENE" --state-dir "$CAL_STATE" \
  fit-references --output "$CAL_STATE/manual_corners.json"
```

也可以用 `set-reference ... --xyz X Y Z --confirm` 输入测量值。至少四点、不能共线，X/Y 分布分别至少 40/20 厘米；拟合残差超过 5 毫米时拒绝。导出的角点标明“由内部参考点推算”，并提供误差放大系数；拟合残差为零不能证明测量误差为零。

### 工具点与执行

默认 TCP 是仿真代理，不是已经校准的真机指尖。可以通过 `--tcp-offsets measured_tcp.json` 输入左右两只手相对 `zarm_l7_link`/`zarm_r7_link` 的实测局部 XYZ：

```json
{
  "left": [0.0, -0.045, -0.193],
  "right": [0.0, 0.045, -0.193]
}
```

以上仍是默认代理值的格式示例。实际应填写测量值；手掌法向仍按当前机器人模型约定。改变工具点后重新做桌面标定，并换状态目录。

```bash
KUAVO_SOFTWARE_RENDER=1 bash tools/kuavo_tabletop.sh closed-loop \
  --scene "$SCENE" \
  --corners "$CAL_STATE/manual_corners.json" \
  --state-dir outputs/robot_control/my_manual_trial_01 \
  --execute
```

省略 `--execute` 先做预检。任务会在每件前重新观察，运输后复核物品原位置是否腾空，放到秤上后再观察并从当前 14 维状态重新规划右臂路线。源位置腾空只是一项视觉证据，不能单独证明抓手已夹紧。

`--left-palm-yaw` 和 `--right-palm-yaw` 是每组试验开始时选定的偏航角；任务内保持固定，掌法向向下。规划被拒绝可能来自当前姿态、规划路径或碰撞约束，不能直接推断机器人在所有姿态下都无法到达该点。

### 记录与验收范围

每次运行有独立的 `runs/<attempt>.json` 和代码哈希；定位图像保存在 `observations/<attempt>/...`，称重图像保存在 `scale_images/<read>/...`，避免恢复运行覆盖旧证据。`latest_observation_files.json` 指向最新观测，动作记录的 `vision_observation_file` 指向当时的具体观测。

`private_bin_environment.json` 和 `private_bin_drops/` 是仿真环境与独立评分的物理真值，用于渲染和审查，不参与 RGB 定位或 IK 目标生成。独立审查还会不提供预期重量地重新解码归档 RGB，再与独立负载记录核对。独立审查分别检查 14 维状态连续性、固定准备姿态的全身模型关节范围、轨迹、实际场景几何、关节速度/加速度/jerk、电机限值与地面反力必要条件。后两项基于模型和假定摩擦，不能替代真机驱动器、抓手、软袋和全身控制器验证。

真实执行仍需确认工具点、实际包裹高度与抓取区域、抓手闭合/释放接口，以及可用的准备姿态。14 维数组只包含两臂关节；这次没有连接或控制真实机器人。

释放入框采用独立的刚性盒接触沉降：填充物是静态椭球，先前已沉降包裹在后续释放中固定；尚未模拟颗粒重排、软袋形变或多件包裹同时移动。每次释放的接触、包含范围和最终速度证据见 `private_bin_drops/`，不能将该代理解释为真实物料箱已完成抓放验证。

### 本轮发现的摆放与规划限制

紧贴摆放的三包在顺序交换实验中出现约 3.93 mm 的邻包穿透：视觉重建的接触关系和实际模型略有不同，辅助抓取把包裹置于估计位置后发生接触。该实验虽然留下63段完整动作记录，但独立检查失败，不能算作可用结果。建议源包和设备之间留出能覆盖定位、控制和模型误差的余量；本轮新增50 mm以上的源包间隙实验。它是测试范围内的设计余量，不是任意真实场景的安全保证。

物料箱外尺寸400×300 mm不等于内尺寸。当前模型壁厚10 mm，内尺寸约380×280 mm；300×200 mm刚性袋捆旋转90°后，300 mm长边不能直接放入280 mm方向。仅改变源包角度不能保证放置成功，必须同时考虑箱体方向或增加平面内转向/重抓流程。当前每组流程保持选定的手掌姿态，不以临时转腕绕过这一限制。

初始化的左手停车位会受到源包摆放影响。新代码在原停车位不可达时，依据人工桌面坐标和RGB障碍物试选有限个候选点，仍要求掌向和碰撞检查通过；这只是仿真重置的选择，没有验证真机姿态过渡。抓取点预检还利用IK误差在桌深方向上的分量安排接触点搜索，减少向错误方向重复尝试。

最后的间隙布局实验未通过完整路线预检：秤向桌深处移动会增加左手前伸需求；旋转秤虽然缩短左手前伸，但放置点移向另一侧后，当前右臂固定姿态的转移预检仍被拒绝。这些是本版本、指定姿态和搜索方式下的结果，不是所有姿态均不可达的证明。较大的摆放间隙解决不了工作区或手臂之间的协调问题，应一起检查源区、秤和箱的相对位置。

当前识别依赖可见桌面定位标记和设备基座标记，以及已知的物品尺寸/固定高度。未验证无标记桌面、任意真实软袋、真实LCD字体、动态移动桌子或真实手指接触。桌子/机器人相对位置改变后重新标定；这些假设应在现场验收前逐项确认。
