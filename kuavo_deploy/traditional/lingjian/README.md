# Kuavo-5 零件分拣：传统视觉与规则控制

本目录先完成离线感知，不会连接或控制真机。目标是用头部相机定位桌面零件和料盒，再通过腕部相机、深度、手眼标定、IK 和任务状态机完成闭环分拣。

## 1. 数据现状

数据集默认位于 `/mnt/e/Data/Leju/Beijing-lingjian/lerobot`，包含 507 个 episode、145729 帧、10 Hz 三路 RGB 视频以及 16 维状态和动作。它只有任务级标签 `parts sorting Task`，没有零件框、分割掩码、相机内参、深度图或手眼标定结果。

视频与状态对照显示，这套数据中归一化夹爪值 `0` 对应张开、接近 `1` 对应夹紧/抓住零件。该方向与当前 SDK 消息源码中的文字注释存在冲突，所以真机控制前必须用低速、空载测试再次确认，控制层应提供可配置的 `open_value` 和 `close_value`，不能硬编码消息注释。

画面中的黄色塑料件、蓝色料盒与桌面颜色差异明显，可以先用 HSV 和轮廓实现基线。灰色金属件受反光、阴影和机械臂遮挡影响较大，建议补充实例分割标注或使用固定背景差分与模板特征。

## 2. 抽取低冗余图像

在仓库根目录执行：

```bash
python kuavo_deploy/traditional/lingjian/extract_lerobot_frames.py \
  --dataset-root /mnt/e/Data/Leju/Beijing-lingjian/lerobot \
  --output-dir /tmp/lingjian_samples \
  --count 200
```

输出 JPEG 和 `samples.jsonl`。清单保留 episode、帧号、时间戳、关节状态与示教动作，便于追溯失败样本。

需要建立标注集时，建议按 episode 抽取起始、中间和结束阶段，而不是只在全局时间线上均匀取样：

```bash
python kuavo_deploy/traditional/lingjian/extract_lerobot_frames.py \
  --dataset-root /mnt/e/Data/Leju/Beijing-lingjian/lerobot \
  --output-dir /tmp/lingjian_annotation \
  --per-episode 3 \
  --episode-step 2
```

## 3. 运行颜色与轮廓基线

```bash
python kuavo_deploy/traditional/lingjian/classical_detector.py \
  --input /tmp/lingjian_samples \
  --output-dir /tmp/lingjian_detection
```

参数位于 `config.yaml`。输出包括标注图、二值掩码和 `detections.jsonl`。检测结果只包含像素中心和图像平面方向，不能直接作为机器人坐标。

当前阈值已在跨全数据范围均匀抽取的 30 张头部图像上做过冒烟检查：每张都找到了左右两个料盒，12 张含桌面黄色候选。该结果还不是有人工真值的精度评测。夹爪已经抓住的黄色件也会被颜色规则检出，因此状态机只能在手臂回到观察位后选择下一个目标，或根据机械臂投影区域屏蔽夹爪附近候选。

### 回放完整 episode

下面的命令直接根据 LeRobot v3 元数据定位共享 MP4 中的 episode 时间段，生成带检测框的视频、逐帧 JSONL 和延迟汇总：

```bash
python kuavo_deploy/traditional/lingjian/replay_episode.py \
  --dataset-root /mnt/e/Data/Leju/Beijing-lingjian/lerobot \
  --episode 0 \
  --output-video /tmp/lingjian_episode_0.mp4
```

### ROS 实时旁路检测

实时节点只订阅头部相机并发布检测结果，不会创建任何机器人控制 publisher：

```bash
source /opt/ros/noetic/setup.bash
ROS_HOME=/tmp/ros-home ROS_LOG_DIR=/tmp/ros-log \
  python kuavo_deploy/traditional/lingjian/ros_live_detector.py
```

输出话题：

- `/lingjian/perception/annotated/compressed`：带检测框画面。
- `/lingjian/perception/detections`：JSON 检测结果和单帧延迟。

可以通过 `rqt_image_view` 查看标注画面，通过 `rostopic hz` 和 `rostopic echo` 检查实时频率与结果。

## 4. 标注与模式识别

按 episode 划分 70%/15%/15% 的训练、验证和测试集，避免同一段连续视频同时进入训练集和验证集。每隔 10 至 30 帧选一张，优先标注无遮挡、部分遮挡、强反光和抓取失败画面。

建议类别：

- `yellow_part`
- `metal_part`
- `left_bin`
- `right_bin`
- `gripper`

传统识别路线可使用轮廓的面积、长宽比、Hu 矩、HOG 和灰度直方图训练 SVM。若灰色零件在不同姿态下仍不稳定，可只将感知模块换成小型实例分割模型，后续坐标变换、IK、状态机和安全控制保持规则化。

## 5. 真机闭环阶段

1. 标定三路相机内参。
2. 标定头部相机到 `base_link`、腕部相机到末端执行器的外参。
3. 引入原始深度值，计算目标三维中心和抓取姿态。
4. 用头部相机选择目标，移动到预抓取位。
5. 用腕部相机将目标中心对齐夹爪中心。
6. 通过 `arm_ik()` 计算关节目标，以 `control_arm_joint_trajectory()` 执行轨迹。
7. 夹爪闭合后检查 `lejuClawState.state == GRABBED`。
8. 抬升、移动到对应料盒、张开夹爪、撤离，并用头部相机验证目标已离开工作区。

真机首次测试必须限制单臂、低速度、桌面上方安全高度和允许工作空间，并在目标置信度低、深度缺失、IK 失败、夹爪未抓取或相机数据超时时停止动作。

## 6. YOLO / 开放词汇检测实验

通用 COCO 权重不包含这两类工业零件。实测 `yolo26n.pt` 会把蓝框识别成
`suitcase`、把黄色件识别成 `apple/sports ball`，金属件基本漏检，因此不能直接部署。

YOLO-World 使用下面三个英文提示词时可以稳定定位蓝框，也能定位部分黄色件；金属件的
零样本召回仍然较低。它适合为人工标注生成候选框，不应把未经复核的结果直接用于抓取：

```text
yellow plastic industrial part
silver metal clamp
blue plastic storage bin
```

安装可选依赖：

```bash
pip install -r kuavo_deploy/traditional/lingjian/requirements_yolo.txt
```

使用现成 YOLO-World 权重回放一条 episode：

```bash
python kuavo_deploy/traditional/lingjian/yolo_replay_episode.py \
  --dataset-root /mnt/e/Data/Leju/Beijing-lingjian/lerobot \
  --episode 0 \
  --model /path/to/yolov8s-worldv2.pt \
  --prompts "yellow plastic industrial part" "silver metal clamp" "blue plastic storage bin" \
  --confidence 0.03 \
  --output-video /tmp/lingjian_yolo_world_ep0.mp4
```

使用自建模型时不要传 `--prompts`：

```bash
python kuavo_deploy/traditional/lingjian/yolo_replay_episode.py \
  --dataset-root /mnt/e/Data/Leju/Beijing-lingjian/lerobot \
  --episode 500 \
  --model outputs/lingjian_yolo/parts/weights/best.pt \
  --confidence 0.25 \
  --device 0 \
  --output-video /tmp/lingjian_custom_ep500.mp4
```

实时 ROS 旁路节点使用相同模型接口：

```bash
python kuavo_deploy/traditional/lingjian/yolo_ros_detector.py \
  --model outputs/lingjian_yolo/parts/weights/best.pt \
  --device 0 \
  --max-rate 10
```

建议先训练普通三类检测模型：`yellow_part`、`metal_part`、`bin`。左右料框外观相同，
应在检测后按图像或桌面坐标区分左右，不能将它们作为两个视觉类别。普通检测框的中心用于
候选选择，抓取角度暂由框内轮廓计算。若角度误差无法满足抓取，再将数据改为 OBB 四点标注，
使用 `yolo26n-obb.pt` 微调。

```bash
python kuavo_deploy/traditional/lingjian/train_yolo.py \
  --data /path/to/lingjian_yolo/dataset.yaml \
  --model yolo26n.pt \
  --epochs 100 \
  --image-size 640 \
  --device 0
```

训练、验证和测试必须按 episode 划分，不能随机拆分连续视频帧。第一轮建议标注
300 至 500 张低冗余头部图像，每类零件都覆盖平放、竖立、相互接触、局部遮挡、强反光和
已进入料框等状态。YOLO 输出仍是像素坐标；像素到 `base_link` 的转换必须另做相机内参、
外参和桌面平面标定，模型训练不能替代几何标定。

本机实测结果（RTX 5070 Laptop、848×480 输入缩放到 640）：YOLO-World 在 CPU 上模型
推理约 68 ms/帧，完整调用中位数约 61 ms；GPU 预热后的单帧模型推理中位数约 4.1 ms，
完整调用中位数约 6.3 ms。速度足够处理数据集的 10 Hz 视频，当前瓶颈是零样本识别精度，
而不是推理吞吐。首次 GPU 运行若 `torch.cuda.is_available()` 为假，应先修复 WSL/容器的
GPU 设备权限，不能静默退回 CPU 后仍按 GPU 延迟设计控制周期。
