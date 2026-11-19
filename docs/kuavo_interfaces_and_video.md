# Kuavo-5 数据接口、相机内参与视频格式分析

本文针对当前项目、Kuavo-5（平台配置名 `5`）和已经采集的真机数据。结论来自三部分：项目部署代码、乐聚开源 ROS 消息定义，以及
`E:\Data\Picture\multiview_20260921_200838` 中的五组真机 RGB-D 样本。

## 1. 结论

- 相机接口含有计算 FOV 所需的参数，但 FOV 通常不是单独发布的字段。ROS 的 `sensor_msgs/CameraInfo` 发布图像宽高以及内参矩阵 `K`、投影矩阵 `P`，可据此计算水平和垂直 FOV。
- 当前真机头部流为 848×480。已保存的内参给出有效针孔模型 FOV 约为水平 92.54°、垂直 61.22°；左右视角约 46.09°/46.45°，上/下视角约 31.51°/29.71°。
- RGB 通过 `sensor_msgs/CompressedImage` 逐帧发送，项目用 OpenCV 解码。它是 ROS 图像帧流，不是 MP4/H.264 视频文件。
- 真机深度样本是 `sensor_msgs/Image`、`16UC1`、848×480、`uint16`，当前设备数据按毫米解释。0 表示没有有效测距，不能当作距离 0 mm。
- `/sensors_data_raw` 包含关节、IMU、末端执行器和六维力数据。Kuavo-5 双臂是 `joint_q[13:27]`，左臂 7 维在前、右臂 7 维在后。
- 当前部署配置默认只把三路 RGB、关节和夹爪送入策略；没有启用深度观测。部署端虽有深度回调，但会把每帧深度独立归一化，不能保留稳定的米制尺度。
- MuJoCo 的相机参数和真机并不等价。模型 XML 中可视相机是 `fovy=80°`；ROS 射线深度接口实际使用约 90°×60°、64×36，并把 0–2.5 m 除以 2.5 后发布为 `32FC1`。

## 2. 当前项目使用的真机 ROS 接口

| 方向 | Topic | ROS 类型 | 项目中的用途 |
|---|---|---|---|
| 机器人→程序 | `/cam_h/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 头部 RGB |
| 机器人→程序 | `/cam_l/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 左腕 RGB |
| 机器人→程序 | `/cam_r/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | 右腕 RGB |
| 机器人→程序 | `/cam_h/depth/image_raw` | `sensor_msgs/Image` | 头部原始深度采集 |
| 机器人→程序 | `/cam_h/depth/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 部署端支持的压缩深度 |
| 机器人→程序 | `/cam_h/depth/camera_info` | `sensor_msgs/CameraInfo` | 内参、畸变和投影参数 |
| 机器人→程序 | `/sensors_data_raw` | `kuavo_msgs/sensorsData` | 关节、IMU、末端和力传感器 |
| 机器人→程序 | `/leju_claw_state` | `kuavo_msgs/lejuClawState` | 乐聚夹爪状态 |
| 机器人→程序 | `/dexhand/state` | `sensor_msgs/JointState` | 灵巧手状态（选配） |
| 机器人→程序 | `/gripper/state` | `sensor_msgs/JointState` | RQ2F85 状态（选配/仿真） |

配置中的 `frequency: 30/500` 用作本地 `deque` 的容量，不能证明设备实际正在以 30 Hz 或 500 Hz 发布。实时频率应以 `rostopic hz` 测量。

## 3. RGB 图像消息

`sensor_msgs/CompressedImage` 包含：

```text
Header header       # seq、采集时间 stamp、光学坐标系 frame_id
string format       # 原像素格式、jpeg/png 编码及压缩后的颜色格式
uint8[] data        # JPEG/PNG 字节流
```

项目部署代码进行以下转换：

```text
CompressedImage.data
  -> np.uint8 一维数组
  -> cv2.imdecode(..., IMREAD_COLOR)       # uint8, H×W×3, BGR
  -> cv2.cvtColor(BGR2RGB)                 # uint8, H×W×3, RGB
  -> cv2.resize(image, (848, 480))         # OpenCV 参数顺序为 (width,height)
  -> torchvision.to_tensor
  -> float32, 1×3×480×848, 数值 0..1
```

已经保存的 `rgb.jpg` 用 OpenCV 读回时是 `uint8 (480,848,3)`、BGR 顺序。训练数据或 PIL 通常使用 RGB，混用两者会造成红蓝通道互换。

`CompressedImage` 不是传统的视频封装：每条 ROS 消息是一张独立 JPEG/PNG 图像。配置期望 30 fps，但码率随场景和 JPEG 大小变化，应分别测量 `rostopic hz` 和 `rostopic bw`。

## 4. 深度图像消息

### 4.1 真机未压缩深度

`sensor_msgs/Image` 包含：

```text
Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step          # 每行字节数，可能大于 width × 单像素字节数
uint8[] data         # 长度通常为 step × height
```

五组真机样本的实际格式为：

```text
encoding: 16UC1
shape:    480 × 848
dtype:    uint16
单位:     mm（当前相机/采集约定）
frame:    cam_h_color_optical_frame
```

每帧未压缩数据量约为 `848 × 480 × 2 = 814080` 字节，30 fps 时仅深度载荷约 24.4 MB/s（未计 ROS/TCP 开销）。读取时必须根据 `is_bigendian` 和 `step` 解码，不能直接假定连续小端矩阵。

| 视角 | 有效像素 | 中位深度 | 95% 分位深度 | 最小/最大有效深度 |
|---|---:|---:|---:|---:|
| 正中向下 | 85.21% | 2083 mm | 4866 mm | 404/9751 mm |
| 正向偏航 | 85.52% | 1433 mm | 4245 mm | 425/8833 mm |
| 反向偏航 | 83.02% | 2089 mm | 3662 mm | 368/7182 mm |
| 较浅俯视 | 86.28% | 2333 mm | 5463 mm | 443/10020 mm |
| 较陡俯视 | 83.26% | 1469 mm | 3627 mm | 336/5184 mm |

0 值是无效像素。用于检测包裹时，应在目标掩膜内剔除 0，并用中位数或截尾中位数估计距离，避免单个飞点控制机械臂。

### 4.2 部署代码中的压缩深度

`compressedDepth` 仍使用 `sensor_msgs/CompressedImage`，但 `data` 在 PNG 前可能有 transport 头。项目会查找 PNG magic bytes，再用 `IMREAD_UNCHANGED` 解码。之后它：

1. 将深度裁剪到默认 `[0,1500]`；
2. 用当帧最小值和最大值归一化到 0–1；
3. 输出 `float32 1×H×W`。

该张量可以作为模型视觉输入，但每帧的相同数值不再代表相同物理距离。传统抓取和位置控制应使用原始 `uint16` 毫米深度，或固定比例换算成米。深度 resize 还应使用最近邻插值；当前部署实现使用 OpenCV 默认线性插值，会在物体边缘产生并不存在的中间深度。

## 5. CameraInfo 与 FOV

关键字段如下：

```text
height, width
distortion_model
D[]                 # 畸变系数
K[9]                # 原始图像内参矩阵
R[9]                # 立体校正旋转矩阵
P[12]               # 校正后投影矩阵
binning_x/y, roi    # 像素合并与裁剪区域
```

针孔内参定义为：

```text
K = [fx  0 cx]
    [ 0 fy cy]
    [ 0  0  1]
```

当前保存的真机参数：

```text
width  = 848
height = 480
fx = 405.5993347167969
fy = 405.545166015625
cx = 421.290283203125
cy = 248.60035705566406
D  = [0,0,0,0,0,0,0,0]
```

考虑主点偏离图像正中心时：

```text
left   = atan(cx / fx)
right  = atan((width - cx) / fx)
top    = atan(cy / fy)
bottom = atan((height - cy) / fy)

horizontal FOV = left + right   = 92.5400°
vertical FOV   = top + bottom   = 61.2170°
```

若近似认为主点居中，则得到水平 92.5412°、垂直 61.2338°。二者差异很小。这里使用的是 `/cam_h/depth/camera_info`；其 `frame_id` 是 `cam_h_color_optical_frame`，并且采集记录表明深度已经对齐到彩色光学坐标系。要严格确认 RGB 原始流的镜头参数，还应读取 `/cam_h/color/camera_info` 并比较 `K/P`。

`D` 全零说明这条流可能已经校正，或驱动将畸变参数置零；它不等于物理镜头绝对没有畸变。对于 `image_rect` 应优先使用 `P`，对于未校正 `image_raw` 使用 `K+D` 做畸变处理。

分辨率等比例缩放且没有裁剪时，FOV 不变，`fx,fy,cx,cy` 按相同比例缩放。裁剪、letterbox 或非等比拉伸后必须同步更新内参。

## 6. 从像素和深度恢复相机坐标

ROS 光学坐标系约定为 `+X` 向图像右、`+Y` 向下、`+Z` 向前。像素 `(u,v)` 的有效深度为 `d_mm` 时：

```python
Z = d_mm / 1000.0
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
point_camera = [X, Y, Z]  # metres
```

这只得到相机坐标。要控制手臂到目标上方，还必须取得同一时刻的 TF 外参，例如
`base_link <- cam_h_color_optical_frame`，再做：

```text
p_base = T_base_camera × p_camera
```

随后才可生成末端期望位姿并调用 IK。只有 FOV 和深度而没有相机外参，无法安全地把图像坐标直接变成双臂 14 维关节角。

## 7. 机器人状态消息

`kuavo_msgs/sensorsData` 的结构为：

```text
std_msgs/Header header
time sensor_time
jointData joint_data
imuData imu_data
endEffectorData end_effector_data
FTsensorData FTsensor_data
```

### 7.1 joint_data

```text
float64[] joint_q       # 位置；当前项目按弧度使用
float64[] joint_v       # 速度
float64[] joint_vd      # 加速度
float64[] joint_torque  # 力矩
```

Kuavo-5 v52 配置的总关节数是 29，项目平台映射为：

```text
joint_q[0:12]   双腿，共 12
joint_q[12]     腰，共 1
joint_q[13:20]  左臂，共 7
joint_q[20:27]  右臂，共 7
joint_q[27:29]  头，共 2
```

部署代码取 `joint_q[13:27]` 作为双臂 14 维状态。消息定义本身使用可变长数组，因此程序应先检查 `len(joint_q)`，不能只凭类型假定一定为 29。`joint_v/joint_vd/joint_torque` 的物理单位没有写进 `.msg` 文件；按控制器常规应分别核验为 rad/s、rad/s²、N·m，使用前以当前固件文档或静止实验确认。

### 7.2 imu_data

```text
Vector3 gyro
Vector3 acc
Vector3 free_acc
Quaternion quat       # ROS 字段顺序 x,y,z,w
```

MuJoCo 发布端把内部 `w,x,y,z` 转成 ROS 的 `x,y,z,w`。真机使用时仍应确认 IMU frame_id、轴方向和单位，不能把 IMU 坐标系直接等同于 `base_link`。

### 7.3 FTsensor_data

```text
float64[] Fx, Fy, Fz
float64[] Mx, My, Mz
```

仿真发布端每个数组按 `[left, right]` 填两个值。消息定义没有固定数组长度，也没有写明单位或测量坐标系；一般力/矩使用 N 和 N·m，但真机代码在用于接触阈值前必须实测并确认符号方向。

### 7.4 lejuClawState

```text
Header header
int8[] state                 # 索引 0 左、1 右
endEffectorData data         # name/position/velocity/effort
bool is_high_freq
```

状态值：`-1 error`、`0 unknown`、`1 moving`、`2 reached`、`3 grabbed`。项目将 `data.position` 除以 100，得到 0–1 的策略状态；设备命令侧约定 0 为闭合、100 为打开。

## 8. 时间戳与对齐

五组已保存 RGB/深度的时间戳差均约 20 ms。这说明两帧接近，但不是严格同步。快速运动时，20 ms 足以造成目标边缘错位。

当前部署配置 `frame_alignment: false`，因此各 topic 都取本地缓存中的最新值，时间戳不保证一致。若开启对齐，环境会以 `1 / ros_rate` 作为容差；当前 `ros_rate=10`，即 100 ms。几何抓取建议使用 `message_filters.ApproximateTimeSynchronizer`，把容差设为能够覆盖设备实际偏差的最小值，并记录最终差值。

还应区分：

- `header.stamp`：传感器帧采集时间，是多传感器对齐首选；
- 消息到达 WSL 的本地时间：包含网络抖动，不能代替采集时间；
- `sensorsData.sensor_time`：控制器/仿真内部时间，时钟域可能与相机不同。

## 9. MuJoCo 相机接口与真机差异

Kuavo-5 s52 XML 中的 `waist_camera` 写有 `fovy="80"`。若渲染为 848×480，对应水平 FOV 约 111.99°；若为 640×480，约 96.42°。

但是 ROS 射线深度发布器没有直接沿用这个 `fovy`。其代码常量为：

```text
topic:       /waist_camera/depth/image_raw
frame_id:    waist_camera_link
resolution:  64 × 36
encoding:    32FC1
focal:       2.12
aperture:    4.24 × 2.448
range:       0.17..2.5 m
FOV:         90.0° × 60.0°
```

发布值是 `z_depth / 2.5`，范围约 0–1，之后还经过 3×3 Gaussian blur。源码没有为这条流发布 `CameraInfo`。因此：

- 仿真 `32FC1=0.4` 表示约 1.0 m，而不表示 0.4 m；
- 真机 `16UC1=1000` 表示约 1.0 m；
- XML 渲染画面和射线深度图的 FOV 可能不同；
- 若要用仿真验证真机视觉几何，需统一分辨率、FOV、相机位姿、深度单位、噪声和无效像素模型。

## 10. 当前代码中需要修正或明确的点

1. `configs/deploy/total/deploy_total.yaml` 把 `image_size` 写成 `[848,480]`，OpenCV 按 `(width,height)` 使用；`config.py` 的校验提示却称其为 `[height,width]`。应统一命名为 `resize_wh`，避免后续误配。
2. `ObsBuffer` 的 Gym 图像空间把同一个数组解释成 `(height,width)`，与 OpenCV 的解释相反，可能产生声明形状与实际张量形状不一致。
3. 深度 resize 应使用 `cv2.INTER_NEAREST`。
4. 几何控制应保留米制深度；不要使用逐帧 min-max 后的模型深度张量。
5. 当前总配置未订阅任何 depth key。若 LingBot-VLA-V2 训练数据没有深度，推理时临时加入深度也不会被模型自动利用。
6. 相机内参与外参应随每条数据记录保存；只存图像不足以复现三维定位。

## 11. 只读检查命令

在机器人 ROS 环境已配置后，可执行：

```bash
rostopic type /cam_h/color/image_raw/compressed
rostopic echo -n 1 /cam_h/color/image_raw/compressed/format
rostopic type /cam_h/depth/image_raw
rostopic echo -n 1 /cam_h/depth/camera_info
rostopic hz /cam_h/color/image_raw/compressed
rostopic hz /cam_h/depth/image_raw
rostopic bw /cam_h/color/image_raw/compressed
rostopic echo -n 1 /sensors_data_raw
rosmsg show kuavo_msgs/sensorsData
rosmsg show kuavo_msgs/jointData
```

查看画面：

```bash
rqt_image_view /cam_h/color/image_raw/compressed
rqt_image_view /cam_h/depth/image_raw
```

改进后的采集脚本会把完整 `CameraInfo`、FOV、图像格式、时间差和深度统计写入 `camera.json`：

```bash
/usr/bin/python3 tools/capture_kuavo_rgbd.py \
  --camera cam_h \
  --output /mnt/e/Data/Picture/test_rgbd
```

这里显式使用系统 Python，因为当前 Conda `base` 的 `python3` 没有 ROS/OpenCV/NumPy 依赖。该脚本只订阅和保存数据，不发布机器人控制命令。
