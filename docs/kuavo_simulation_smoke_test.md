# Kuavo 仿真验证记录

## 已完成

- 使用乐聚官方 `kuavo-ros-opensource` 的 Kuavo 5.2 MJCF 加载 29 自由度模型。
- 验证 14 维顺序为 `[左臂 J1..J7, 右臂 J1..J7]`，四个标准姿态均未超出模型关节限位。
- 生成 16.2 秒、30 FPS 的离屏仿真视频和四姿态关键帧。
- 加载官方 `opensource/kuavo-data-challenge` 分支的任务 1“传送带物品分拣”场景。
- 将任务 1 房间、桌面、传送带和物体与 Kuavo 5.2 官方模型组合，完成视觉与姿态回放。
- 增加 `configs/deploy/kuavo_sim_env.yaml`，补齐推理文档引用但当前仓库缺失的仿真配置。

这些结果验证模型加载、14 维映射、关节限位和画面输出。它们是运动学视觉验证，未验证 MPC/WBC 动力学、自动评分或 LingBot-VLA-V2 推理质量。

## 直接观察

生成默认 Kuavo 5.2 姿态视频：

```bash
tools/run_kuavo_mujoco_demo.sh
```

打开 WSLg 实时循环窗口：

```bash
tools/run_kuavo_mujoco_demo.sh --live
```

传入任意 14 维角度（单位：度）：

```bash
tools/run_kuavo_mujoco_demo.sh \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20
```

生成 Kuavo 5.2 零件分拣视觉场景并回放：

```bash
python tools/prepare_kuavo5_lingjian_scene.py
tools/run_kuavo_mujoco_demo.sh \
  --scene outputs/kuavo_sim/scenes/lingjian_task1_kuavo52_visual.xml \
  --output outputs/kuavo_sim/kuavo52_lingjian_pose_demo.mp4
```

## 完整推理闭环的边界

推理文档中的 `auto_test` 依赖另一侧同时提供以下 ROS 服务和话题：

- `/simulator/init`、`/simulator/reset`、`/simulator/start`、`/simulator/success`
- `/cam_h`、`/cam_l`、`/cam_r` 的 RGB/深度图像
- `/sensors_data_raw`、`/gripper/state`、`/kuavo_arm_traj`

这些接口来自官方 data-challenge 仿真分支的 Docker ROS 控制器。该分支当前提供的是 `biped_s45`（Kuavo 4.5）控制和评分场景。本文生成的 Kuavo 5.2 组合场景只替换了视觉模型；若要做 Kuavo-5 的完整自动评测，还需移植 launch、控制器关节索引、相机 topic、夹爪和评分脚本。

当前仓库没有训练完成的 LingBot-VLA-V2 checkpoint，因此不能运行真实策略推理。取得 checkpoint 后，`configs/deploy/kuavo_sim_env.yaml` 已按外部服务模式设置为 `policy_type: client`，启动顺序为：

1. 启动 ROS master 和 Kuavo-5 仿真控制/评分进程；
2. 启动 LingBot-VLA-V2 policy server；
3. 执行：

```bash
python kuavo_deploy/src/scripts/script_auto_test.py \
  --task auto_test \
  --config configs/deploy/kuavo_sim_env.yaml
```
