# Kuavo-5 标准姿态与增量移动

入口脚本为 `tools/kuavo_arm_controller.py`。默认只规划；只有显式加入
`--execute` 或在 Python 中传入 `execute=True` 才会发布机械臂命令。

## 标准姿态

14 维顺序固定为：

```text
[左臂 J1..J7, 右臂 J1..J7]
```

预置姿态：

| 名称 | 含义 |
|---|---|
| `arms_down` | 双臂完全放下，14 维零位 |
| `ready` | 双臂自然前摆并略微屈肘 |
| `arms_horizontal_palms_down` | 双臂向两侧平举、手心向下 |
| `elbows_90_palms_down` | 双臂向前、肘部约 90°、手心向下 |

查看所有数组：

```bash
python3 tools/kuavo_arm_controller.py list-poses
```

## 任意 14 维目标

14 个值使用角度制，顺序为左臂 J1..J7、右臂 J1..J7。默认只规划：

```bash
python3 tools/kuavo_arm_controller.py joints \
  --values 0 15 70 -70 90 0 -20 0 -15 -70 -70 -90 0 20 \
  --duration-s 4
```

确认计划后增加 `--execute` 才会发布到真机或所选仿真后端。

只规划：

```bash
python3 tools/kuavo_arm_controller.py pose arms_down
```

实际执行：

```bash
python3 tools/kuavo_arm_controller.py pose arms_down --duration-s 4 --execute
```

标准姿态可以同时控制双臂，也可以只控制一侧。只控制一侧时，另一侧使用执行
前刚读取的实际关节反馈保持不动：

```bash
# 仅左臂进入屈肘 90° 姿态，右臂保持当前状态
python3 tools/kuavo_arm_controller.py pose elbows_90_palms_down \
  --arm left --duration-s 4 --execute

# 仅右臂完全放下，左臂保持当前状态
python3 tools/kuavo_arm_controller.py pose arms_down \
  --arm right --duration-s 4 --execute
```

Python 调用：

```python
from tools.kuavo_arm_controller import KuavoArmController

arm = KuavoArmController(robot="5")
arm.arms_down(execute=True)
arm.ready(execute=True)
arm.arms_horizontal_palms_down(execute=True)
arm.elbows_90_palms_down(execute=True)

# 单臂调用；未选择的一侧保持实时姿态
arm.left_arm_to_pose("elbows_90_palms_down", execute=True)
arm.right_arm_to_pose("arms_down", execute=True)

# 同一组快捷函数也可指定 arm
arm.arms_horizontal_palms_down(arm="left", execute=True)
```

实际持续时间不会短于命令参数。程序根据“最大关节变化 / 25°/s”自动延长
大幅度动作，降低跳变速度。

## 基于当前状态移动

腰部坐标系约定：`+X` 前、`+Y` 左、`+Z` 上。

```bash
python3 tools/kuavo_arm_controller.py move \
  --arm left --direction forward --step-m 0.02
```

上述命令只调用 FK/IK 并输出计划。实际执行需要：

```bash
python3 tools/kuavo_arm_controller.py move \
  --arm left --direction forward --step-m 0.02 \
  --duration-s 1.5 --execute
```

`direction` 支持：`forward`、`backward`、`left`、`right`、`up`、`down`。
单步范围限制在 `(0, 0.05] m`。

每次增量移动遵循：

1. 从 `/sensors_data_raw` 读取最新 14 维关节反馈；
2. 调用 FK 得到当前双手位姿；
3. 给选定手的当前位置增加笛卡尔位移，姿态保持不变；
4. 另一只手保持当前位姿；
5. 用当前 14 维反馈作为 IK 初值；
6. 检查 IK 实际位置误差；
7. 执行后再次读取关节反馈和 FK 位姿并保存；
8. 下一条命令重新从新鲜反馈开始，不把上一条目标值当作实际状态。

如果无法取得实时反馈，程序直接失败，不使用磁盘中的旧状态继续求解。

## 状态记录

默认目录：

```text
outputs/robot_control/kuavo5_arm/
├── latest_state.json
└── history.jsonl
```

- `latest_state.json`：原子更新的最新实际反馈，包含 14 维弧度、角度和双手 FK 位姿。
- `history.jsonl`：追加记录每次规划或执行，包括执行前状态、目标、IK 误差、执行后状态和目标误差。

可指定其它位置：

```bash
python3 tools/kuavo_arm_controller.py \
  --state-dir /path/to/state state
```

旧的 `tools/kuavo_cartesian_step_to_arm14.py` 已改为相同状态控制器的兼容入口，
继续使用它也会执行相同的反馈读取和记录流程。

## Kuavo-5 MuJoCo 验证后端

`kuavo_arm_controller.py` 默认仍使用真机 ROS。加入 `--backend mujoco` 后，
同一个 `move_relative()` 流程会改用乐聚官方 Kuavo-5.2 MJCF：

- 当前状态来自 MuJoCo 当前 `qpos`；
- FK 使用模型中的 `zarm_l7_link`、`zarm_r7_link` 和 0.193 m 手部偏移；
- IK 使用保持末端方向的 6D 阻尼最小二乘；
- 执行后的 MuJoCo 状态写入 `latest_state.json` 和 `history.jsonl`；
- 下一条命令以刚执行后的 `qpos` 为初值。

单条规划示例：

```bash
python tools/kuavo_arm_controller.py \
  --backend mujoco \
  move --arm left --direction forward --step-m 0.02
```

命令行仿真状态默认保存在独立目录
`outputs/robot_control/kuavo5_mujoco`。后续命令只会恢复 `source` 明确标记为
MuJoCo Kuavo-5.2 的状态，不会读取真机反馈文件：

```bash
# 从模型零位重新开始，并执行到 ready
python tools/kuavo_arm_controller.py \
  --backend mujoco --reset-sim \
  pose ready --execute

# 从上条命令执行后的 14 维状态继续
python tools/kuavo_arm_controller.py \
  --backend mujoco \
  move --arm left --direction forward --step-m 0.02 --execute
```

在同一仿真实例中从准备姿态连续移动：

```python
from tools.kuavo_mujoco_controller import MuJoCoKuavoArmController

sim = MuJoCoKuavoArmController()
sim.ready(execute=True)
sim.move_relative("left", "forward", step_m=0.02, execute=True)
sim.move_relative("left", "up", step_m=0.02, execute=True)
```

不要把双臂完全放下的零位作为所有方向测试的工作姿态。零位是伸直奇异位形，
某些保持手掌方向的移动没有稳定解；控制器会把这种 IK 结果标记为 `rejected`，
不会执行。批量测试会先进入 `ready` 姿态：

```bash
python tools/test_kuavo_cartesian_sim.py --step-m 0.02
python tools/test_kuavo_cartesian_sim.py \
  --step-m 0.05 \
  --output outputs/kuavo_sim/cartesian_conversion_test_50mm
```

当前验证结果：左右臂各执行六个方向，共 12/12 通过。20 mm 测试最大末端误差
为 0.0164 mm，50 mm 测试最大末端误差为 0.0185 mm；两组测试的相邻动作
状态连续性误差均为 0°。

## 一边输入命令一边查看 GUI

打开两个 WSL 终端，两个终端都进入项目目录。

终端 A 启动实时查看器：

```bash
cd /home/source/LeTools-Learning
tools/run_kuavo_live_viewer.sh
```

终端 B 输入控制命令：

```bash
cd /home/source/LeTools-Learning

# 首次使用：从模型零位进入 ready
tools/kuavo_simctl.sh --reset-sim pose ready --duration-s 2 --execute

tools/kuavo_simctl.sh move \
  --arm left --direction forward --step-m 0.05 --duration-s 2 --execute

tools/kuavo_simctl.sh move \
  --arm left --direction up --step-m 0.05 --duration-s 2 --execute

tools/kuavo_simctl.sh move \
  --arm right --direction left --step-m 0.03 --duration-s 2 --execute
```

查看器和控制器默认共享
`outputs/robot_control/kuavo5_mujoco`。控制器生成目标、执行并写入历史；GUI
监听新增历史记录并平滑播放。使用其它状态目录时，两端必须传入同一个路径：

```bash
tools/run_kuavo_live_viewer.sh --state-dir /tmp/my_kuavo_sim
tools/kuavo_simctl.sh --state-dir /tmp/my_kuavo_sim \
  --reset-sim pose ready --execute
```
