# MoveJ 多点序列示例

## 概述

`movej_sequence` 是一个示例程序，演示如何发送多个 MoveJ 点位让机械臂按顺序运动。

## 编译

```bash
cd ~/Documents/GitHub
source /opt/ros/humble/setup.bash
colcon build --packages-select kungshu_msgs kungshu_hardware_arm
source install/setup.bash
```

## 运行

### 1. 启动机械臂节点

```bash
# 终端 1：启动 arm_node
ros2 launch kungshu_hardware_arm arm.launch.py
```

### 2. 运行序列示例

```bash
# 终端 2：运行 MoveJ 序列
ros2 run kungshu_hardware_arm movej_sequence
```

## 代码结构

```
MoveJSequence 类
├── enable()           # 使能机器人
├── disable()          # 禁用机器人
├── moveJ()            # 单臂运动 (7轴)
├── moveJBothArms()    # 双臂运动 (14轴)
└── waitSeconds()      # 等待指定时间
```

## API 说明

### moveJ(pos, vel, acc)

单臂运动指令。

| 参数 | 类型 | 说明 |
|------|------|------|
| `pos` | `vector<double>` | 目标位置，7个关节角度 (rad) |
| `vel` | `double` | 最大速度 (rad/s)，默认 0.3 |
| `acc` | `double` | 最大加速度 (rad/s²)，默认 1.0 |

```cpp
// 示例：左臂移动到预备位置
node->moveJ({0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0}, 0.3, 1.0);
//          j1   j2   j3   j4   j5   j6   j7
```

### moveJBothArms(left_pos, right_pos, vel, acc)

双臂运动指令。

| 参数 | 类型 | 说明 |
|------|------|------|
| `left_pos` | `vector<double>` | 左臂目标位置 (7个关节) |
| `right_pos` | `vector<double>` | 右臂目标位置 (7个关节) |
| `vel` | `double` | 最大速度 (rad/s)，默认 0.3 |
| `acc` | `double` | 最大加速度 (rad/s²)，默认 1.0 |

```cpp
// 示例：双臂运动
node->moveJBothArms(
  {0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0},   // 左臂
  {0.0,  0.5, 0.0,  1.0, 0.0, -0.5, 0.0}   // 右臂
);
```

## 示例序列说明

默认示例包含 6 个点位：

| 序号 | 名称 | 左臂位置 | 右臂位置 | 等待时间 |
|------|------|----------|----------|----------|
| 1 | Home | 全零 | 全零 | 3秒 |
| 2 | Ready | 预备姿态 | 预备姿态(镜像) | 3秒 |
| 3 | Left Extended | 展开 | 归零 | 3秒 |
| 4 | Right Extended | 归零 | 展开 | 3秒 |
| 5 | Ready | 预备姿态 | 预备姿态 | 3秒 |
| 6 | Home | 全零 | 全零 | 3秒 |

## 自定义序列

修改 [src/movej_sequence.cpp](src/movej_sequence.cpp) 中的 `main` 函数：

```cpp
// 单臂模式（只用左臂）
node->moveJ({0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0});
node->waitSeconds(3.0);

node->moveJ({0.3, -0.3, 0.0, -0.8, 0.0, 0.3, 0.0});
node->waitSeconds(3.0);

// 双臂模式
node->moveJBothArms(
  {0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0},   // 左臂
  {0.0,  0.5, 0.0,  1.0, 0.0, -0.5, 0.0}   // 右臂
);
node->waitSeconds(3.0);
```

## 关节编号

```
左臂: J1-J7 (索引 0-6)
右臂: J1-J7 (索引 7-13)

       J1 (肩)
        │
    J2──┼──J3
        │
       J4 (肘)
        │
    J5──┼──J6
        │
       J7 (腕)
```

## 注意事项

1. **安全第一**：首次运行时降低速度 (`vel=0.1`)，确认运动轨迹正确后再提速
2. **使能状态**：确保 `enable()` 成功后再发送运动指令
3. **等待时间**：根据实际运动时间调整 `waitSeconds()`，避免点位切换过快
4. **关节限位**：确保目标位置在关节限位范围内

## 调试

```bash
# 查看当前关节状态
ros2 topic echo /states

# 手动发送单个 MoveJ 指令
ros2 topic pub --once /move_j_command kungshu_msgs/msg/MoveJCommand "
pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vel: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
acc: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
```
