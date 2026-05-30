# Kungshu Hardware Arm 使用指南

## 1. 编译

```bash
cd ~/Documents/GitHub/kungshu_hardware_arm
colcon build --packages-select kungshu_hardware_arm
source install/setup.bash
```

## 2. 运行主节点

### 方式 A：直接运行（需要 root 权限访问网络）

```bash
# 设置网络权限
sudo setcap 'cap_net_raw,cap_net_admin+eip' install/kungshu_hardware_arm/lib/kungshu_hardware_arm/arm_node

# 运行（指定网口）
ros2 run kungshu_hardware_arm arm_node --ros-args -p left:=enx6c1ff71e2469 -p right:=enp114s0
```

### 方式 B：使用 launch 文件

```bash
ros2 launch kungshu_hardware_arm arm.launch.py
```

> **注意**：`launch/arm.launch.py` 中已硬编码网口名称，需根据实际网络接口修改。

## 3. 使能机器人

```bash
# 使能所有 14 个关节
ros2 service call /set_enable_service kungshu_msgs/srv/SetEnable "{enable: true}"

# 禁用
ros2 service call /set_enable_service kungshu_msgs/srv/SetEnable "{enable: false}"
```

## 4. 发送运动指令

### 关节空间运动 (MoveJ)

```bash
# 通过 topic 发送目标位置
ros2 topic pub --once /move_j_command kungshu_msgs/msg/MoveJCommand "
pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vel: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,   0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
acc: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"
```

### 力矩模式

```bash
# 切换到力矩模式 (CST)
ros2 service call /set_torque_enable kungshu_msgs/srv/SetTorqueEnable "{torque_enable: true}"

# 发送力矩指令
ros2 topic pub --once /tau_command kungshu_msgs/msg/ArmTorqueCommand "
tau: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

## 5. 监控状态

```bash
# 查看关节状态（位置、速度、力矩）
ros2 topic echo /states
```

## 6. 测试灵巧手

```bash
# 运行灵巧手 CAN 测试
ros2 run kungshu_hardware_arm hand_test
```

## 关键参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `left` | 网口名 | 左臂 EtherCAT 网口 |
| `right` | 网口名 | 右臂 EtherCAT 网口 |
| 控制周期 | 4ms | 250Hz |
| 关节顺序 | 0-6 左臂, 7-13 右臂 | 14 轴 |

## 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 权限不足 | 需要 root 权限访问网络设备 | 以 root 运行或设置 `cap_net_raw` 能力 |
| 找不到从站 | 网口名称错误或线缆未连接 | 检查网口名称，确认 EtherCAT 线缆连接 |
| 关节不动 | 未使能 | 先确认 `set_enable_service` 返回 `success: true` |

## ROS2 接口列表

### Topics

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `states` | `kungshu_msgs/msg/ArmState` | 发布 | 关节状态（位置、速度、力矩、负载、状态） |
| `move_j_command` | `kungshu_msgs/msg/MoveJCommand` | 订阅 | 关节空间运动指令（14轴位置/速度/加速度） |
| `tau_command` | `kungshu_msgs/msg/ArmTorqueCommand` | 订阅 | 力矩指令（14轴） |

### Services

| 名称 | 类型 | 说明 |
|------|------|------|
| `set_enable_service` | `kungshu_msgs/srv/SetEnable` | 使能/禁用所有关节 |
| `set_mode_service` | `kungshu_msgs/srv/SetModeOfOperation` | 设置操作模式 |
| `set_torque_enable` | `kungshu_msgs/srv/SetTorqueEnable` | 切换力矩模式(CST)或位置模式(CSP) |

### 操作模式

| 模式 | 编号 | 说明 |
|------|------|------|
| CSP | 8 | 循环同步位置模式（默认） |
| CSV | 9 | 循环同步速度模式 |
| CST | 10 | 循环同步力矩模式 |
