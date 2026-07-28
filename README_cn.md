# hex_ros_demo_arm_force_feedback
**中文** | [English](README.md)

## 目录

- [1. 包的简介](#1-包的简介)
- [2. 包架构](#2-包架构)
- [3. 话题接口](#3-话题接口)
- [4. 参数说明](#4-参数说明)
- [5. 依赖关系](#5-依赖关系)
- [6. 快速使用](#6-快速使用)

---

## 1. 包的简介

这是 **HEXFELLOW** Archer Y6 机械臂的**双边力反馈演示包**（bilateral force feedback demo），实现主从双臂的力觉临场感（telepresence）控制。

本包在力反馈控制循环中以 `__feedback` 为核心，完成以下功能：

- **主从跟随控制** — 每周期读取主/从臂的实时关节状态，通过 deadzone 补偿和 clip 饱和限制计算有效目标位置，分别发布 MIT 阻抗控制指令：从臂跟随主臂位置（`__build_follow_ctrl`，从端 PD 增益），主臂反馈从臂受力（`__build_feedback_ctrl`，主端 PD 增益）。
- **夹爪力反馈** — 1-DOF 夹爪沿用相同模式，支持主从夹爪的双边位置跟随与力反馈。
- **缓启动/缓退出** — 启动和退出时, 将双机械臂从当前位置平滑运动到稳定位置。
- **键盘控制** — 按 **`s`** 启动力反馈控制，按 **`q`** 停止并归位退出。

同时支持 **ROS 1** 和 **ROS 2**，提供四种启动场景：独立节点、仿真对仿真（sim2sim）、真机对仿真（real2sim）、真机对真机（real2real）。


---

## 2. 包架构

```
hex_ros_demo_arm_force_feedback/
├── config/                                    # 参数配置
│   ├── ros1/
│   │   └── arm_force_feedback.yaml            #   ROS 1 参数（force_feedback）
│   └── ros2/
│       └── arm_force_feedback.yaml            #   ROS 2 参数（force_feedback）
├── launch/                                    # ROS launch 启动文件
│   ├── ros1/
│   │   ├── arm_force_feedback.launch          #   单节点启动
│   │   └── sim2sim_force_feedback.launch      #   仿真对仿真完整启动
│   └── ros2/
│       ├── arm_force_feedback.launch.py       #   单节点启动
│       ├── real2real_force_feedback.launch.py #   真机对真机完整启动
│       ├── real2sim_force_feedback.launch.py  #   真机对仿真完整启动
│       └── sim2sim_force_feedback.launch.py   #   仿真对仿真完整启动
├── hex_ros_demo_arm_force_feedback/           # 核心代码
│   ├── arm_force_feedback.py                  #   主节点：双边力反馈控制循环
│   ├── TrajectoryController.py                #   轨迹规划器
│   └── utility/                               #   双层 ROS 接口抽象层
│       ├── __init__.py                        #     ROS 版本选择器（ROS_VERSION 环境变量）
│       ├── interface_base.py                  #     抽象基类（InterfaceBase）
│       ├── ros1_interface.py                  #     ROS 1 DataInterface
│       └── ros2_interface.py                  #     ROS 2 DataInterface
├── resource/                                  # ament 资源索引
├── setup.py                                   # Python 打包配置（ROS 2）
├── CMakeLists.txt                             # CMake 打包配置（ROS 1）
├── package.xml                                # ROS 包清单（双系统条件依赖）
├── README.md                                  # 英文文档
└── README_cn.md                               # 中文文档
```

---

## 3. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `master/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | 主臂 MIT 力反馈控制指令（臂 + 夹爪） |
| 发布 | `slave/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | 从臂 MIT 跟随控制指令（臂 + 夹爪） |
| 订阅 | `master/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 主臂实时状态（关节位置/速度/力矩） |
| 订阅 | `slave/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | 从臂实时状态（关节位置/速度/力矩） |
| 订阅 | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | 键盘按键状态 |

> [消息类型描述](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rate_ros` | 500.0 | 力反馈控制循环频率 [Hz] |
| `rate_teleop` | 100.0 | 键盘监听频率 [Hz] |
| `model_urdf` | "" | URDF 模型文件路径（用于动力学计算） |
| `model_frame_id` | `base_link` | 机器人基坐标系 |
| `pose_end_in_flange` | `[0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]` | 末端执行器在法兰盘上的位姿 [x, y, z, qw, qx, qy, qz] |
| `gravity` | `[0.0, 0.0, -9.81]` | 重力加速度向量 [m/s²] |
| `arm_start_pos` | `[0.0, 0.1, 2.54, -1.07, 0.0, 0.0]` | 机械臂启动稳定位置 [rad] |
| `arm_end_pos` | `[0.0, -1.5, 3.0, 0.07, 0.0, 0.0]` | 机械臂退出归位位置 [rad] |
| `grip_stable_pos` | `[0.5]` | 夹爪稳定位置 |
| `arm_stable_kp` | `[200.0, 200.0, 250.0, 150.0, 100.0, 100.0]` | 机械臂稳定运动 PD 增益 — 比例 |
| `arm_stable_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | 机械臂稳定运动 PD 增益 — 微分 |
| `grip_stable_kp` | `[10.0]` | 夹爪稳定运动 PD 增益 — 比例 |
| `grip_stable_kd` | `[0.5]` | 夹爪稳定运动 PD 增益 — 微分 |
| `arm_master_kp` | `[30.0, 30.0, 30.0, 10.0, 5.0, 1.5]` | 主臂力反馈 PD 增益 — 比例 |
| `arm_master_kd` | `[2.0, 2.0, 2.0, 2.1, 0.5, 0.5]` | 主臂力反馈 PD 增益 — 微分 |
| `grip_master_kp` | `[20.0]` | 主夹爪力反馈 PD 增益 — 比例 |
| `grip_master_kd` | `[0.0]` | 主夹爪力反馈 PD 增益 — 微分 |
| `arm_slave_kp` | `[200.0, 200.0, 250.0, 200.0, 100.0, 100.0]` | 从臂跟随 PD 增益 — 比例 |
| `arm_slave_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | 从臂跟随 PD 增益 — 微分 |
| `grip_slave_kp` | `[200.0]` | 从夹爪跟随 PD 增益 — 比例 |
| `grip_slave_kd` | `[10.0]` | 从夹爪跟随 PD 增益 — 微分 |
| `arm_master_deadzone` | `[0.1, 0.1, 0.1, 0.2, 0.10, 0.10]` | 主臂 deadzone — 误差低于此阈值归零 [rad] |
| `arm_master_clip` | `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]` | 主臂误差饱和限幅 [rad] |
| `arm_slave_deadzone` | `[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]` | 从臂 deadzone — 误差低于此阈值归零 [rad] |
| `arm_slave_clip` | `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` | 从臂误差饱和限幅 [rad] |
| `grip_master_deadzone` | `[0.01]` | 主夹爪 deadzone — 误差低于此阈值归零 [rad] |
| `grip_master_clip` | `[0.5]` | 主夹爪误差饱和限幅 [rad] |
| `grip_slave_deadzone` | `[0.01]` | 从夹爪 deadzone — 误差低于此阈值归零 [rad] |
| `grip_slave_clip` | `[0.3]` | 从夹爪误差饱和限幅 [rad] |

> 参数在 `config/` 中设置。ROS 1 与 ROS 2 的默认值在 `arm_start_pos` 和各增益参数上略有差异，以各自配置文件为准。

---

## 5. 依赖关系

### Python 包

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS 包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_force_feedback.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

---

## 6. 快速使用

### 1. 构建工作空间

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. 克隆包

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_force_feedback.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 3. 编译包

**ROS 1：**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2：**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. 使用包

本包提供四种启动场景的 launch 文件，PD 增益和 deadzone/clip 等参数在 `config/<ros_version>/arm_force_feedback.yaml` 中配置。

**ROS 2：**

```shell
# 仿真对仿真（sim2sim）：启动两个仿真环境 + 键盘遥控 + 力反馈节点
ros2 launch hex_ros_demo_arm_force_feedback sim2sim_force_feedback.launch.py \
    viewer:=true rviz:=false

# 真机对仿真（real2sim）：主臂是真机，从臂是仿真
ros2 launch hex_ros_demo_arm_force_feedback real2sim_force_feedback.launch.py \
    master_robot_host:=<master_ip> master_robot_port:=8439 robot_grip_type:=gr100

# 真机对真机（real2real）：主臂和从臂都是真机
ros2 launch hex_ros_demo_arm_force_feedback real2real_force_feedback.launch.py \
    master_robot_host:=<master_ip> master_robot_port:=8439 \
    slave_robot_host:=<slave_ip> slave_robot_port:=8439 robot_grip_type:=gr100

# 仅启动力反馈节点（需自行提供机械臂状态和控制驱动）
ros2 launch hex_ros_demo_arm_force_feedback arm_force_feedback.launch.py
```

**ROS 1：**

```shell
# 仿真对仿真（sim2sim）
roslaunch hex_ros_demo_arm_force_feedback sim2sim_force_feedback.launch viewer:=true rviz:=false

# 仅启动力反馈节点
roslaunch hex_ros_demo_arm_force_feedback arm_force_feedback.launch
```

> 确保 `config/` 中的参数配置正确，URDF 路径由 launch 文件自动设置。

键盘控制：

- **`s`** — 启动力反馈控制
- **`q`** — 停止力反馈控制，机械臂归位后退出
