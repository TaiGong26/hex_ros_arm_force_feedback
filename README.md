# hex_ros_demo_arm_force_feedback
[中文](README_cn.md) | **English**

## Table of Contents

- [1. About](#1-about)
- [2. Package Structure](#2-package-structure)
- [3. Topics](#3-topics)
- [4. Parameters](#4-parameters)
- [5. Dependencies](#5-dependencies)
- [6. Quick Start](#6-quick-start)

---

## 1. About

This is the **bilateral force feedback demo** for the **HEXFELLOW** Archer Y6 robotic arm, enabling telepresence control between master and slave arms.

The force feedback control loop centers on `__feedback`, performing the following functions:

- **Master-slave tracking control** — At each cycle, reads real-time joint states from both master and slave arms, computes valid target positions via deadzone compensation and clip saturation limiting, and publishes MIT impedance control commands: the slave arm follows the master arm position (`__build_follow_ctrl`, slave-side PD gains), while the master arm reflects the slave arm's force feedback (`__build_feedback_ctrl`, master-side PD gains).
- **Gripper force feedback** — 1-DoF gripper follows the same pattern, supporting bilateral position tracking and force feedback for both master and slave grippers.
- **Smooth start/exit** — On start and exit, smoothly transitions both arms from their current positions to stable positions.
- **Keyboard control** — Press **`s`** to start force feedback control, press **`q`** to stop and return to home position.

Supports both **ROS 1** and **ROS 2**, with four launch scenarios: standalone node, simulation-to-simulation (sim2sim), real-to-simulation (real2sim), and real-to-real (real2real).

---

## 2. Package Structure

```
hex_ros_demo_arm_force_feedback/
├── config/                                    # Configuration files
│   ├── ros1/
│   │   └── arm_force_feedback.yaml            #   ROS 1 params (force_feedback)
│   └── ros2/
│       └── arm_force_feedback.yaml            #   ROS 2 params (force_feedback)
├── launch/                                    # ROS launch files
│   ├── ros1/
│   │   ├── arm_force_feedback.launch          #   Standalone node launch
│   │   └── sim2sim_force_feedback.launch      #   Sim-to-sim full launch
│   └── ros2/
│       ├── arm_force_feedback.launch.py       #   Standalone node launch
│       ├── real2real_force_feedback.launch.py #   Real-to-real full launch
│       ├── real2sim_force_feedback.launch.py  #   Real-to-sim full launch
│       └── sim2sim_force_feedback.launch.py   #   Sim-to-sim full launch
├── hex_ros_demo_arm_force_feedback/           # Core source
│   ├── arm_force_feedback.py                  #   Main node: bilateral force feedback control loop
│   ├── TrajectoryController.py                #   Trajectory planner
│   └── utility/                               #   Dual-layer ROS interface abstraction
│       ├── __init__.py                        #     ROS version selector (ROS_VERSION env var)
│       ├── interface_base.py                  #     Abstract base class (InterfaceBase)
│       ├── ros1_interface.py                  #     ROS 1 DataInterface
│       └── ros2_interface.py                  #     ROS 2 DataInterface
├── resource/                                  # ament resource index
├── setup.py                                   # Python packaging (ROS 2)
├── CMakeLists.txt                             # CMake packaging (ROS 1)
├── package.xml                                # ROS package manifest (dual-system conditional deps)
├── README.md                                  # English documentation
└── README_cn.md                               # Chinese documentation
```

---

## 3. Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| pub | `master/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Master arm MIT force feedback control command (arm + gripper) |
| pub | `slave/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Slave arm MIT tracking control command (arm + gripper) |
| sub | `master/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Master arm real-time state (joint position/velocity/torque) |
| sub | `slave/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Slave arm real-time state (joint position/velocity/torque) |
| sub | `teleop_keyboard_state` | `hex_ros_msgs/(msg/)HexRosTeleopKeyboardStateStamped` | Keyboard key state |

> [Message Type Description](https://github.com/hexfellow/hex_ros_msgs#public-apis)

---

## 4. Parameters

| Param | Default | Description |
|-------|---------|-------------|
| `rate_ros` | 500.0 | Force feedback control loop rate [Hz] |
| `rate_teleop` | 100.0 | Keyboard monitor rate [Hz] |
| `model_urdf` | "" | URDF model file path (for dynamics computation) |
| `model_frame_id` | `base_link` | Robot base frame ID |
| `pose_end_in_flange` | `[0.187, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]` | End-effector pose in flange [x, y, z, qw, qx, qy, qz] |
| `gravity` | `[0.0, 0.0, -9.81]` | Gravity acceleration vector [m/s²] |
| `arm_start_pos` | `[0.0, 0.1, 2.54, -1.07, 0.0, 0.0]` | Arm stable start position [rad] |
| `arm_end_pos` | `[0.0, -1.5, 3.0, 0.07, 0.0, 0.0]` | Arm exit home position [rad] |
| `grip_stable_pos` | `[0.5]` | Gripper stable position |
| `arm_stable_kp` | `[200.0, 200.0, 250.0, 150.0, 100.0, 100.0]` | Arm stable motion PD gain — proportional |
| `arm_stable_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Arm stable motion PD gain — derivative |
| `grip_stable_kp` | `[10.0]` | Gripper stable motion PD gain — proportional |
| `grip_stable_kd` | `[0.5]` | Gripper stable motion PD gain — derivative |
| `arm_master_kp` | `[30.0, 30.0, 30.0, 10.0, 5.0, 1.5]` | Master arm force feedback PD gain — proportional |
| `arm_master_kd` | `[2.0, 2.0, 2.0, 2.1, 0.5, 0.5]` | Master arm force feedback PD gain — derivative |
| `grip_master_kp` | `[20.0]` | Master gripper force feedback PD gain — proportional |
| `grip_master_kd` | `[0.0]` | Master gripper force feedback PD gain — derivative |
| `arm_slave_kp` | `[200.0, 200.0, 250.0, 200.0, 100.0, 100.0]` | Slave arm tracking PD gain — proportional |
| `arm_slave_kd` | `[5.0, 5.0, 5.0, 5.0, 2.0, 2.0]` | Slave arm tracking PD gain — derivative |
| `grip_slave_kp` | `[200.0]` | Slave gripper tracking PD gain — proportional |
| `grip_slave_kd` | `[10.0]` | Slave gripper tracking PD gain — derivative |
| `arm_master_deadzone` | `[0.1, 0.1, 0.1, 0.2, 0.10, 0.10]` | Master arm deadzone — error below threshold is zeroed [rad] |
| `arm_master_clip` | `[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]` | Master arm error saturation clip [rad] |
| `arm_slave_deadzone` | `[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]` | Slave arm deadzone — error below threshold is zeroed [rad] |
| `arm_slave_clip` | `[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]` | Slave arm error saturation clip [rad] |
| `grip_master_deadzone` | `[0.01]` | Master gripper deadzone — error below threshold is zeroed [rad] |
| `grip_master_clip` | `[0.5]` | Master gripper error saturation clip [rad] |
| `grip_slave_deadzone` | `[0.01]` | Slave gripper deadzone — error below threshold is zeroed [rad] |
| `grip_slave_clip` | `[0.3]` | Slave gripper error saturation clip [rad] |

> Parameters are set in `config/`. Default values for `arm_start_pos` and various gain parameters differ slightly between ROS 1 and ROS 2 — refer to the respective config files.

---

## 5. Dependencies

### Python Packages

```shell
pip3 install 'hex-util-msg>=0.1.0'
pip3 install 'hex-util-ros>=0.0.1a0'
pip3 install 'hex-driver-robot>=0.1.0'
```

### ROS Packages

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_force_feedback.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

---

## 6. Quick Start

### 1. Create Workspace

```shell
mkdir -p <your_ws>/src
cd <your_ws>/src
```

### 2. Clone Repositories

```shell
git clone https://github.com/hexfellow/hex_ros_msgs.git
git clone https://github.com/hexfellow/hex_ros_demo_arm_force_feedback.git
git clone https://github.com/hexfellow/hex_ros_robot_arm.git
git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
git clone https://github.com/hexfellow/hex_ros_teleop_keyboard.git
git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
```

### 3. Build

**ROS 1:**

```shell
source /opt/ros/noetic/setup.bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

**ROS 2:**

```shell
source /opt/ros/humble/setup.bash
cd <your_ws>
colcon build
source install/setup.bash
```

### 4. Use

This package provides launch files for four scenarios. PD gains, deadzone, clip, and other parameters are configured in `config/<ros_version>/arm_force_feedback.yaml`.

**ROS 2:**

```shell
# Simulation-to-simulation (sim2sim): launches two simulators + keyboard teleop + force feedback node
ros2 launch hex_ros_demo_arm_force_feedback sim2sim_force_feedback.launch.py \
    viewer:=true rviz:=false

# Real-to-simulation (real2sim): master is real robot, slave is simulation
ros2 launch hex_ros_demo_arm_force_feedback real2sim_force_feedback.launch.py \
    master_robot_host:=<master_ip> master_robot_port:=8439 robot_grip_type:=gr100

# Real-to-real (real2real): both master and slave are real robots
ros2 launch hex_ros_demo_arm_force_feedback real2real_force_feedback.launch.py \
    master_robot_host:=<master_ip> master_robot_port:=8439 \
    slave_robot_host:=<slave_ip> slave_robot_port:=8439 robot_grip_type:=gr100

# Start force feedback node only (requires separate arm state/control drivers)
ros2 launch hex_ros_demo_arm_force_feedback arm_force_feedback.launch.py
```

**ROS 1:**

```shell
# Simulation-to-simulation (sim2sim)
roslaunch hex_ros_demo_arm_force_feedback sim2sim_force_feedback.launch viewer:=true rviz:=false

# Start force feedback node only
roslaunch hex_ros_demo_arm_force_feedback arm_force_feedback.launch
```

> Ensure parameters in `config/` are correctly set. The URDF path is set automatically by the launch file.

Keyboard control:

- **`s`** — Start force feedback control
- **`q`** — Stop force feedback control; arms return to home position and exit
