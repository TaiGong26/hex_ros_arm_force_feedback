# hex_ros_arm_force_feedback

## What does this package do

This package is an **impedance control demo** for the Archer Y6 arm that works in **both ROS 1 and ROS 2**.

The node first drives the arm to a stable start pose, then switches into impedance control: at every control cycle it reads the latest arm state, limits the SE(3) pose error toward the stable pose, solves analytic IK for a target joint position, and publishes a `MIT` control command with impedance stiffness/damping. The robot driver (or the [`hex_ros_sim_archer_y6`](../hex_ros_sim_archer_y6) simulator) adds the model gravity/coriolis compensation, so the arm compliantly returns toward the stable pose when disturbed.

A keyboard interface (see [`hex_ros_teleop_keyboard`](../hex_ros_teleop_keyboard)) is used for runtime control:

* press **`q`** to stop the demo and move the arm back to the stable pose.

Data recording is left to ROS's built-in bag tools (`ros2 bag record` / `rosbag record`).

## Maintainer

[Dong Zhaorui](https://github.com/IBNBlank)

## Prerequisites

Ensure the following software is installed:

* **ROS**: Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)
* **hex_ros_msgs**: provides the robot/teleop message definitions.
* **hex_ros_urdf_archer_y6**: provides the `gr100_comp.urdf` used for the dynamics model.
* A state/control source for the arm, e.g. **hex_ros_sim_archer_y6**.
* A keyboard source, e.g. **hex_ros_teleop_keyboard**.

### Verified Platforms

* [x] **x64**
* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

## Public APIs

### Published Topics

| Topic         | Msg Type                                | Description                                       |
| ------------- | --------------------------------------- | ------------------------------------------------ |
| `/manip_ctrl` | `hex_ros_msgs/HexRosRoboManipCtrlStamped` | Arm + gripper control command.                   |

### Subscribed Topics

| Topic                    | Msg Type                                       | Description                  |
| ------------------------ | ---------------------------------------------- | ---------------------------- |
| `/manip_state`           | `hex_ros_msgs/HexRosRoboManipStateStamped`     | Current arm + gripper state. |
| `/teleop_keyboard_state` | `hex_ros_msgs/HexRosTeleopKeyboardStateStamped` | Keyboard key states.         |

### Parameters

| Name                          | Data Type        | Description                                              |
| ----------------------------- | ---------------- | -------------------------------------------------------- |
| `rate_ros`                    | `double`         | Impedance control work loop rate [hz].                  |
| `rate_teleop`                 | `double`         | Keyboard monitor rate [hz].                             |
| `model_urdf`                  | `string`         | Path to the URDF used for the dynamics model.           |
| `model_frame_id`              | `string`         | Frame id of the robot base.                             |
| `pose_end_in_flange`          | `vector<double>` | End-effector pose in flange `[x,y,z,qw,qx,qy,qz]`.      |
| `gravity`                     | `vector<double>` | Gravity vector `[x,y,z]` [m/s^2].                       |
| `arm_stable_pos`              | `vector<double>` | Arm joint stable (init/exit) position [rad].            |
| `grip_stable_pos`             | `vector<double>` | Gripper stable position.                                |
| `arm_kp` / `arm_kd`           | `vector<double>` | Arm gains used while moving to the stable position.     |
| `grip_kp` / `grip_kd`         | `vector<double>` | Gripper gains used while moving to the stable position. |
| `arm_impedance_kp` / `kd`     | `vector<double>` | Arm gains used during impedance control.                |
| `grip_impedance_kp` / `kd`    | `vector<double>` | Gripper gains used during impedance control.            |
| `arm_se3_threshold`           | `double`         | Max SE(3) error step applied per cycle.                 |
| `arrive_threshold`            | `double`         | Max joint error [rad] to consider the pose reached.     |

## Getting Started

1. Install necessary dependencies:

   ```shell
   pip3 install 'hex-util-msg>=0.1.0a0'
   pip3 install 'hex-util-ros>=0.0.1a0'
   ```

2. Create a workspace and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

3. Clone the repository:

   ```shell
   git clone https://github.com/hexfellow/hex_ros_arm_force_feedback.git
   ```

4. Navigate back and build the workspace:

   For ROS 1:

   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:

   ```shell
   cd ../
   colcon build
   ```

5. Source the `setup.bash` file:

   For ROS 1:

   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:

   ```shell
   source install/setup.bash --extend
   ```

### Usage

1. Start an arm state/control source (e.g. the simulator) and the keyboard node:

   For ROS 2:

   ```shell
   ros2 launch hex_ros_sim_archer_y6 sim_archer_y6.launch.py
   ros2 launch hex_ros_teleop_keyboard teleop_keyboard.launch.py
   ```

2. Launch the `arm_impedance` node:

   For ROS 1:

   ```shell
   roslaunch hex_ros_arm_force_feedback arm_impedance.launch
   ```

   For ROS 2:

   ```shell
   ros2 launch hex_ros_arm_force_feedback arm_impedance.launch.py
   ```

3. The arm moves to the stable pose and then enters impedance control. Press `q` to exit. To record data, use ROS's bag tools, e.g. `ros2 bag record -a`.
