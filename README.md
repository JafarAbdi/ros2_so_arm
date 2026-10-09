# ROS 2 package for the SO-ARMs

Compact ROS 2 description, drivers and MoveIt configuration for the **[SO-ARM100 and SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100)** robotic arms.
Leveraging the [`feetech_ros2_driver`](https://github.com/JafarAbdi/feetech_ros2_driver) package.

<table>
  <tr>
    <td style="vertical-align: top; padding-right: 16px;">
      <div style="margin-bottom: 12px;">
        <video
          src="https://github.com/user-attachments/assets/5655b956-5536-4143-9707-17cad5d1cbc8"
          controls
          muted
          loop
        ></video>
      </div>
      <div>
        <video
          src="https://github.com/user-attachments/assets/abf55009-3655-4e24-bebe-3b65d5361f03"
          controls
          muted
          loop
        ></video>
      </div>
    </td>
    <td style="vertical-align: top;">
      <div style="margin-bottom: 12px;">
        <video
          src="https://github.com/user-attachments/assets/36ccaca0-82dd-4206-a4dd-953867e89a20"
          controls
          muted
          loop
        ></video>
      </div>
      <div>
        <video
          src="https://github.com/user-attachments/assets/4c50575a-0a44-43e2-adc7-abf2d15a16f2"
          controls
          muted
          loop
        ></video>
      </div>
    </td>
  </tr>
</table>

## Installation

```bash
# 1. Create a workspace
mkdir -p ~/so_arm_ws/src
cd ~/so_arm_ws/src

# 2. Clone core packages
git clone https://github.com/JafarAbdi/ros2_so_arm100.git
git clone https://github.com/JafarAbdi/feetech_ros2_driver.git

# 3. (Optional) MuJoCo packages
git clone https://github.com/ros-controls/mujoco_ros2_control.git

# 4. Install ROS dependencies
cd ~/so_arm_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install
source install/setup.bash
```


## Quick Start

### Full Demo (RViz + controllers + MoveIt)

```bash
ros2 launch so_arm100_moveit_config demo.launch.py \
  hardware_type:=mock_components   # or :=real, :=gazebo, :=mujoco
```

> [!TIP]
> - `mock_components` -> RViz-only
> - `real` -> USB, default to `/dev/LeRobotFollower`, override with `usb_port:=<device>`
> - `gazebo` -> Gazebo simulation
> - `mujoco` -> MuJoCo simulation

> [!NOTE]
> To use the SO-ARM101, use the `so_arm101_description` package instead.

### Bring-up Only (no RViz)

| Purpose               | Command                                                                                          |
| --------------------- | ------------------------------------------------------------------------------------------------ |
| MoveIt server         | `ros2 launch so_arm100_moveit_config move_group.launch.py`                                       |
| Low-level controllers | `ros2 launch so_arm100_description controllers_bringup.launch.py hardware_type:=mock_components` |

### Visualisation Shortcuts

| View                    | Command                                                                                     |
| ----------------------- | ------------------------------------------------------------------------------------------- |
| Robot model in RViz     | `ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share so_arm100_description)/rviz/config.rviz` |
| RViz with MoveIt plugin | `ros2 launch so_arm100_moveit_config moveit_rviz.launch.py`                                 |

### Interact & Test

| Tool                     | Command                                                                                                                      |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| Joint trajectory GUI     | `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`                                                   |
| MoveIt Setup Assistant\* | `ros2 run moveit_setup_assistant moveit_setup_assistant --config_pkg ~/so_arm_ws/src/ros2_so_arm100/so_arm100_moveit_config` |

*Use the assistant to tweak or regenerate MoveIt configs.*

### Gazebo Simulation

Launch standalone Gazebo simulation with ros2_control (no MoveIt):

```bash
ros2 launch so_arm_gz so_arm_gz_bringup.launch.py arm_id:=so_arm101 # or amr_id:=so_arm100
```

> [!TIP]
> See [so_arm_gz/README.md](so_arm_gz/README.md) for all configuration options.

### Joint Zero Calibration

Measure how far each joint's reading is from the description's angle, by touching dots on the
table with the arm's tips, and fold the offsets into the URDF:

```bash
ros2 run so_arm_calibration calibrate_zeros --port /dev/ttyACM0 --release
```

> [!TIP]
> See [so_arm_calibration/README.md](so_arm_calibration/README.md) for the procedure.
