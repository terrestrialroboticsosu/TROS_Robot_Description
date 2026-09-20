# view_robot_pkg 
## Visualize a Robot's URDF in RViz (ROS2)

This ROS2 package lets you **visualize your robot’s URDF file** in **RViz2**.  
It also runs the **Joint State Publisher GUI**, so you can move the robot’s joints and see how they move in real time.

---
## How to Use

Run this command:

```bash
ros2 launch tros_sldworks_pkg view_robot.launch
```

Or to view current version in gazebo run: 
```bash
ros2 launch tros_sldworks_pkg view_gazebo.launch.py
```
In order to start joystick sim:
```bash
ros2 launch tros_sldworks_pkg view_gazebo.launch.py
ros2 run controller_manager spawner diff_cont
ros2 run controller_manager spawner joint_broad
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node --ros-args -r /cmd_vel:=/diff_cont/cmd_vel -p publish_stamped_twist:=true --params-file ~/tros_sldworks_pkg/config/xbox_controller_config.yaml
```

from there hold down right bumper and steer with left joystick

## Dependencies
```bash
sudo apt install ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-robot-state-publisher
```
