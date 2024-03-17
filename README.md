# DAM ROBOTICS CLUB ROVER ARM CAPSTONE 2024
This code contains the ROS packages for the 2024 rover arm capstone made for the DAM Robotics Club. This was built on ROS2 Humble using the Moveit2! and ros2_control libraries. There is also a modified version of the official odrive_can ros_control plugin included.

To successfully run the code, a socketCAN interface must be running as "can0". Also, a gamepad with valid drivers must be plugged in. 

## Installation
A working installation of ROS2 Humble is required. To install the code, first clone the repository to a new directory:

```
mkdir arm_ws
cd arm_ws
git pull https://www.github.com/PreetDhami/Robot-Arm-Capstone-2024
```
Next, install dependencies using rosdep:

```
sudo apt-get install python3-rosdep
rosdep init
rosdep update
rosdep install --from-paths Robot-Arm-Capstone-2024 -y --ignore-src
```
Then, compile with colcon and source the installation:
```
colcon build && source install/setup.bash
```
Finally, to run the code with gamepad control:
```
ros2 launch rover_arm arm.launch.py
```


## Code Explanation
Below are all the included packages, a brief explanation of their content, and a link to reference documentation for further reading:
- `rover_arm_urdf`: This includes the URDF description of the rover arm, as well as the necessary mesh files (.stl and .dae). This was created by following [this article](https://control.ros.org/humble/doc/ros2_control_demos/example_7/doc/userdoc.html#writing-a-urdf) from `ros2_control` as an example

- `odrive_ros2_control` and `odrive_base`: These were originally made by Odrive Robotics from [this repo](https://github.com/odriverobotics/odrive_can/tree/ros-control), but were modified to consider gear ratios and units when communicating between ROS2 and the Odrive S1's. ROS uses radians, whereas the Odrives use rotations. `ros2_control` uses the angular positions and velocities of each joint rather than those of the motors.

- `joy_to_servo`: The package `moveit_servo` is used to do all calculations for operating the arm from some input. The `joy` package is used to read inputs from a gamepad. This package serves as a bridge between these two. Any efforts to re-map user control to operate the rover should be changed here. As of right now, inverse kinematics does not work with user operation, so all control is done on individual joints. This code was adapted from 

- `rover_arm`: Finally, this is the main package which contains the launch files for the arm, as well as ALL of the config files for the other packages.
