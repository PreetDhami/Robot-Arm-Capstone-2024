# rover_arm Package

This is the main package for the arm's code. The base for this was constructed with the `moveit_setup_assistant` package, following [this guide](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html).
## Configs
The config folder contains all the config and parameter files used

## Launch
The main launch file is `arm.launch.py` and runs the following nodes:
- `Rviz`: Handles displaying robot to gui.

- `control_node`: Loads the rover arm controller.

- `robot_state_pub_node`: Publishes the robot state to the `/state` topic. 

- `joint_state_broadcaster_spawner`:

- `robot_controller_spawner`:

- `joy_node`:

- `servo_node`:

- `joy_to_servo_node`: