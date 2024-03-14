# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.descriptions import ComposableNode


from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_arm"), "config", "rover_arm.urdf.xacro"]
            ),

        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("rover_arm", package_name="rover_arm")
        .robot_description(file_path="config/rover_arm.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = {"moveit_servo": load_yaml("rover_arm", "config/servo_config.yaml")}

    # rviz_config_file = (
    #     get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
    # )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )



    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover_arm"),
            "config",
            "ros2_controllers.yaml",
        ]
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],

    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],

    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_arm_controller", "--controller-manager", "/controller_manager"],
    )




    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    # container = ComposableNodeContainer(
    #     name="moveit_servo_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         # Example of launching Servo as a node component
    #         # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
    #         # ComposableNode(
    #         #     package="moveit_servo",
    #         #     plugin="moveit_servo::ServoServer",
    #         #     name="servo_server",
    #         #     parameters=[
    #         #         servo_params,
    #         #         moveit_config.robot_description,
    #         #         moveit_config.robot_description_semantic,
    #         #     ],
    #         # ),
    #         ComposableNode(
    #             package="robot_state_publisher",
    #             plugin="robot_state_publisher::RobotStatePublisher",
    #             name="robot_state_publisher",
    #             parameters=[moveit_config.robot_description],
    #         ),
    #         ComposableNode(
    #             package="tf2_ros",
    #             plugin="tf2_ros::StaticTransformBroadcasterNode",
    #             name="static_tf2_broadcaster",
    #             parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
    #         ),
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #         ),
    #         ComposableNode(
    #             package="joy",
    #             plugin="joy::Joy",
    #             name="joy_node",
    #         ),
    #     ],
    #     output="screen",
    # )
    # # Launch a standalone Servo node.
    # # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    # servo_node = Node(
    #     package="joy_to_joint_controller",
    #     executable="joy_to_joint_controller_node",
    #     parameters=[
    #         servo_params,
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #     ],
    #     output="screen",
    # )





    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # servo_node,
        # container,
        #rviz_node,

    ]

    return LaunchDescription(declared_arguments + nodes)
