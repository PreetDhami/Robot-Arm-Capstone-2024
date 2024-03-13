#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>


#include "sensor_msgs/msg/joy.hpp"


using std::placeholders::_1;


class IKInterface : public rclcpp::Node
{
public:
    IKInterface()
        : Node("ik_interface_node"),
          robot_model_loader_(std::make_shared<rclcpp::Node>(this->get_name()), "robot_description"),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_)),
          planning_group_("rover_arm_controller"), // Change planning_group_ to match your robot
          move_group_interface(std::make_shared<rclcpp::Node>(this->get_name()), "rover_arm")
    {   
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

    }

    void sampleSolveIK()
    {
        
        // get current pose, increase its z coordinate by 5 cm and solve IK to see if the new pose is reachable
        auto current_pose = move_group_interface.getCurrentPose();
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.pose.position.x;
        target_pose.position.y = current_pose.pose.position.y;
        target_pose.position.z = current_pose.pose.position.z + 0.05; // Fixed to directly modify the z coordinate
        
        bool found_ik = robot_state_->setFromIK(joint_model_group_, target_pose);
        if (found_ik)
        {
            RCLCPP_INFO(this->get_logger(), "IK solution found.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "IK solution not found.");
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface;

    robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::string planning_group_;
    const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto ik_interface = std::make_shared<IKInterface>();
    ik_interface->sampleSolveIK(); 

    rclcpp::spin(ik_interface);
    rclcpp::shutdown();
    return 0;
}