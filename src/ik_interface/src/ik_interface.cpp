#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>

#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;



class IKInterface : public rclcpp::Node
{
public:
    IKInterface() : Node("ik_interface_node"),          
        robot_model_loader_(std::make_shared<rclcpp::Node>(this->get_name()), "robot_description"),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_)),
        planning_group_(std::string("panda_arm")) // Change planning_group_ to match your robot

    {
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

        subscription_ = this->create_subscription<std_msgs::msg::String>("gripper_goal", 10, std::bind(&IKInterface::gripper_goal_callback, this, _1));
    }

private:
    robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    std::string planning_group_;
    const moveit::core::JointModelGroup* joint_model_group_;

    


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


    void gripper_goal_callback(const std_msgs::msg::String & msg) const {
        moveit::planning_interface::MoveGroupInterface move_group(std::make_shared<rclcpp::Node>(this->get_name()), planning_group_);

        auto current_pose = move_group.getCurrentPose();
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


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto ik_interface = std::make_shared<IKInterface>();

    rclcpp::spin(ik_interface);
    rclcpp::shutdown();
    return 0;
}

