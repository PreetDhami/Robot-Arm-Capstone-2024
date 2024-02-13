#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>

class IKInterface : public rclcpp::Node
{
public:
    IKInterface()
        : Node("ik_interface_node"),
            planning_group_("panda_arm"),           // Change planning_group_ to match your robot
          robot_model_loader_(std::make_shared<rclcpp::Node>(this->get_name()), "robot_description"),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(std::make_shared<moveit::core::RobotState>(robot_model_))

    {   
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
    }

private:
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

    rclcpp::spin(ik_interface);
    rclcpp::shutdown();
    return 0;
}

