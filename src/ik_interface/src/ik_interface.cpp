#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "moveit/move_group_interface/move_group_interface.h"


using moveit::planning_interface::MoveGroupInterface;
using std::placeholders::_1;

class IKInterface : public rclcpp::Node{
  public:
    IKInterface() : Node("ik_interface"){
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "pose_goal", 10, std::bind(&IKInterface::topic_callback, this, _1));
    }

  private:

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    void topic_callback(const geometry_msgs::msg::Pose &goal_pose) const{
      RCLCPP_INFO(this->get_logger(), "x val: %.5f\ny val: %.5f\nz val: %.5f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKInterface>());
  rclcpp::shutdown();
  return 0;
}