#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"



using std::placeholders::_1;

class JoyToJointController : public rclcpp::Node{
  public:
    JoyToJointController() : Node("joy_to_joint_controller"){
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToJointController::topic_callback, this, _1));
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("topic", 10);

    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy & joy_msg) const{
    
      if(joy_msg.axes[1] > 0.5){
        RCLCPP_INFO(this->get_logger(), "msg recieved");
      }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<JoyToJointController>());
  rclcpp::shutdown();
  return 0;
}