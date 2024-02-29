// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ArmTeleop : public rclcpp::Node
{
public:
  ArmTeleop()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ArmTeleop::topic_callback, this, _1));
    
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy& joy_message) const
    {

      //RCLCPP_INFO(this->get_logger(), "I recieved a /Joy update");
      auto twist_msg = geometry_msgs::msg::Twist();

      joy_message.axes[0];
      twist_msg.linear.z = (joy_message.buttons[4] - joy_message.buttons[5]);
      if(twist_msg.linear.z != 0){
        RCLCPP_INFO(this->get_logger(), "Publishing Twist message");
      }
      publisher_->publish(twist_msg);
    }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTeleop>());
  rclcpp::shutdown();
  return 0;
}
