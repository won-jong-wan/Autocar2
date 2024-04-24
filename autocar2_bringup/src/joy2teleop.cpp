#ifndef AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_
#define AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
//#include "pop_interfaces/srv/moter.hpp"

using std::placeholders::_1;

namespace joy2teleop
{
class joy2teleop : public rclcpp::Node
{
public:
  joy2teleop() : Node("joy2teleop")
  {
    joySb = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&joy2teleop::joy_callback, this, _1));

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySb;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received joy message");
  }

};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy2teleop::joy2teleop>());
  rclcpp::shutdown();
  return 0;
}
 // namespace /* namespace_name */
#endif  // AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_
