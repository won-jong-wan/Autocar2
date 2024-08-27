#ifndef AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_
#define AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "pop_interfaces/srv/motor.hpp"
#include "pop_interfaces/srv/steering.hpp"

#include <memory>

using std::placeholders::_1;

namespace j2t
{
  class joy2teleop : public rclcpp::Node
  {
  public:
    joy2teleop() : Node("autocar2")
    {
      joySb = this->create_subscription<sensor_msgs::msg::Joy>(
          "joy", 10, std::bind(&joy2teleop::joy_callback, this, _1));

      motor = this->create_client<pop_interfaces::srv::Motor>("motor_service");
      steer = this->create_client<pop_interfaces::srv::Steering>("steering_service");
      RCLCPP_INFO(this->get_logger(), "init!");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySb;
    rclcpp::Client<pop_interfaces::srv::Motor>::SharedPtr motor;
    rclcpp::Client<pop_interfaces::srv::Steering>::SharedPtr steer;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      auto motorRequest = std::make_shared<pop_interfaces::srv::Motor::Request>();
      auto steerRequest = std::make_shared<pop_interfaces::srv::Steering::Request>();
      RCLCPP_INFO(this->get_logger(), "joy received");
      // axes[1]: L ud, axes[3]: R rl
      if (msg->axes[1] > 0)
      {
        motorRequest->direction = 1;              //-1, 0, 1
        motorRequest->speed = msg->axes[1] * 100; // 0~100
      }
      else if (msg->axes[1] < 0)
      {
        motorRequest->direction = -1;
        motorRequest->speed = -msg->axes[1] * 100;
      }
      else
      {
        motorRequest->direction = 0;
        motorRequest->speed = 0;
      }

      steerRequest->steering = -msg->axes[3]; // l ~1 ~ r 1

      RCLCPP_INFO(this->get_logger(), "Sending motor request");
      motor->async_send_request(
          motorRequest, std::bind(&joy2teleop::motor_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "Sending steer request");
      steer->async_send_request(
          steerRequest, std::bind(&joy2teleop::steer_callback, this, _1));
    }
    using ServiceResponseFutureM = rclcpp::Client<pop_interfaces::srv::Motor>::SharedFuture;
    void motor_callback(ServiceResponseFutureM future)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Motor response: %d", response);
    }
    using ServiceResponseFutureS = rclcpp::Client<pop_interfaces::srv::Steering>::SharedFuture;
    void steer_callback(ServiceResponseFutureS future)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Steer response: %d", response);
    }
  };
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  //RCLCPP_INFO(this->get_logger(), "main");
  rclcpp::spin(std::make_shared<j2t::joy2teleop>());
  rclcpp::shutdown();
  return 0;
}
// namespace /* namespace_name */
#endif // AUTOCAR2_BRINGUP__JOY2TELEOP_HPP_
