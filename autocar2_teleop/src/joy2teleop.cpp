#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "pop_interfaces/srv/motor.hpp"
#include "pop_interfaces/srv/steering.hpp"

using std::placeholders::_1;

namespace j2t
{
class joy2teleop : public rclcpp::Node
{
public:
  joy2teleop() : Node("joy2teleop")
  {
    joySb = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 0, std::bind(&joy2teleop::joy_callback, this, _1));

    motor = this->create_client<pop_interfaces::srv::Motor>("motor_service");
    steer = this->create_client<pop_interfaces::srv::Steering>("steering_service");

    this->declare_parameter<float>("msec", 100);
    this->msec = this->get_parameter("msec").get_value<float>();

    this->declare_parameter<int>("max_speed", 40);
    this->max_speed = this->get_parameter("max_speed").get_value<int>();

    motor_t = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&joy2teleop::motor_t_callback, this));
    RCLCPP_INFO(this->get_logger(),"init!");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySb;
  rclcpp::Client<pop_interfaces::srv::Motor>::SharedPtr motor;
  rclcpp::Client<pop_interfaces::srv::Steering>::SharedPtr steer;
  rclcpp::TimerBase::SharedPtr motor_t;
  std::vector<float> axes = std::vector<float>(8);
  float msec;
  int max_speed;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    axes = msg->axes;
  }
  void motor_t_callback()
  {
    auto motorRequest = std::make_shared<pop_interfaces::srv::Motor::Request>();
    auto steerRequest = std::make_shared<pop_interfaces::srv::Steering::Request>();
    RCLCPP_INFO(this->get_logger(),"joy received");
    //axes[1]: L ud, axes[3]: R rl
    if(axes[1]>0){
      motorRequest->direction = 1; //-1, 0, 1
      motorRequest->speed = axes[1]*max_speed; // 0~100
    }else if(axes[1]<0){
      motorRequest->direction = -1;
      motorRequest->speed = -axes[1]*max_speed;
    }

    steerRequest->steering = -axes[3]; //l ~1 ~ r 1

    RCLCPP_INFO(this->get_logger(), "Sending motor request: %f", axes[1]*max_speed);
    motor->async_send_request(
      motorRequest, std::bind(&joy2teleop::motor_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Sending steer request %f", steerRequest->steering);
    steer->async_send_request(
      steerRequest, std::bind(&joy2teleop::steer_callback, this, _1));
  }

  using ServiceResponseFutureM = rclcpp::Client<pop_interfaces::srv::Motor>::SharedFuture;
  void motor_callback(ServiceResponseFutureM future)
  {
    auto response = future.get();
    //RCLCPP_INFO(this->get_logger(), "Motor response: %d", response);
  }
  using ServiceResponseFutureS = rclcpp::Client<pop_interfaces::srv::Steering>::SharedFuture;
  void steer_callback(ServiceResponseFutureS future)
  {
    auto response = future.get();
    //RCLCPP_INFO(this->get_logger(), "Steer response: %d", response);
  }

};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<j2t::joy2teleop>());
  rclcpp::shutdown();
  return 0;
}




