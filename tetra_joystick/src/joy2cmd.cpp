#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define LIN_VEL_GAIN 1.0
#define ANG_VEL_GAIN 1.0

class SendCMD : public rclcpp::Node
{
  public:
    SendCMD()
    : Node("joy2cmd"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&SendCMD::joystic_callback, this, _1));

      timer_ = this->create_wall_timer(
      100ms, std::bind(&SendCMD::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = this->linear_x;
      message.angular.z = this->angular_z;
      RCLCPP_INFO(this->get_logger(), "linear_x : '%.2f', angular_z : '%.2f'", this->linear_x, this->angular_z);
      publisher_->publish(message);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    size_t count_;

    float linear_x, angular_z;

    void joystic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      this->linear_x = msg->axes[1]*LIN_VEL_GAIN;
      this->angular_z = msg->axes[0]*ANG_VEL_GAIN;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendCMD>());
  rclcpp::shutdown();
  return 0;
}