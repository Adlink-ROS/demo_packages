//#include <chrono>
//#include <functional>
//#include <memory>
//#include <string>

#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
/*
using namespace std::chrono_literals;

class LEDPublisher : public rclcpp::Node
{
    public:
      LEDPublisher() : Node("led_pub_node"), count_(0)
      {
        publisher_ = this->create_publisher<std_msgs::msg::String>("led_blink", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&LEDPublisher::counterCallback, this));
      }
    
    private:
      void counterCallback()
      {
          auto message = std_msgs::msg::String();
          message.data = "Hello, world! " + std::to_string(count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          publisher_->publish(message);
       }
       rclcpp::TimerBase::SharedPtr timer_;
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       size_t count_;


    };

    int main(int argc, char* argv[])
    {
      rclcpp::init(argc, argv);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Example: LED Blinki");
      rclcpp::spin(std::make_shared<LEDPublisher>());
      rclcpp::shutdown();
      return 0;
    }
*/
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start Example: LED Blink");
  auto node = rclcpp::Node::make_shared("led_pub_node");
  auto pub = node->create_publisher<std_msgs::msg::Int32>("led_blink", 1000);
  rclcpp::Rate loop_rate(1);
  int level = 0;
  std_msgs::msg::Int32 msg;

  while (rclcpp::ok())
  {
    RCLCPP_INFO(node->get_logger(), "%d\n", level);
    msg.data = level;
    pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    level = rand() % 2;
  }
  return 0;
}

