#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"

#include "linux/EApiOs.h"
#include "EApi.h"
#include "semaeapi.h"

#define OUT_DIRECTION 0
#define LEVEL_LOW 0
#define LEVEL_HIGH 1

using std::placeholders::_1;
namespace led_subscriber_node
{

class LEDSubscriber : public rclcpp::Node
{
public:
  explicit LEDSubscriber(const rclcpp::NodeOptions & options)
  : Node("led_sub_node", options)
  { 

    RCLCPP_INFO(this->get_logger(), "Start Example: LED Blink");
    sub_ = create_subscription<std_msgs::msg::Int32>("led_blink", 1000, std::bind(&LEDSubscriber::counterCallback, this, _1));

    // init SEMA
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"---init SEMA---");
    ret = SemaEApiLibInitialize(false, IP_V4, addr, 0, (char*)"123", &libHandle_);
    if (ret != EAPI_STATUS_SUCCESS){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "[ERROR] Neuron_GPIO -Can't initialize SEMA Lib. Error code: %X\n", ret);
        }

    // set direciton
    ret = SemaEApiGPIOSetDirection(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, OUT_DIRECTION);
    if (ret != EAPI_STATUS_SUCCESS){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "[ERROR] Neuron_GPIO --- Error setting GPIO direction: 0x%X\n", ret);
        }
  }

  void counterCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Receiving GPIO Level: %d", msg->data);
    if(msg->data == 1){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"LED ON\n");
        // Set level low
        ret = SemaEApiGPIOSetLevel(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, LEVEL_HIGH);
        if (ret != EAPI_STATUS_SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
            "[ERROR] Neuron_GPIO --- Error setting GPIO level: 0x%X\n", ret);
           }
        }

    else if(msg->data == 0){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"LED OFF\n");
        // Set level high
        ret = SemaEApiGPIOSetLevel(libHandle_, EAPI_GPIO_GPIO_ID(_pin), 0x01, LEVEL_LOW);
        if (ret != EAPI_STATUS_SUCCESS){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
            "[ERROR] Neuron_GPIO --- Error setting GPIO level: 0x%X\n", ret);
           }
        }

}

  ~LEDSubscriber()
  {
    // uninitailize SEMA
    SemaEApiLibUnInitialize(libHandle_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  uint32_t libHandle_;
  char addr[16] = "127.0.0.1";
  uint32_t ret = 0;

  // init GPIO
  int _pin = 7;
};

}


RCLCPP_COMPONENTS_REGISTER_NODE(led_subscriber_node::LEDSubscriber)

