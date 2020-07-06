#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "demo_neuronbot_gpio/srv/sema_gpio.hpp"

#include "linux/EApiOs.h"
#include "EApi.h"
#include "semaeapi.h"

#define IN_DIRECTION 1

namespace sema_server_node
{

class SemaServerNode : public rclcpp::Node
{
public:
  explicit SemaServerNode(const rclcpp::NodeOptions & options)
  : Node("sema_server", options)
  {
    uint32_t ret;
    char ipAddr[] = "127.0.0.1";
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // init SEMA
    ret = SemaEApiLibInitialize(false, IP_V4, ipAddr, 0, (char*)"123", &sema_handler);
    
    int _pin = 9;


    // set direction
    ret = SemaEApiGPIOSetDirection(sema_handler, EAPI_GPIO_GPIO_ID(_pin), 0x01, IN_DIRECTION);
    if (ret != EAPI_STATUS_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Can't initialize SEMA Lib. Make sure you are root. Error code: %X.", ret);
      return;
    }
    auto sema_gpio_service =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<demo_neuronbot_gpio::srv::SemaGpio::Request> request,
        std::shared_ptr<demo_neuronbot_gpio::srv::SemaGpio::Response> response) -> void
      {
        (void)request_header;
        if (request->gpio == "get") {
            // get level
            uint32_t level;
            int pin = 9;
            response->ret = SemaEApiGPIOGetLevel(sema_handler, EAPI_GPIO_GPIO_ID(pin),0x01, &level);
            response->level = level;
            }
      };
    // Create a service that will use the callback function to handle requests. The service's name is example_service_name.
    RCLCPP_INFO(this->get_logger(), "Service starts to listening...");
    srv_ = create_service<demo_neuronbot_gpio::srv::SemaGpio>("sema_gpio", sema_gpio_service);
  }
  ~SemaServerNode()
  {
    SemaEApiLibUnInitialize(sema_handler);
  }

private:
  rclcpp::Service<demo_neuronbot_gpio::srv::SemaGpio>::SharedPtr srv_;
  uint32_t sema_handler;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(sema_server_node::SemaServerNode)
