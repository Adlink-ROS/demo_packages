#include "rclcpp/rclcpp.hpp"

#include "demo_neuronbot_gpio/srv/sema_gpio.hpp"

using namespace std::chrono_literals;  // used by 1s

demo_neuronbot_gpio::srv::SemaGpio::Response::SharedPtr send_request(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<demo_neuronbot_gpio::srv::SemaGpio>::SharedPtr client,
    demo_neuronbot_gpio::srv::SemaGpio::Request::SharedPtr request)
{
    auto result = client->async_send_request(request);
    // Waiting
    if (rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get();
    } else {
        return NULL;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Node name
    auto node = rclcpp::Node::make_shared("sema_client");
    // Ceraete a client to query service name "sema_cmd"
    auto client = node->create_client<demo_neuronbot_gpio::srv::SemaGpio>("sema_gpio");

    // Build the request
    auto request = std::make_shared<demo_neuronbot_gpio::srv::SemaGpio::Request>();
    request->gpio = (argc >= 2)?argv[1]:"get";

    // Waiting until the service is up
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    }

    // Send the request
    auto result = send_request(node, client, request);
    if (result) {
        RCLCPP_INFO(node->get_logger(), "level = %d", result->level);
        if (result->level == 1){
            RCLCPP_INFO(node->get_logger(), "User does not push the button!");
            }
        else if (result->level == 0){
            RCLCPP_INFO(node->get_logger(), "User is pushing the button!");
            }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
    }

    rclcpp::shutdown();

    return 0;
}
