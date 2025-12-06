#include "rclcpp/rclcpp.hpp"
#include "maki_interfaces/srv/haptic_feedback.hpp"
#include <memory>
#include <string>

using HapticFeedback = maki_interfaces::srv::HapticFeedback;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("haptic_client");

    if (argc < 3) {
        RCLCPP_ERROR(node->get_logger(), "Usage: haptic_client <intensity(1-10)> <duration(seconds)>");
        return 1;
    }

    int intensity = std::stoi(argv[1]);
    float duration = std::stof(argv[2]);

    if (intensity < 1 || intensity > 10 || duration <= 0.0f) {
        RCLCPP_ERROR(node->get_logger(), "Invalid values. Intensity 1-10, duration > 0");
        return 1;
    }

    auto client = node->create_client<HapticFeedback>("apply_haptic");

    while (!client->wait_for_service(1s)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
        rclcpp::spin_some(node);
    }

    auto request = std::make_shared<HapticFeedback::Request>();
    request->intensity = static_cast<float>(intensity);
    request->duration = duration;

    auto result_future = client->async_send_request(request);

    // Esperar resultado
    while (rclcpp::ok()) {
        auto status = result_future.wait_for(500ms);
        if (status == std::future_status::ready) {
            auto result = result_future.get();
            RCLCPP_INFO(node->get_logger(), "Success: %s, Response: %s",
                        result->success ? "true" : "false",
                        result->response.c_str());
            break;
        }
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
