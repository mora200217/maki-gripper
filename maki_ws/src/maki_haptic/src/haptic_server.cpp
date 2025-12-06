#include "rclcpp/rclcpp.hpp"
#include "maki_interfaces/srv/haptic_feedback.hpp"
#include <curl/curl.h>
#include <sstream>
#include <future>

using HapticFeedback = maki_interfaces::srv::HapticFeedback;
using namespace std::chrono_literals;

// Función para enviar POST
std::pair<bool, std::string> send_haptic_post(int intensity, float duration) {
    std::ostringstream json;
    json << "{"
         << "\"intensity\":" << intensity << ","
         << "\"duration\":" << duration
         << "}";
    std::string json_str = json.str();

    CURL* curl = curl_easy_init();
    if (!curl) {
        return {false, "Failed to initialize curl"};
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");

    curl_easy_setopt(curl, CURLOPT_URL, "https://sherrell-presurgical-abe.ngrok-free.dev/metaquest/command");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, json_str.size());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L); // solo testing ngrok

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::string error_str = curl_easy_strerror(res);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        return {false, error_str};
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return {true, "Haptic command sent successfully"};
}

class HapticServer : public rclcpp::Node {
public:
    HapticServer() : Node("haptic_server") {
        RCLCPP_INFO(this->get_logger(), "HapticServer node started. Waiting for requests...");

        service_ = this->create_service<HapticFeedback>(
            "apply_haptic",
            [this](const std::shared_ptr<HapticFeedback::Request> request,
                   std::shared_ptr<HapticFeedback::Response> response) {

                int intensity = static_cast<int>(request->intensity); // convertir a entero
                float duration = request->duration;

                RCLCPP_INFO(this->get_logger(), "Received request: intensity=%d, duration=%.2f",
                            intensity, duration);

                // Validación intensidad 1-10 y duración positiva
                if (intensity < 1 || intensity > 10 || duration <= 0.0f) {
                    response->success = false;
                    response->response = "Invalid request values (intensity 1-10, duration > 0)";
                    RCLCPP_WARN(this->get_logger(), "Request rejected due to invalid values");
                    return;
                }

                // Ejecutar POST en segundo plano
                auto future_result = std::async(std::launch::async, send_haptic_post,
                                                intensity, duration);

                if (future_result.wait_for(6s) == std::future_status::ready) {
                    auto [ok, resp] = future_result.get();
                    response->success = ok;
                    response->response = resp;
                } else {
                    response->success = false;
                    response->response = "HTTP request timed out";
                    RCLCPP_ERROR(this->get_logger(), "HTTP request timed out");
                }

                RCLCPP_INFO(this->get_logger(), "Response: success=%s, message=%s",
                            response->success ? "true" : "false",
                            response->response.c_str());
            });
    }

private:
    rclcpp::Service<HapticFeedback>::SharedPtr service_;
};

int main(int argc, char **argv) {
    curl_global_init(CURL_GLOBAL_DEFAULT); // Inicializa CURL globalmente
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticServer>());
    rclcpp::shutdown();
    curl_global_cleanup();
    return 0;
}
