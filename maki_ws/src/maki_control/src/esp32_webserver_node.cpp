#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <boost/asio.hpp>
#include <string>
#include <sstream>

using boost::asio::ip::tcp;
using std::placeholders::_1;

class MakiControlNode : public rclcpp::Node
{
public:
    MakiControlNode()
    : Node("maki_control_node"),
      io_context_(),
      resolver_(io_context_)
    {
        declare_parameter<std::string>("esp32_ip", "192.168.0.50");

        sub_motor1_ = create_subscription<std_msgs::msg::Int32>(
            "/motor1/motor_cmd", 10,
            std::bind(&MakiControlNode::motor1_callback, this, _1)
        );

        sub_motor2_ = create_subscription<std_msgs::msg::Int32>(
            "/motor2/motor_cmd", 10,
            std::bind(&MakiControlNode::motor2_callback, this, _1)
        );

        RCLCPP_INFO(get_logger(), "Nodo MakiControl listo.");
    }

private:

    // ---------- CALLBACKS ----------
    void motor1_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        send_position(1, msg->data);
    }

    void motor2_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        send_position(2, msg->data);
    }

    // ---------- ENVÍO HTTP SIN BLOQUEAR ----------
    void send_position(int id, int pos)
    {
        std::string ip = get_parameter("esp32_ip").as_string();

        std::stringstream path;
        path << "/move?id=" << id << "&pos=" << pos;

        std::stringstream request;
        request << "GET " << path.str() << " HTTP/1.1\r\n";
        request << "Host: " << ip << "\r\n";
        request << "Connection: close\r\n\r\n";

        RCLCPP_INFO(get_logger(), "Enviando HTTP GET → %s%s",
                    ip.c_str(), path.str().c_str());

        // Lanzar en un hilo separado para no bloquear ROS
        std::thread([this, ip, req = request.str()]() {
            try {
                boost::asio::io_context ctx;
                tcp::resolver resolver(ctx);
                tcp::socket socket(ctx);

                auto endpoints = resolver.resolve(ip, "80");
                boost::asio::connect(socket, endpoints);

                boost::asio::write(socket, boost::asio::buffer(req));

                // NO leemos respuesta → envío sin bloqueo
                socket.close();
            }
            catch (std::exception &e) {
                RCLCPP_WARN(this->get_logger(),
                    "HTTP send failed: %s", e.what());
            }
        }).detach();
    }

    // ---------- ATRIBUTOS ----------
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_motor1_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_motor2_;

    boost::asio::io_context io_context_;
    tcp::resolver resolver_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MakiControlNode>());
    rclcpp::shutdown();
    return 0;
}
