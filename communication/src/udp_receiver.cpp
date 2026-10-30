#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <thread>

namespace {
    constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    constexpr size_t kPacketSize = 20;

    struct Feedback {
        int32_t left_ticks; int32_t right_ticks;
        int32_t vel_x_cms; int32_t vel_y_cms;
    };

    int32_t read_i32_le(const uint8_t *src) {
        uint32_t raw;
        std::memcpy(&raw, src, sizeof(raw));
        return static_cast<int32_t>(le32toh(raw));
    }

    bool parse_packet(const uint8_t *buf, size_t len, Feedback &out) {
        if (len < kPacketSize || std::memcmp(buf, kHeader, sizeof(kHeader)) != 0) return false;
        out.left_ticks = read_i32_le(buf + 4);
        out.right_ticks = read_i32_le(buf + 8);
        out.vel_x_cms = read_i32_le(buf + 12);
        out.vel_y_cms = read_i32_le(buf + 16);
        return true;
    }
} // namespace

// BUNGKUSAN ROS 2
class UdpReceiverNode : public rclcpp::Node {
public:
    UdpReceiverNode() : Node("udp_receiver") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("gazebo_feedback", 10);
        
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(12152));
        inet_pton(AF_INET, "10.144.174.111", &addr.sin_addr);
        bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));

        // Agar tidak stuck saat dimatikan 
        timeval tv{1, 0};
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(), "Listening UDP feedback on port 12152");
        receive_thread_ = std::thread(&UdpReceiverNode::receive_loop, this);
    }

    ~UdpReceiverNode() {
        close(sock_);
        if (receive_thread_.joinable()) receive_thread_.join();
    }

private:
    int sock_;
    std::thread receive_thread_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

    void receive_loop() {
        uint8_t buf[64];
        while (rclcpp::ok()) {
            ssize_t n = recvfrom(sock_, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n < 0) continue;

            Feedback fb;
            // Memanggil Fungsi Parse
            if (parse_packet(buf, static_cast<size_t>(n), fb)) {
                RCLCPP_INFO(this->get_logger(), "enc L=%d R=%d vel x=%d y=%d", fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms);
                
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
                publisher_->publish(msg);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiverNode>());
    rclcpp::shutdown();
    return 0;
}
