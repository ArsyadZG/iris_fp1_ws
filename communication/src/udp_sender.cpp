#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

namespace {
    constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
    constexpr size_t kPacketSize = 12;

    void build_packet(uint8_t (&buf)[kPacketSize], int32_t pwm, int32_t steer) {
        pwm = std::clamp<int32_t>(pwm, -255, 255);
        uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm));
        uint32_t steer_le = htole32(static_cast<uint32_t>(steer));

        std::memcpy(buf, kHeader, sizeof(kHeader));
        std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
        std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
    }
} // namespace

// BUNGKUSAN ROS 2
class UdpSenderNode : public rclcpp::Node {
public:
    UdpSenderNode() : Node("udp_sender") {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(static_cast<uint16_t>(12151)); // Port default simulasi
        inet_pton(AF_INET, "127.0.0.1", &addr_.sin_addr);

        // Subscribe perintah gerak dari package master (isi array: [pwm, steer])
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "cmd_motor", 10, std::bind(&UdpSenderNode::command_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "UDP Sender siap kirim ke port 12151");
    }

    ~UdpSenderNode() {
        // Stop mobil saat node dimatikan
        uint8_t packet[kPacketSize];
        build_packet(packet, 0, 0); 
        sendto(sock_, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
        close(sock_);
    }

private:
    int sock_;
    sockaddr_in addr_{};
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

    void command_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            uint8_t packet[kPacketSize];
            // Pakai fungsi panitia untuk menyusun paket
            build_packet(packet, msg->data[0], msg->data[1]); 
            sendto(sock_, packet, sizeof(packet), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
            RCLCPP_INFO(this->get_logger(), "Sending pwm=%d steer=%d", msg->data[0], msg->data[1]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpSenderNode>());
    rclcpp::shutdown();
    return 0;
}
