#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <thread>

// Penerima feedback dari Gazebo
struct Feedback {
    int32_t left_ticks;
    int32_t right_ticks;
    int32_t vel_x_cms;
    int32_t vel_y_cms;
};

// Pengirim perintah ke Gazebo
#pragma pack(push, 1)
struct ControlPacket {
    char header[4];
    int32_t motor_pwm;
    int32_t steering_degree;
    
    ControlPacket() {
        header[0] = 'I'; header[1] = 'R'; header[2] = 'I'; header[3] = 'S';
    }
};
#pragma pack(pop)

class UdpBridgeNode : public rclcpp::Node {
public:
    UdpBridgeNode() : Node("udp_bridge_node") {
        // Setup Socket UDP
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        
        // Bind ke port 12152 untuk menerima data dari Gazebo
        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(12152); // Port dari Changelog
        local_addr.sin_addr.s_addr = INADDR_ANY;
        
        bind(sock_, (struct sockaddr*)&local_addr, sizeof(local_addr));
        
        RCLCPP_INFO(this->get_logger(), "Listening UDP feedback on port 12152...");
    // Inisialisasi Publisher ROS 2
        // Nama topic: "gazebo_feedback", ukuran queue: 10
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("gazebo_feedback", 10);
        // Jalankan thread terpisah untuk terus mendengarkan paket masuk
        receive_thread_ = std::thread(&UdpBridgeNode::receive_loop, this);
    }

    ~UdpBridgeNode() {
        close(sock_);
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    int sock_;
    std::thread receive_thread_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    // Fungsi untuk membaca integer little-endian dari buffer
    int32_t read_i32_le(const uint8_t *src) {
        uint32_t raw;
        std::memcpy(&raw, src, sizeof(raw));
        return static_cast<int32_t>(le32toh(raw));
    }

    void receive_loop() {
        uint8_t buf[64];
        char expected_header[4] = {'I', 'R', 'I', 'S'};

        while (rclcpp::ok()) {
            ssize_t n = recvfrom(sock_, buf, sizeof(buf), 0, nullptr, nullptr);
            if (n >= 20 && std::memcmp(buf, expected_header, 4) == 0) {
                Feedback fb;
                fb.left_ticks = read_i32_le(buf + 4);
                fb.right_ticks = read_i32_le(buf + 8);
                fb.vel_x_cms = read_i32_le(buf + 12);
                fb.vel_y_cms = read_i32_le(buf + 16);

                // Print ke terminal ROS 2
                RCLCPP_INFO(this->get_logger(), "Feedback -> L:%d R:%d vX:%d vY:%d", 
                            fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms);
                
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
                publisher_->publish(msg);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
