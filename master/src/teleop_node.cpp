#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>

// Fungsi khusus untuk menangkap tombol keyboard tanpa perlu tekan Enter
char getch() {
    char buf = 0;
    struct termios old = {0};
    tcgetattr(0, &old);
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &old);
    read(0, &buf, 1);
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    tcsetattr(0, TCSADRAIN, &old);
    return buf;
}

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_node"), current_pwm_(0), current_steer_(0) {
        cmd_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("cmd_motor", 10);
        
        RCLCPP_INFO(this->get_logger(), "🚗 Teleop Node Aktif!");
        RCLCPP_INFO(this->get_logger(), "Kontrol: [W] Maju | [S] Mundur | [A] Kiri | [D] Kanan | [Spasi] Rem | [Q] Keluar");

        // Timer 20 Hz untuk terus mengirim command ke udp_sender (sesuai ekspektasi Gazebo)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TeleopNode::timer_callback, this));

        // Thread khusus membaca keyboard agar tidak memblokir sistem ROS 2
        key_thread_ = std::thread(&TeleopNode::key_loop, this);
    }

    ~TeleopNode() {
        if (key_thread_.joinable()) key_thread_.join();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread key_thread_;

    int32_t current_pwm_;
    int32_t current_steer_;

    void timer_callback() {
        std_msgs::msg::Int32MultiArray cmd_msg;
        cmd_msg.data = {current_pwm_, current_steer_};
        cmd_pub_->publish(cmd_msg);
    }

    void key_loop() {
        while (rclcpp::ok()) {
            char c = getch();
            if (c == 'w' || c == 'W') {
                current_pwm_ = 150; current_steer_ = 0;
            } else if (c == 's' || c == 'S') {
                current_pwm_ = -150; current_steer_ = 0;
            } else if (c == 'a' || c == 'A') {
                current_pwm_ = 150; current_steer_ = -30;
            } else if (c == 'd' || c == 'D') {
                current_pwm_ = 150; current_steer_ = 30;
            } else if (c == ' ') { 
                current_pwm_ = 0; current_steer_ = 0; // Spasi untuk ngerem
            } else if (c == 'q' || c == 'Q') {
                current_pwm_ = 0; current_steer_ = 0;
                RCLCPP_INFO(this->get_logger(), "Keluar dari Teleop...");
                rclcpp::shutdown();
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
