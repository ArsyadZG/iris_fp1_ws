#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> // <-- Tambahan: Header LiDAR
#include <cmath>
#include <limits>
#include <algorithm>

class MasterNode : public rclcpp::Node {
public:
    MasterNode() : Node("master_node") {
        cmd_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("cmd_motor", 10);
        
        feedback_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "gazebo_feedback", 10, std::bind(&MasterNode::feedback_callback, this, std::placeholders::_1));
        
        vision_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/vision/target_point", 10, std::bind(&MasterNode::vision_callback, this, std::placeholders::_1));

        // <-- Tambahan: Subscriber untuk topik LiDAR Gazebo
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/catvehicle/front_laser_points", 10, std::bind(&MasterNode::lidar_callback, this, std::placeholders::_1));

        // Catat waktu pertama kali node menyala untuk perhitungan dt
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Master Node aktif dengan Kinematika Ackermann dan LiDAR!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_; // <-- Tambahan

    const double Phi = std::acos(-1);
    double l = 0.3;  // Panjang Wheelbase
    double x = 0.0;
    double y = 0.0;
    double theta_rad = 0.0;
    
    int32_t last_steer_deg = 0;  
    rclcpp::Time last_time_;     
    
    bool is_reversing_ = false; 

    // Mengukur dan Melacak Posisi (Odom) dari Gazebo [TIDAK DIUBAH]
    void feedback_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 4) {
            double v = msg->data[2] / 100.0;  
            
            rclcpp::Time current_time = this->now();
            double dt = (current_time - last_time_).seconds();
            last_time_ = current_time;

            double rad = last_steer_deg * Phi / 180.0;  
            x += v * std::cos(theta_rad) * dt;  
            y += v * std::sin(theta_rad) * dt;
            
            double w = (v * std::tan(rad)) / l;
            theta_rad += w * dt;

            double theta_deg = theta_rad * 180.0 / Phi;
            while (theta_deg >= 360.0) theta_deg -= 360.0;
            while (theta_deg < 0.0) theta_deg += 360.0;

            RCLCPP_INFO(this->get_logger(), "Odom Tracker -> X: %.3f, Y: %.3f, Theta: %.3f deg", x, y, theta_deg);
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) return;

        int center_idx = static_cast<int>(msg->ranges.size() / 2);
        int spread = 30;

        // Pastikan indeks tidak keluar dari batas array
        int start_idx = std::max(0, center_idx - spread);
        int end_idx = std::min(
            static_cast<int>(msg->ranges.size()) - 1,
            center_idx + spread
        );

        double min_dist = std::numeric_limits<double>::infinity();

        // Cari jarak terdekat dalam area depan kendaraan
        for (int i = start_idx; i <= end_idx; ++i) {
            double r = msg->ranges[i];

            if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
                min_dist = std::min(min_dist, r);
            }
        }
        
        // Aktifkan mode mundur jika ada objek dalam jarak 2 meter
        if (min_dist < 2.0) {
            if (!is_reversing_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Obstacle detected: %.2f m. Reversing!",
                    min_dist
                );
            }
            is_reversing_ = true;
        }
        // Kembali maju jika objek sudah lebih dari 2.5 meter
        else if (min_dist > 2.5) {
            if (is_reversing_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Path clear: %.2f m. Moving forward.",
                    min_dist
                );
            }
            is_reversing_ = false;
        }
    }

    // Menentukan Sudut Setir Berdasarkan Target Visi [DIMODIFIKASI SEDIKIT UNTUK VETO LIDAR]
    void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        int32_t calc_pwm = 150;
        int32_t calc_steer = 0;

        // VETO LIDAR: Jika terdeteksi halangan, paksa mundur
        if (is_reversing_) {
            calc_pwm = -150; // Mundur
            calc_steer = 30;  // Banting setir ke kanan
        }  
        else {
            // MODE NORMAL: MAJU DAN TRACKING JALUR [TIDAK DIUBAH]
            double target_x = msg->x;
            double target_y = msg->y;
            double target_angle_rad = std::atan2(target_y, target_x);
            
            double Kp = 0.7;  
            calc_steer = static_cast<int32_t>((target_angle_rad * 180.0 / Phi) * Kp);
            calc_steer = std::clamp(calc_steer, -30, 30);

            calc_pwm = 150;
            if (std::abs(calc_steer) > 15) {
                calc_pwm = 100;
            } else if (std::abs(calc_steer) > 5) {
                calc_pwm = 120;
            }
        }

        last_steer_deg = calc_steer;

        // Kirim perintah ke roda
        std_msgs::msg::Int32MultiArray cmd_msg;
        cmd_msg.data = {calc_pwm, calc_steer};
        cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}