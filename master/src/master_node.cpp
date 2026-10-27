#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <algorithm>

class MasterNode : public rclcpp::Node {
public:
    MasterNode() : Node("master_node") {
        cmd_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("cmd_motor", 10);
        
        feedback_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "gazebo_feedback", 10, std::bind(&MasterNode::feedback_callback, this, std::placeholders::_1));
        
        vision_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/vision/target_point", 10, std::bind(&MasterNode::vision_callback, this, std::placeholders::_1));

        // Catat waktu pertama kali node menyala untuk perhitungan dt
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Master Node aktif dengan Kinematika Ackermann!");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vision_sub_;

    const double Phi = std:: acos(-1);
    double l = 0.3; //Panjang wheelbase, asumsi 30cm
    double x = 0.0;
    double y = 0.0;
    double theta_rad = 0.0;

    int32_t last_steer_deg = 0; //menyimpan setir terakhir yang dieksekusi
    rclcpp:Time last_time_; //Untuk menghitung dt
    
    //Mengukur dan track posiis dari Gazebo
    void feedback_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)){
        if(msg->data.size() >=4){
            //ubah kecepatan dari cm.s jadi m/s
            double v= msg->data[2]/100.0;
            //hitung dt(selisih waktu nyata ROS2)
            rclcpp::Time current_time = this->now();
            double dt = (current_time - last_time_).seconds();
            last_time = current_time;

            //Ackerman
            double rad = last_steer_deg*Phi/100.0;
            x+= v*std::cos(theta_rad)*dt;
            y+= v*std::sin(theta_rad)*dt;
            double w = v*std::tan(rad)/l;
            theta_rad += w*dt;

            //Normalisasi derajat
            double theta_deg = theta_rad *180.0/Phi;
            while (theta_deg >=360.0) theta_deg -= 360.0;
            while (theta_deg < 0.0) theta_deg += 360.0;

            RCLCPP_INFO(this->get_logger(), "Tracker -> X: %.3f, Y: %.3f, Theta: %.3f deg", x, y, theta_deg);
        }
    }
    // Menentukan Sudut Setir Berdasarkan Target 
    void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Asumsi dari kamera: 
        // x = seberapa jauh jalan di depan
        // y = seberapa menyimpang jalur ke kiri/kanan
        double target_x = msg->x;
        double target_y = msg->y;

        // Reverse Kinematics
        // Cari sudut belok agar moncong mobil mengarah lurus ke target_y
        double target_angle_rad = std::atan2(target_y, target_x);
        // Konversi ke derajat untuk dikirim ke servo Gazebo
        int32_t calc_steer = static_cast<int32_t>(target_angle_rad * 180.0 / Phi);
        // Batasi sudut setir fisik agar servo tidak patah (misal -30 hingga 30 derajat)
        calc_steer = std::clamp(calc_steer, -30, 30);
        // Kecepatan konstan
        int32_t calc_pwm = 150; 
        // Update memori setir terakhir agar bisa diukur oleh rumus Ackermann di atas
        last_steer_deg = calc_steer;

        // Kirim perintah
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
