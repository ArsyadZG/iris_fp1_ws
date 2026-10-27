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
    
    //next feedback dan juga vision
    /*
    #include <iostream>
    #include <cmath>
    #include <cstdio>
    using namespace std;


    in t main() {
    double l;
    cin >> l;
    double N;
    cin >> N;
    double x = 0;
    double y = 0;
    double theta_rad = 0;
    const double Phi = acos(-1);
    
    for(int i = 0; i < N; i++){
        double v, deg, dt;
        cin >> v >> deg >> dt;
        double rad = deg*Phi/180; 
        x+=v*cos(theta_rad)*dt; //cos old*delta s[alias (v*delta t)]
        y+= v*sin(theta_rad)*dt;
        double w = (v*tan(rad))/l;
        theta_rad += w*dt;
    } 
    
    double theta_deg = theta_rad*180/Phi;
    if (theta_deg>360){
        theta_deg -= 360;
    }
    if (theta_deg<0){
        theta_deg += 360;
    }
    printf("%.4f %.4f %.4f\n", x, y, theta_deg);
    
    return 0;
    }

    */
    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MasterNode>());
        rclcpp::shutdown();
        return 0;
}
