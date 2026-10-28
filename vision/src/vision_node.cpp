#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        // Publisher koordinat target ke master_node
        target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/target_point", 10);
        // Subscriber ke topic kamera Gazebo
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/catvehicle/camera_front/image_raw_front", 10,
        std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Vision Node aktif. Mendengarkan /camera/image_raw_front...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat hsv, mask;

       // Konversi BGR ke HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // Revisi: Range warna HSV untuk mendeteksi warna PUTIH (Saturasi mendekati 0, Value tinggi)
        cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 40, 255), mask);
        int h = frame.rows;
        int w = frame.cols;
        // Potong ROI menjadi garis tipis di depan mobil saja agar beloknya responsif 
        cv::Mat roi = mask(cv::Rect(0, static_cast<int>(h * 0.6), w, static_cast<int>(h * 0.2)));
        // Hitung centroid (titik tengah piksel jalur)
        cv::Moments m = cv::moments(roi, true);
        geometry_msgs::msg::Point target_pt;

        if (m.m00 > 0) {
            double cx = (m.m10 / m.m00);
            double center_offset = cx - (w / 2.0); // Deviasi dari tengah frame
          
            target_pt.x = 1.0; // Jarak pandang ke depan (1 meter)
            target_pt.y = center_offset / (w / 2.0); // Offset kemudi ter-normalisasi (-1.0 s/d 1.0)
        } else {
            // Kalau jalur nggak terdeteksi, default jalan lurus
            target_pt.x = 1.0;
            target_pt.y = 0.0;
        }

        target_pub_->publish(target_pt);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
