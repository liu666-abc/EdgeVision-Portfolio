// C++ subscriber probe: measures publish -> receive transport latency of
// /evi/detections and writes a CSV. Read-only; no chassis, no /cmd_vel.

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "evi_cpp_detector/msg/detection_array.hpp"
#include "rclcpp/rclcpp.hpp"

class LatencyProbe : public rclcpp::Node {
public:
    LatencyProbe() : Node("latency_probe") {
        csv_path_ = declare_parameter<std::string>("csv_path", "latency.csv");
        max_messages_ = declare_parameter<int>("max_messages", 2000);
        csv_.open(csv_path_, std::ios::trunc);
        csv_ << "seq,detections,transport_ms\n";
        subscription_ = create_subscription<evi_cpp_detector::msg::DetectionArray>(
            "/evi/detections", 10,
            [this](evi_cpp_detector::msg::DetectionArray::SharedPtr message) {
                const auto arrival = std::chrono::system_clock::now();
                const int64_t stamp_ns =
                    static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
                    static_cast<int64_t>(message->header.stamp.nanosec);
                const int64_t arrival_ns =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        arrival.time_since_epoch()).count();
                const double transport_ms =
                    static_cast<double>(arrival_ns - stamp_ns) / 1e6;
                ++count_;
                csv_ << count_ << "," << message->detections.size() << ","
                     << transport_ms << "\n";
                csv_.flush();
                if (count_ >= max_messages_) {
                    RCLCPP_INFO(get_logger(), "received %d messages", count_);
                    csv_.close();
                    rclcpp::shutdown();
                }
            });
        RCLCPP_INFO(get_logger(), "latency_probe listening -> %s", csv_path_.c_str());
    }

private:
    std::string csv_path_;
    int max_messages_ = 2000;
    int count_ = 0;
    std::ofstream csv_;
    rclcpp::Subscription<evi_cpp_detector::msg::DetectionArray>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatencyProbe>());
    rclcpp::shutdown();
    return 0;
}
