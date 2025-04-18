#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <vector>

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node"), latency_values_(60, 0.0), count_(0) {
        sub_value_ = this->create_subscription<std_msgs::msg::Int64>(
            "final_value_topic", 10, std::bind(&SubscriberNode::receive_value, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "final_time_topic", 10, std::bind(&SubscriberNode::receive_time, this, std::placeholders::_1));

        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    }

private:
    void receive_value(const std_msgs::msg::Int64::SharedPtr msg) {
        last_value_ = msg->data;
    }

    void receive_time(const builtin_interfaces::msg::Time::SharedPtr msg) {
        auto now = clock_->now();  // SYSTEM_TIME verwenden
        rclcpp::Time sent_time(msg->sec, msg->nanosec, RCL_SYSTEM_TIME);  

        double latency_ms = (now - sent_time).seconds() * 1000.0;
        latency_values_[count_ % 60] = latency_ms;
        count_++;

        if (count_ >= 60) {
            double sum = 0.0;
            for (double latency : latency_values_) {
                sum += latency;
            }
            double avg_latency = sum / 60.0;
            RCLCPP_INFO(this->get_logger(), "Average latency over the last 60 samples: %.2f ms", latency_ms);
            count_ = 0; // Reset counter after computing average
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_value_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_time_;
    int64_t last_value_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<double> latency_values_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
