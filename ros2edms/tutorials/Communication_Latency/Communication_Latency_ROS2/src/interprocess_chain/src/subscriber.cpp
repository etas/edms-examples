#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/data.hpp"
#include "builtin_interfaces/msg/time.hpp"

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node"), count_(0), latency_values_(60, 0.0) {
        sub_data_ = this->create_subscription<msgs::msg::Data>(
            "final_data_topic", 10, std::bind(&SubscriberNode::receive_data, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "final_time_topic", 10, std::bind(&SubscriberNode::receive_time, this, std::placeholders::_1));

        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    }

private:
    void receive_data(const msgs::msg::Data::SharedPtr msg) {
        last_data_size_ = msg->data.size();
    }

    void receive_time(const builtin_interfaces::msg::Time::SharedPtr msg) {
        auto now = clock_->now();
        rclcpp::Time sent_time(msg->sec, msg->nanosec, RCL_SYSTEM_TIME);

        double latency_ms = (now - sent_time).seconds() * 1000.0;
        latency_values_[count_ % 60] = latency_ms;
        count_++;

        if (count_ == 60) {
            double sum = 0.0;
            for (double latency : latency_values_) {
                sum += latency;
            }
            double avg_latency = sum / 60.0;
            RCLCPP_INFO(this->get_logger(), "Average latency over last 60 samples: %.2f ms", avg_latency);
            count_ = 0;
        }
    }

    size_t last_data_size_;
    rclcpp::Subscription<msgs::msg::Data>::SharedPtr sub_data_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_time_;

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
