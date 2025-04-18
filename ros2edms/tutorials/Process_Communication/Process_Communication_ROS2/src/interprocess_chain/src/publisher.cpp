#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "builtin_interfaces/msg/time.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node"), count_(0) {
        publisher_value_ = this->create_publisher<std_msgs::msg::Int64>("value_topic", 10);
        publisher_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message() {
        auto value_msg = std_msgs::msg::Int64();
        auto time_msg = builtin_interfaces::msg::Time();
        value_msg.data = count_++;
        time_msg = this->get_clock()->now();

        publisher_value_->publish(value_msg);
        publisher_time_->publish(time_msg);
    }

    int64_t count_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_value_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
