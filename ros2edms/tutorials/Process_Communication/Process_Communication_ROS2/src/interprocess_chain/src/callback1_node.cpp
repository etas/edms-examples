#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "builtin_interfaces/msg/time.hpp"

class Callback1Node : public rclcpp::Node {
public:
    Callback1Node() : Node("callback1_node") {
        sub_value_ = this->create_subscription<std_msgs::msg::Int64>(
            "value_topic", 10, std::bind(&Callback1Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic", 10, std::bind(&Callback1Node::forward_time, this, std::placeholders::_1));

        pub_value_ = this->create_publisher<std_msgs::msg::Int64>("value_topic1", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic1", 10);
    }

private:
    void process_message(const std_msgs::msg::Int64::SharedPtr msg) {
        msg->data += 10;
        pub_value_->publish(*msg);
    }

    void forward_time(const builtin_interfaces::msg::Time::SharedPtr msg) {
        pub_time_->publish(*msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_value_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_time_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_value_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Callback1Node>());
    rclcpp::shutdown();
    return 0;
}
