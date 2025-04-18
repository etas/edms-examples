#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/data.hpp"
#include "builtin_interfaces/msg/time.hpp"

class Callback2Node : public rclcpp::Node {
public:
    Callback2Node() : Node("callback2_node") {
        sub_data_ = this->create_subscription<msgs::msg::Data>(
            "data_topic1", 10, std::bind(&Callback2Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic1", 10, std::bind(&Callback2Node::forward_time, this, std::placeholders::_1));

        pub_data_ = this->create_publisher<msgs::msg::Data>("data_topic2", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic2", 10);
    }

private:
    void process_message(const msgs::msg::Data::SharedPtr msg) {
        pub_data_->publish(*msg);
    }

    void forward_time(const builtin_interfaces::msg::Time::SharedPtr msg) {
        pub_time_->publish(*msg);
    }

    rclcpp::Subscription<msgs::msg::Data>::SharedPtr sub_data_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_time_;
    rclcpp::Publisher<msgs::msg::Data>::SharedPtr pub_data_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr pub_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Callback2Node>());
    rclcpp::shutdown();
    return 0;
}
