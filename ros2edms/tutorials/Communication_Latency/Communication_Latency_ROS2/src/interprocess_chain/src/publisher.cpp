#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/data.hpp"
#include "builtin_interfaces/msg/time.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node") {
        publisher_data_ = this->create_publisher<msgs::msg::Data>("data_topic", 10);
        publisher_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message() {
        auto data_msg = msgs::msg::Data();
        data_msg.data.resize(40000000, 0);

        auto time_msg = builtin_interfaces::msg::Time();
        time_msg = this->get_clock()->now();

        publisher_data_->publish(data_msg);
        publisher_time_->publish(time_msg);
    }

    rclcpp::Publisher<msgs::msg::Data>::SharedPtr publisher_data_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
