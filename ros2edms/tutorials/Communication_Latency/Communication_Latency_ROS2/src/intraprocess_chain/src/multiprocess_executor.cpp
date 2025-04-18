#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/data.hpp"
#include "builtin_interfaces/msg/time.hpp"

// Publisher-Node: Sendet eine 10 MB Nachricht mit Zeitstempel
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

// Callback1-Node: Leitet die Nachricht weiter
class Callback1Node : public rclcpp::Node {
public:
    Callback1Node() : Node("callback1_node") {
        sub_data_ = this->create_subscription<msgs::msg::Data>(
            "data_topic", 10, std::bind(&Callback1Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic", 10, std::bind(&Callback1Node::forward_time, this, std::placeholders::_1));

        pub_data_ = this->create_publisher<msgs::msg::Data>("data_topic1", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic1", 10);
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

// Callback2-Node: Leitet die Nachricht weiter
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

// Callback3-Node: Leitet die Nachricht weiter
class Callback3Node : public rclcpp::Node {
public:
    Callback3Node() : Node("callback3_node") {
        sub_data_ = this->create_subscription<msgs::msg::Data>(
            "data_topic2", 10, std::bind(&Callback3Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic2", 10, std::bind(&Callback3Node::forward_time, this, std::placeholders::_1));

        pub_data_ = this->create_publisher<msgs::msg::Data>("final_data_topic", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("final_time_topic", 10);
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

// Subscriber-Node: Berechnet die Latenz
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
            RCLCPP_INFO(this->get_logger(), "Average latency over the last 60 samples: %.2f ms", avg_latency);
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
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto publisher = std::make_shared<PublisherNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "PublisherNode created");

    auto callback1 = std::make_shared<Callback1Node>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Callback1Node created");

    auto callback2 = std::make_shared<Callback2Node>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Callback2Node created");

    auto callback3 = std::make_shared<Callback3Node>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Callback3Node created");

    auto subscriber = std::make_shared<SubscriberNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "SubscriberNode created");

    executor->add_node(publisher);
    executor->add_node(callback1);
    executor->add_node(callback2);
    executor->add_node(callback3);
    executor->add_node(subscriber);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting executor...");
    executor->spin();
    rclcpp::shutdown();

    return 0;
}
