#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "builtin_interfaces/msg/time.hpp"

// Publisher-Node: Sendet eine Zahl und den aktuellen Zeitstempel
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

// Callback1-Node: Erhöht die Zahl um 10
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

// Callback2-Node: Multipliziert die Zahl mit 2
class Callback2Node : public rclcpp::Node {
public:
    Callback2Node() : Node("callback2_node") {
        sub_value_ = this->create_subscription<std_msgs::msg::Int64>(
            "value_topic1", 10, std::bind(&Callback2Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic1", 10, std::bind(&Callback2Node::forward_time, this, std::placeholders::_1));

        pub_value_ = this->create_publisher<std_msgs::msg::Int64>("value_topic2", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("time_topic2", 10);
    }

private:
    void process_message(const std_msgs::msg::Int64::SharedPtr msg) {
        msg->data *= 2;
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

// Callback3-Node: Subtrahiert 5
class Callback3Node : public rclcpp::Node {
public:
    Callback3Node() : Node("callback3_node") {
        sub_value_ = this->create_subscription<std_msgs::msg::Int64>(
            "value_topic2", 10, std::bind(&Callback3Node::process_message, this, std::placeholders::_1));
        sub_time_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "time_topic2", 10, std::bind(&Callback3Node::forward_time, this, std::placeholders::_1));

        pub_value_ = this->create_publisher<std_msgs::msg::Int64>("final_value_topic", 10);
        pub_time_ = this->create_publisher<builtin_interfaces::msg::Time>("final_time_topic", 10);
    }

private:
    void process_message(const std_msgs::msg::Int64::SharedPtr msg) {
        msg->data -= 5;
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

// Subscriber-Node: Berechnet die Latenz in Millisekunden und gibt das Endergebnis aus
class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node"), count_(0), latency_values_(60, 0.0) {
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
        auto now = clock_->now();  
        rclcpp::Time sent_time(msg->sec, msg->nanosec, RCL_SYSTEM_TIME);  

        double latency_ms = (now - sent_time).seconds() * 1000.0;
        latency_values_[count_ % 60] = latency_ms;
        count_++;

        // Warte, bis genau 60 Werte empfangen wurden
        if (count_ == 60) {
            double sum = 0.0;
            for (double latency : latency_values_) {
                sum += latency;
            }
            double avg_latency = sum / 60.0;
            RCLCPP_INFO(this->get_logger(), "Average latency over the last 60 samples: %.2f ms", avg_latency);
            
            // Zähler zurücksetzen, um neue 60 Samples zu sammeln
            count_ = 0; 
        }
    }

    int64_t last_value_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_value_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr sub_time_;
    
    rclcpp::Clock::SharedPtr clock_;
    std::vector<double> latency_values_;
    size_t count_;
};

// Main-Funktion: Startet alle Nodes in einem einzigen Prozess
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto publisher = std::make_shared<PublisherNode>();
    auto callback1 = std::make_shared<Callback1Node>();
    auto callback2 = std::make_shared<Callback2Node>();
    auto callback3 = std::make_shared<Callback3Node>();
    auto subscriber = std::make_shared<SubscriberNode>();

    executor->add_node(publisher);
    executor->add_node(callback1);
    executor->add_node(callback2);
    executor->add_node(callback3);
    executor->add_node(subscriber);

    executor->spin();
    rclcpp::shutdown();
    return 0;
}
