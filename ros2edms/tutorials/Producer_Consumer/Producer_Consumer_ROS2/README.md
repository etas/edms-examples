# Set up your first Producer and Consumer application in ROS 2

This guide provides a step-by-step walkthrough for creating a simple Producer and Consumer application in ROS 2.

![Activity Graph of the example](./res_readme/Producer_Consumer_rosgraph.png)

## 1. Creating the Environment

To start, we need to create an ROS 2 workspace for the application by running the following lines in the terminal:

```bash
mkdir -p ~/workspace/src
cd ~/workspace
```

Now initialize the empty workspace by running:

```bash
colcon build
```

## 2. Creating a ROS 2 Package

Now we have to set up a package for our first application. Therefore run the following commands in the terminal:

```bash
cd ~/workspace/src
ros2 pkg create --build-type ament_cmake producer_consumer
```

This should create a package in the **src**-folder with a structure looking like this:

```bash
producer_consumer/
├── CMakeLists.txt
├── package.xml
└── src/
```

## 3. Setting Up the Producer Node

The producer node publishes messages with a counter appended to the message. To set this up you have to create a **producer.cpp** in the **src**-folder of the created package. Therefore run the following command and copy the code into the new file:

```bash
touch ~/workspace/src/producer_consumer/src/producer.cpp
nano ~/workspace/src/producer_consumer/src/producer.cpp
```

**Code:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Producer : public rclcpp::Node {
public:
    Producer() : Node("Producer"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("number_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Producer::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "sending number: %s", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Producer>());
    rclcpp::shutdown();
    return 0;
}
```

**Explanation:**

- **Class Definition:** The `Producer` class inherits from `rclcpp::Node`, which makes it a ROS 2 node. It includes a publisher, a timer, and a counter.
- **Constructor:** `Node("Producer")`: Names the node "Producer."
  - **Publisher:** `create_publisher<std_msgs::msg::String>("number_topic", 10)` creates a publisher that sends messages of type `std_msgs::msg::String` to the topic named `number_topic`. The `10` represents the queue size for storing messages before they're processed.
  - **Timer:** `create_wall_timer(std::chrono::seconds(1), ...)` sets up a timer that triggers the `timer_callback` function every second.
- **Timer Callback:** Creates a message with a string representation of the current counter value. Also it logs the message to the console using `RCLCPP_INFO` and publishes the message to the `number_topic`.
- **Main Function:**
  - **`rclcpp::init(argc, argv)`:** Initializes ROS 2.
  - **`rclcpp::spin`:** Runs the node until it is stopped manually.
  - **`rclcpp::shutdown`:** Cleans up resources when exiting.

Now you can safe the file by pressing **Ctrl + O** and **Enter** and exit `nano` by pressing **Ctrl + X**.

## 4. Setting Up the Consumer Node

After implementing the Producer Node, that sends increasing numbers, we have to implement a Consumer Node, that receives the send messages. To set this up you have to create a **consumer.cpp** in the **src**-folder of the created package. Therefore run the following command and copy the code into the new file:

```bash
touch ~/workspace/src/producer_consumer/src/consumer.cpp
nano ~/workspace/src/producer_consumer/src/consumer.cpp
```

**Code:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Consumer : public rclcpp::Node {
public:
    Consumer() : Node("Consumer") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "number_topic", 10,
            std::bind(&Consumer::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "receiving: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Consumer>());
    rclcpp::shutdown();
    return 0;
}
```

**Explanation:**

- **Class Definition:** The `Consumer` class inherits from `rclcpp::Node`, which makes it a ROS 2 node. It includes a subscription object to listen for messages from the `number_topic`.
- **Constructor:** `Node("Consumer")`: Names the node "Consumer."
  - **Subscription:** `create_subscription<std_msgs::msg::String>("number_topic", 10, ...)` creates a subscription that listens to the topic `number_topic` for messages of type `std_msgs::msg::String`. The `10` represents the queue size for incoming messages. The received messages are passed to the `topic_callback` function.
- **Topic Callback:** The `topic_callback` function processes incoming messages. It logs the content of each message to the console using `RCLCPP_INFO`.
- **Main Function:**
  - **`rclcpp::init(argc, argv)`:** Initializes ROS 2.
  - **`rclcpp::spin`:** Runs the node until it is stopped manually.
  - **`rclcpp::shutdown`:** Cleans up resources when exiting.

Now you can save the file by pressing **Ctrl + O** and **Enter** and exit `nano` by pressing **Ctrl + X**.

## 5. Update the `CMakeLists.txt`

To add the created executables from the Producer and Consumer to your Build-System we have to add some lines into the `CMakeLists.txt`-file of the **src**-folder of our created package. Therefore run the following command and add the code after the first `find_package` line:

```bash
nano ~/workspace/src/producer_consumer/CMakeLists.txt
```

**Code:**

```txt
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(producer src/producer.cpp)
ament_target_dependencies(producer rclcpp std_msgs)

add_executable(consumer src/consumer.cpp)
ament_target_dependencies(consumer rclcpp std_msgs)

# Install targets
install(TARGETS
  producer
  consumer
  DESTINATION lib/${PROJECT_NAME}
)
```

**Explanation:**

- **`find_package(rclcpp REQUIRED)` and `find_package(std_msgs REQUIRED)`:** These lines declare dependencies on the `rclcpp` and `std_msgs` libraries. The build system ensures these libraries are available for your package.
- **`add_executable`:** This command tells the build system to create executables for the `producer.cpp` and `consumer.cpp` source files.
- **`ament_target_dependencies`:** Links the required libraries (`rclcpp` and `std_msgs`) to the corresponding executables.
- **`install`:** Specifies the target executables (`producer` and `consumer`) and defines where they should be placed (`lib/${PROJECT_NAME}`) during installation.

Now you can save the file by pressing **Ctrl + O** and **Enter** and exit `nano` by pressing **Ctrl + X**.

## 6. Update the `package.xml`

Lastly, you need to declare the dependencies in the `package.xml` file of the **producer_consumer** package. This ensures the build environment includes the required libraries. Run the following command and add the lines after `<buildtool_depend>ament_cmake</buildtool_depend>`:

```bash
nano ~/workspace/src/producer_consumer/package.xml
```

**Code:**

```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
```

**Explanation:**

- **`<depend>rclcpp</depend>`:** Specifies a dependency on the `rclcpp` library, which is used for ROS 2 node creation and interaction.
- **`<depend>std_msgs</depend>`:** Specifies a dependency on the `std_msgs` library, which provides the standard message types used in the application.

Now you can save the file by pressing **Ctrl + O** and **Enter** and exit `nano` by pressing **Ctrl + X**.

## 7. Build the package

With the `CMakeLists.txt` and `package.xml` updated, you can now build the package. Follow these steps:

1. Navigate to the root of your workspace:

```bash
cd ~/workspace
```

2. Build the package using `colcon`:

```bash
colcon build --packages-select producer_consumer
```

3. Source the setup script to make the package available:

```bash
source install/setup.bash
```

## 8. Run the Application

Now you can run both the Producer and Consumer nodes to test your application.

1. Open two terminals.

2. In the first terminal, run the Producer node:

```bash
ros2 run producer_consumer producer
```

3. In the second terminal, run the Consumer node:

```bash
ros2 run producer_consumer consumer
```

You should see the Producer publishing messages like:

```less
[INFO] [Producer]: sending number: 0
[INFO] [Producer]: sending number: 1
```

And the Consumer receiving these messages:

```less
[INFO] [Consumer]: receiving: 0
[INFO] [Consumer]: receiving: 1
```

Congratulations this completes the setup and testing of your Producer-Consumer application in ROS 2.
