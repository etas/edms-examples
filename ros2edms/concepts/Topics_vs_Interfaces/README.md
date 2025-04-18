# Communication patterns

## Topics in ROS 2

In `ROS 2`, `topics` are a core component of the communication model and are used in the `publish-subscribe` paradigm. A `topic` is a named channel through which nodes can send and receive messages asynchronously. Nodes that publish messages on a topic do not need to know the subscribers of that topic, and vice versa. This decoupling of the sender and receiver is one of the main features of ROS 2, enabling modular, flexible, and scalable system design.

In ROS 2, topics are used for broadcasting messages to one or more subscribers, and messages can be of any type that conforms to the ROS 2 message definitions. A `publisher` node sends data to a specific topic, while a `subscriber` node listens to a topic and processes the data it receives. This system is particularly effective for continuous or real-time data such as sensor readings, status updates, or command broadcasts.

## Interfaces in EDMS

`EDMS` takes a different approach with its concept of `interfaces`. In `EDMS`, interfaces define the structure and type of data that can be exchanged between system components, but they do not carry the same semantics as ROS 2 topics. Interfaces in EDMS are more tightly coupled to the `memory layout` and `data structure`, typically defined at compile time with `C++ templates`. These interfaces enable the `communication` between different system components, but they often require a more explicit connection in the system architecture.

For example, in EDMS, `templated interfaces` allow for greater performance optimization by leveraging compile-time configuration to define the structure of the data being exchanged. Interfaces are often `grouped` into `interface groups`, which bundle related interfaces into a single coherent group, improving the readability and organization of the system. However, unlike ROS 2 topics, EDMS interfaces are more constrained in terms of how they can be used across different systems or processes, as they rely on `shared memory` and `target compiler-specific memory layouts` for high-performance communication.

## Key Feature Comparison

Here’s a comparison of the key features between `Topics` in ROS 2 and `Interfaces` in EDMS:

- **Decoupling:**
  - **ROS 2 Topics:** Topics enable `loose coupling` between producers and consumers of data. Nodes do not need to be aware of each other beyond the topic they subscribe to or publish on.
  - **EDMS Interfaces:** Interfaces are more `tightly coupled` and often designed to optimize performance at the hardware level. The communication mechanism is more explicit, with developers defining how and when data is exchanged.

- **Communication Model:**
  - **ROS 2 Topics:** Topics are part of the `publish-subscribe` model, which supports asynchronous communication where publishers and subscribers are independent entities.
  - **EDMS Interfaces:** Interfaces are used to define `data structures` for communication, and their interactions are more directly tied to the system architecture, requiring careful management of memory and serialization.

- **Flexibility:**
  - **ROS 2 Topics:** Topics provide `high flexibility` in terms of message types and how components interact with each other. Since ROS 2 topics are based on DDS (Data Distribution Service), they allow for seamless communication between different hardware and network layers.
  - **EDMS Interfaces:** EDMS interfaces are `optimized for specific system configurations` and require developers to define the data structure explicitly at compile-time. These interfaces are suited for high-performance embedded systems where low-latency and efficient memory usage are crucial.

- **Performance:**
  - **ROS 2 Topics:** While topics in ROS 2 allow for a flexible communication pattern, they may not be as optimized for real-time performance as interfaces in EDMS.
  - **EDMS Interfaces:** EDMS interfaces are designed for `high-performance embedded systems`, where the system’s `memory layout` and `data serialization` can be tightly controlled to minimize overhead.

## Key Benefits of ROS 2 Topics

- **Loose Coupling:** Allows easy communication between independent nodes without knowledge of each other.
- **Scalability:** Topics can be subscribed to by multiple nodes, supporting distributed systems.
- **Ease of Use:** The ROS 2 ecosystem provides simple tools for handling topics, and message types are easily extended.
- **Flexible Communication:** Topics support asynchronous, real-time data exchange in both homogeneous and heterogeneous systems.

## Key Benefits of EDMS Interfaces

- **Performance Optimization:** EDMS interfaces enable fine-grained control over memory and communication performance, crucial for embedded systems.
- **Tight Integration:** Interfaces can be tightly integrated with hardware and system architecture for optimal performance.
- **Compile-time Configuration:** EDMS allows data structures to be defined at compile time, ensuring that the system is tailored for the target environment.

## References

- [ROS 2 Topics](https://docs.ros.org/en/rolling/Concepts/Basic/About-Topics.html)
- Refer to **Interfaces** in the EDMS User Documentation
