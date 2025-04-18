# Software Architecture of ROS 2 and EDMS

### ROS 2 Software Architecture

ROS 2 (Robot Operating System 2) provides a flexible and modular software architecture designed for robotic applications. Its core architecture follows a distributed model where different nodes communicate via a publish-subscribe mechanism. The key components of ROS 2’s middleware abstraction include:

- **DDS (Data Distribution Service)**: ROS 2 relies on DDS as its default middleware, allowing nodes to communicate efficiently over various network topologies. DDS ensures Quality of Service (QoS) control, real-time communication, and fault tolerance.
- **Middleware Abstraction Layer (RMW)**: ROS 2 introduces an abstraction layer known as the ROS Middleware (RMW) interface. This enables support for multiple DDS implementations, allowing developers to choose the most suitable one for their application.
- **Execution Management**: The executor is responsible to handle in-coming data and calling the corresponding callback functions.
  - **Deployment Configuration**: ROS 2 does not provide concepts for real-time scheduling configuration. However, multi-threading can be activated by using so-called a MultiThreadedExecutor without any further configuration. Additional operating system functions need to be called to define further scheduling and real-time parameters, such as process priority and core assignment.
- **Launch System**: ROS 2 employs a YAML/XML-based launch system to configure and initialize nodes dynamically.

### EDMS Software Architecture

EDMS takes a different approach to middleware abstraction by leveraging **YAAA (YAml As Architecture)** and the **Middleware**. This system is specifically designed for automotive and safety-critical applications.

- **YAAA (YAml As Architecture)**: YAAA is a YAML-based configuration and architecture description language that defines the software components, the communication patterns, and the deployment configuration.
  - **Functional Components**: YAAA models essential software blocks such as Runnables, Activities, and Gateways.
  - **Communication**: Defines message passing and data interfaces between components.
  - **Execution Management**: Components in EDMS are structured in Activities. Inside Activities one or multiple Runnables can be defined, representing functional units (so-called Business-Logics). All Runnables inside an Activity must form a Directed-Acyclic-Graph. The Activity has a trigger condition that defines the activation of the Activity. It can be data-triggered (e.g. based on some input data) or time-triggered (e.g. periodic activation). When the trigger condition is met, the input data for all Runnables of the Activity is "frozen" in an inbox. During the entire processing of all runnables the input data seen by these runnables does not change until the next Activity activation. This enables deterministic processing of an Activity.
  - **Deployment Configuration**: Specifies operating system and middleware parameters of the target architecture. For example: number of threads, thread priority, and core assignments.
- **Middleware Abstraction Layer (MWALA)**: MWALA abstracts the underlying communication framework from business logic (e.g., Runnables), enabling portability across different middleware implementations.
- **Activity Distributed Activation Management (ADAM)**: A library for managing activation logic separately from inter-process communication, improving system latency.
- **RouDi (Routing and Discovery)**: A crucial component of the middleware that sets up and manages shared memory for inter-process communication.

### Key Feature Comparison

| Feature                    | ROS 2                                                                                         | EDMS (YAAA + Middleware)                                                                                                                             |
| -------------------------- | --------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Middleware Abstraction** | RMW layer for DDS-based middleware abstraction                                                | MWALA for abstracting middleware from business logic                                                                                                 |
| **Configuration Approach** | XML/YAML-based launch files for node configuration, direct implementation in ROS 2 (no model) | YAAA (YAML-based architecture description) model                                                                                                     |
| **Communication Model**    | DDS-based publish-subscribe mechanism                                                         | RouDi-managed shared memory and Inter-Process Communication, publish-and-subscribe mechanism, zero-copy                                              |
| **Execution Management**   | Executor model for scheduling and callback handling                                           | ADAM library for distributed activation, Directed-Acyclic-Graph(DAG) scheduling (data-driven) within an Activity, time-stamped input data visibility |
| **Deployment**             | basic, multi-threading enable/disable                                                         | advanced scheduling configuration possible (process priority, core-assignment)                                                                       |
| **Target Applications**    | Robotics, autonomous systems                                                                  | Automotive, safety-critical applications                                                                                                             |

### Conclusion

While both ROS 2 and EDMS provide middleware abstraction layers, their approaches differ significantly. ROS 2 emphasizes flexibility with DDS-based communication with an Executor, making it suitable for robotics applications. EDMS, on the other hand, employs YAAA for software modeling and MWALA for middleware abstraction, making it a robust choice for automotive applications where shared memory and deterministic behavior are crucial.

## References

- [ROS 2 Middleware Abstraction Layer (RMW)](https://design.ros2.org/articles/ros_middleware_interface.html)
- [YAAA (YAml As Architecture)](https://edms.etas.com/yaaa_modeling_concepts_and_workflow.html)
- [Middleware Abstraction Layer (MWALA)](https://edms.etas.com/explanations/mwala.html)
