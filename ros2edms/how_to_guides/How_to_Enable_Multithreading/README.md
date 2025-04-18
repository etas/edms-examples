# How to Enable Multithreading?

This guide demonstrates how to enable multithreading in both **ROS 2** and **EDMS** middleware. We'll explain the process for both systems and how they manage threads for concurrent execution.

## ROS 2

In ROS 2, enabling multithreading is typically done by using the **MultiThreaded Executor**. ROS 2's executor is responsible for managing and executing nodes in a multi-threaded environment. The MultiThreaded Executor provides automatic thread management for running nodes concurrently, making it simple to achieve parallelism across different parts of your ROS 2 application.

### Steps for Enabling Multithreading in ROS 2

1. **Define the Executor Type**: ROS 2 allows you to use a `MultiThreadedExecutor` instead of the default `SingleThreadedExecutor`.

   Example code:

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   int main(int argc, char *argv[]) {
       rclcpp::init(argc, argv);
       
       rclcpp::executors::MultiThreadedExecutor exec;
       auto node = std::make_shared<rclcpp::Node>("example_node");

       exec.add_node(node);
       exec.spin(); // Spinning the executor to handle callbacks
       
       rclcpp::shutdown();
       return 0;
   }
   ```

2. **No further configuration available**: ROS 2's `MultiThreadedExecutor` will automatically manage threads based on the number of available cores and will spin the nodes concurrently in the system.
3. **Thread priority and core pinning**: ROS 2 does not have built-in support for fine-grained control of thread priorities or core assignment in the `MultiThreadedExecutor`. However, you can influence thread management by adjusting system settings or using external operating system libraries for real-time control if needed.

## EDMS

In EDMS, multithreading is managed through the deployment configuration, where you can specify the size of the thread pool, thread importance, core affinity, and other parameters related to thread scheduling and management.

### Steps for Enabling Multithreading in EDMS

1. **Thread Pool Configuration**: The deployment configuration (`deployment.yaml`) allows you to specify the thread pool size and other related settings, such as thread priority and core affinity.

2. **Specify Thread Pool Size**: You can define the number of threads in the thread pool using the `thread_pool_size` parameter. This is part of the deployment configuration under the `deployed_instances` section.

3. **Set Thread Core Affinity**: The `core_affinity` parameter allows you to specify which CPU cores the threads should be bound to, ensuring that your threads run on specific cores.

4. **Thread Priority and Scheduling Policy**: EDMS allows you to define thread priority (`thread_priority`) and scheduling policy (`thread_sched_policy`) for each thread within the deployment configuration.

### Example YAML Configuration for Thread Pool

```yaml
yaaa_version: 0.6

imports:
  - <activity_graph_name>.activity_graph

deployment_targets:
  - name: <deployment_target_name_1>
    type: carma_0_22
    mempool_config:
      - chunk_size: <chunk size for the interfaces used>
        chunk_count: <chunk count for the interfaces used>

deployed_instances:
  - activity_instance: <activity_instance_name_to_deploy>
    deploy_to: <deployment_target_name_1>
    thread_pool_size: 2
    core_affinity: [0]
    importance: 10
    thread_config:
      - thread_name: T1
        thread_priority: 15
        thread_sched_policy: SCHED_FIFO
        thread_core_affinity: [0, 1]
      - thread_name: T2
        thread_priority: 17
        thread_sched_policy: SCHED_RR
        thread_core_affinity: [0, 2]
```

### Explanation of the YAML Configuration

- **`thread_pool_size`**: Specifies the number of threads in the thread pool.
- **`core_affinity`**: Defines the cores where the threads should be pinned. Here, `core_affinity: [0]` pins the thread to CPU core 0.
- **`importance`**: Indicates the importance of the thread or instance, affecting its scheduling priority.
- **`thread_config`**: This section provides detailed thread configurations:
  - **`thread_name`**: The name of the thread (e.g., `T1`, `T2`).
  - **`thread_priority`**: The priority of the thread, with a valid range depending on the OS.
  - **`thread_sched_policy`**: The scheduling policy used for the thread, e.g., `SCHED_FIFO` or `SCHED_RR`.
  - **`thread_core_affinity`**: The list of CPU cores to which the thread is pinned.

5. **Thread Affinity and Priority**: You can pin threads to specific cores and adjust their scheduling policies for improved real-time performance, as shown in the example above.

### Notes

- The `thread_name` is mandatory for each thread defined in `thread_config`. If `thread_name` is not provided, a compile-time error will be raised.
- For QNX systems, the thread priority is computed as `59 - thread_priority`, while on Linux, the priority is calculated as `thread_priority + 4`.
- Root privileges are required to modify thread priority and scheduling policy.

### Visualization and Debugging

EDMS provides a visualization tool, **YaaaVis**, to display thread configurations for the runnables. This tool shows the thread configuration parameters in a tooltip format and allows for downloading thread configurations in a CSV format for further analysis.

## References

- [ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- Refer to **EDMS Thread Pool Customization** in the User Documentation
