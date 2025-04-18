# Comparison of Memory Resources

When dealing with memory resources, both ROS 2 and EDMS approach memory management differently to optimize performance, reduce latency, and ensure reliable execution in their respective environments. Below is a comparison of how each system handles memory resources.

## Memory Allocation and Management

- **ROS 2:**
  - Uses dynamic memory allocation by default, which can introduce non-deterministic behavior.
  - Implements real-time-safe memory allocation strategies in `rclcpp` through pre-allocated memory pools to reduce fragmentation and improve performance.
  - `ros2_rmw` middleware allows different strategies for handling memory, depending on the selected DDS implementation.
- **EDMS:**
  - Primarily relies on static memory allocation for deterministic behavior, ensuring that all memory requirements are defined at initialization.
  - Implements Direct Memory Connector (DMC) to manage externally allocated memory buffers with zero-copy data transfer.
  - Uses explicit memory management policies to avoid fragmentation and ensure consistent performance in constrained environments.

## Inter-Process Communication (IPC) and Shared Memory

- **ROS 2:**
  - Supports shared memory communication through DDS’s Zero-Copy Shared Memory Transport.
  - Implements intra-process communication to avoid unnecessary serialization/deserialization between nodes running in the same process.
  - The DDS-implementation, "Fast DDS" by Eprosima, provides a shared memory mechanism to reduce CPU usage by directly accessing memory without serialization.
- **EDMS:**
  - Provides a Direct Memory Connector (DMC) for zero-copy communication, eliminating unnecessary data copies.
  - Uses `NoViRoC` for next-generation IPC, ensuring memory efficiency and security.
  - Defines explicit memory access permissions to prevent unauthorized processes from accessing critical data.

## Real-Time Performance and Determinism

- **ROS 2:**
  - Offers Real-Time Linux (RT) compatibility and fine-tuned memory handling using real-time allocators.
  - Relies on lock-free data structures to minimize latency but still requires careful tuning to avoid memory fragmentation.
  - Real-time guarantees regarding data communication (e.g. re-transmitting data) depend on the underlying DDS implementation.
  - No real-time configuration for callback processing available as API.
  - Non-deterministic execution due to scheduling effects and Executor model.
- **EDMS:**
  - Designed for hard real-time performance with minimal runtime memory operations.
  - Ensures memory determinism by avoiding dynamic memory allocations at runtime.
  - Utilizes QNX Security Policies (Secpol) to control process memory access and prevent security vulnerabilities.
  - Deterministic execution of Activities (execution according to the DAG-structure and operating on "frozen" data in Activity inbox, once trigger condition is met)

## Memory Utilization in Gateway and Activity Models

- **ROS 2:**
  - Memory allocation depends on the middleware and configuration but generally follows a node-centric model where each node has its own allocated memory space.
  - Supports memory pooling for nodes that share similar data structures.
  - Loaned messages, memory buffers allocated by the underlying middleware, can be used by the application for zero-copy communication.
- **EDMS:**
  - Uses memory allocation strategies that align with Activity and Gateway models.
  - Gateways act as memory allocators for DMC, ensuring that only one gateway is responsible for memory allocation in a 1:n communication setup.
  - Provides a feedback loop mechanism where allocated memory is reused efficiently, reducing memory overhead.

## Flexibility and Customization

- **ROS 2:**
  - Allows developers to choose different memory allocation strategies and middleware implementations.
  - Custom memory management solutions can be implemented via plugins.
- **EDMS:**
  - Defines memory handling at the model level (YAAAC), allowing memory resources to be configured without recompilation.
  - Custom memory bindings for external frameworks (e.g., OpenVX) can be injected at compile time via CMake.

## Conclusion

- **ROS 2** provides flexibility in memory management, with options for optimizing real-time performance using pre-allocated memory pools and shared memory transport.
- **EDMS** takes a more deterministic and controlled approach, prioritizing static memory allocation, zero-copy communication, and strict access control for reliability in embedded systems.

Both approaches have their advantages depending on the use case, with **ROS 2** being more adaptable and **EDMS** being more deterministic and optimized for safety-critical applications.

## References

- [ROS 2 Memory allocator](https://docs.ros.org/en/rolling/Tutorials/Advanced/Allocator-Template-Tutorial.html)
- Refer to [Direct Memory Connector (DMC)] in the EDMS User Documentation
