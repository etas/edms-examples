# ROS 2 Parameters and EDMS Internal States

## Introduction

When designing middleware solutions for robotics and embedded systems, managing state and configuration parameters is crucial. Both ROS 2 and EDMS provide mechanisms for this, but they take different approaches. This section explores the conceptual differences between ROS 2 Parameters and EDMS Internal States and how each system facilitates runtime configuration and data management.

## ROS 2 Parameters

ROS 2 parameters are key-value pairs associated with a specific Node that allow for runtime configuration without modifying the source code. They enable developers to fine-tune the behavior of nodes and are commonly used for:

- Adjusting configuration settings dynamically.
- Persisting runtime values that influence node execution.
- Enabling or disabling features within a node.

### Characteristics

- **Scoped to individual Nodes**: Each node maintains its own parameter set.
- **Standardized API**: Nodes can declare, get, and set parameters using the ROS client library (e.g.`rclcpp` or `rclpy`) API.
- **Persistent or transient**: Parameters can be loaded at startup (e.g., from a YAML file) or modified at runtime.
- **Parameter Events**: ROS 2 allows monitoring of parameter changes through events, enabling reactive system behavior.

### Example Usage

```python
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.declare_parameter('threshold', 5)
        self.get_logger().info(f"Threshold value: {self.get_parameter('threshold').value}")

rclpy.init()
node = ExampleNode()
rclpy.spin(node)
rclpy.shutdown()
```

## EDMS Internal States

EDMS employs internal states as a means of managing data and memory buffers efficiently. Unlike ROS 2 parameters, which focus on configuration, Internal States in EDMS are primarily used for managing shared memory and data buffers between different processes.

### Key Characteristics

- **Shared memory management**: Internal states allow processes to share hardware-accelerated memory buffers, such as those from TI-OpenVX.
- **Zero-Copy Data Exchange**: The Direct Memory Connector (DMC) facilitates efficient memory sharing without additional copying, improving performance in high-throughput applications.
- **Framework-Specific Bindings**: Internal states in EDMS are closely tied to framework-specific bindings (e.g., TI-OpenVX) and require predefined data structures.
- **Data Synchronization & Buffer Management**: Internal states help synchronize data flow between Activities and Gateways within the middleware.

### Example Use Case

A Triggered Gateway allocates and manages memory for a set of consumers. The allocated memory is exposed to consumers through DMC ports, avoiding unnecessary copies and improving real-time performance.

## Key Feature Comparison

| Feature                | ROS 2 Parameters                          | EDMS Internal States                        |
|------------------------|-------------------------------------------|---------------------------------------------|
| **Purpose**            | Configuration of node behavior            | Memory sharing and state management         |
| **Scope**              | Per-node                                  | Shared across multiple processes            |
| **Persistence**        | Can be stored and reloaded                | Runtime-based, dependent on memory buffers  |
| **Data Type Support**  | Basic types (int, float, bool, string, arrays) | Complex structured data (OpenVX buffers, tensors) |
| **Modification**       | CLI, YAML files, API calls                | Managed via gateways and activities         |
| **Performance Impact** | Minimal overhead                          | Zero-copy optimization for performance      |
| **Use Cases**          | Setting thresholds, toggling features     | Sharing image and tensor data between processes |

## Conclusion

While both ROS 2 Parameters and EDMS Internal States serve as mechanisms for managing runtime data, they are tailored for different purposes. ROS 2 Parameters provide a flexible way to configure node behavior, whereas EDMS Internal States focus on efficient, zero-copy data exchange across processes. The choice between the two depends on whether the primary concern is configuration flexibility or high-performance memory management in embedded systems.

## References

- [ROS 2 Parameters](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html)
- Refer to [Internal States in DMC] in the EDMS User Documentation
