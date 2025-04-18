# Execution Management

## Introduction

Execution management is a crucial aspect of any middleware system, ensuring that processes are launched, monitored, and controlled effectively. In ROS 2, execution management is handled through nodes, lifecycle management, and Executors, while in EDMS, execution is managed via processes, run groups, and manifests. This section compares these two approaches and highlights key differences.

## Execution Management in ROS 2

ROS 2 provides an execution management framework primarily based on nodes, lifecycle states, and executors.

### Nodes and Executors

In ROS 2, computational logic is encapsulated in nodes, which can be run independently or grouped into processes. Executors manage the scheduling of these nodes, ensuring that callback functions are executed as needed based on message passing, timers, or service requests.

### Lifecycle Management

ROS 2 introduces managed lifecycle nodes, allowing finer control over node execution. The lifecycle consists of pre-defined states such as `unconfigured`, `inactive`, `active`, and `finalized`. Transitions between states are controlled through explicit service calls, enabling safe startup, shutdown, and error handling.

### Process Isolation and Security

ROS 2 processes run as independent operating system processes, allowing strong isolation between components. Encrypted communication and real-time communication options are provided by DDS middleware.

## Execution Management in EDMS

EDMS takes a different approach, focusing on process management, security, and flexible configuration through manifest files.

### Processes and Run Groups

In EDMS, an executable refers to a compiled file that can be launched as a process. Each process has a unique environment, including assigned resources and environment variables. Processes are grouped into **Run Groups**, ensuring that related processes, such as gateways and activities, start and stop together.

### State Monitoring and Dependency Management

Processes in EDMS follow a structured lifecycle, monitored by Esme. This enables:

1. Dependency management: Processes can be scheduled based on dependencies (e.g., Process B starts only after Process A completes initialization).
2. Health monitoring: Esme tracks process states to detect failures and react accordingly.

### Security and Inter-Process Communication

EDMS enhances security by enforcing strict memory access policies. It supports Iceoryx for inter-process communication (IPC) or its next-generation NoViRoC IPC, where Esme configures memory access permissions. Additionally, QNX Security Policies (Secpol) can be applied for process-level access control.

### Manifest-Based Configuration

Unlike ROS 2, where configurations are typically set within launch files or parameters, EDMS uses JSON-based **manifest files**. These allow developers to modify execution configurations without recompiling the project, offering greater flexibility.

## Key Feature Comparison

| Feature                     | ROS 2                    | EDMS (Esme Execution Management) |
| --------------------------- | ------------------------ | -------------------------------- |
| Execution Unit              | Nodes                    | Processes                        |
| Grouping                    | Executors                | Run Groups                       |
| Lifecycle Management        | Managed Lifecycle Nodes  | Process State Monitoring         |
| Inter-Process Communication | DDS-based                | Iceoryx / NoViRoC                |
| Security                    | DDS                      | Mandatory security policies      |
| Configuration               | Launch files, parameters | JSON manifests                   |

## Conclusion

Both ROS 2 and EDMS offer robust execution management, but their focus differs. ROS 2 emphasizes flexibility and modularity through nodes and executors, while EDMS prioritizes security, structured execution, and process isolation. The choice between the two depends on system requirements, with ROS 2 being well-suited for research and prototyping, and EDMS being optimized for secure and deterministic execution in automotive and embedded environments.

## References

- [ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- Refer to **ESME Execution Management** in EDMS User Documentation
