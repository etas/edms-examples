# Execution Entities in ROS 2 and EDMS

### ROS 2 Nodes

A **node** in **ROS 2** is a fundamental execution unit that encapsulates computation, communication, and logic for interacting with the system. Nodes communicate asynchronously with other nodes using **topics**, **services**, or **actions**.

- **Decentralized Execution**: Nodes operate independently and can be executed on different hardware platforms.
- **Communication**: Asynchronous communication via publish-subscribe model (topics) or request-response model (services, actions).
- **Flexibility**: Nodes can be written in multiple programming languages like **C++** or **Python**.

### EDMS Activities and Runnables

In **EDMS**, **Activities** are high-level processes that organize **Runnables** into a **Directed Acyclic Graph (DAG)**. An **Activity** groups multiple Runnables and can be triggered time-driven (cyclically) or data-driven.

- **Activity**: Defines a higher-level unit that coordinates Runnables. Each Activity instance runs as a separate process.
- **Runnable**: A single-threaded function that can read input data and can generate output data. It operates within an **Activity** and executes as part of the DAG.
- **DAG**: Ensures Runnables execute in the correct order, maintaining data integrity and execution sequence.
- **Activation Triggers**:
  - **Time-driven Activation**: Activities are triggered at a fixed interval (periodically).
  - **Data-driven Activation**: Activities are triggered by new data on input ports. A trigger condition can be defined.
  
### Key feature comparison

- **Execution Model**:
  - **ROS 2 Nodes**: Asynchronous, independent execution.
  - **EDMS Activities and Runnables**: Structured and deterministic execution with input/output data dependencies.
  
- **Communication**:
  - **ROS 2 Nodes**: Publish-and-subscribe via topics, request-response via services/actions.
  - **EDMS**: data ports via Interfaces. Within an Activity: DAG-based through input/output ports.

- **Activation Triggers**:
  - **ROS 2 Nodes**: time-triggered, data-triggered on level of callback-functions.
  - **EDMS**: time-triggered, data-triggered on Activity level.

- **System Architecture**:
  - **ROS 2 Nodes**: Decentralized, flexible architecture.
  - **EDMS**: Structured, performance-optimized for real-time embedded system on single platform.

## References

- [ROS 2 Nodes](https://docs.ros.org/en/rolling/Concepts/Basic/About-Nodes.html)
- [EDMS Activities](https://edms.etas.com/yaaa_modeling_concepts_and_workflow.html#activities)
- [EDMS Runnables](https://edms.etas.com/yaaa_modeling_concepts_and_workflow.html#runnables)
