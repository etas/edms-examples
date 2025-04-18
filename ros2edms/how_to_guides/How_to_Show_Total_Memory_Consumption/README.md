# How to Show Total Memory Consumption?

## ROS 2

In ROS 2, there isn't a built-in command that directly provides the total memory consumption across nodes and topics. However, you can monitor system memory usage using external tools like `top`, `htop`, or specific monitoring tools that track memory usage for processes.

### Using System Monitoring Tools

1. **htop**:
   - Open a terminal and run:

     ```bash
     htop
     ```

   - This will give you a real-time overview of the processes running, along with their memory usage.

2. **top**:
   - You can also use the `top` command to view memory consumption:

     ```bash
     top
     ```

   - Look for the process related to your ROS 2 nodes and their memory usage.

3. **ros2 run diagnostics**:
   - ROS 2 provides a diagnostic tool that might help in tracking node health and performance, although it doesn't directly show total memory consumption:

     ```bash
     ros2 run rclcpp_components component_container
     ```

---

## EDMS

In EDMS, total memory consumption can be calculated using the **Mempool Report Generator**. The report shows memory usage based on the number of samples and their respective sizes in shared memory. Here’s how you can generate and interpret the memory consumption data in EDMS.

### Step 1: Build the Project

Before you can generate the memory pool report, ensure your project is built and ready. If you already have the project set up but need to generate the visualizations, you can follow these steps:

1. **Install dependencies**:
   - Run the following command to install all the necessary dependencies:

     ```bash
     conan install -if install -pr:b x86_64_linux_gcc8_debug -pr:h x86_64_linux_gcc8_debug $(pwd)
     ```

2. **Build the project**:
   - After dependencies are installed, build your project:

     ```bash
     conan build -if install -bf build $(pwd)
     ```

### Step 2: Run the Mempool Report Generator

Once the project is built, you can use the **Mempool Report Generator** to calculate and show the memory consumption.

1. **Generate the report**:
   - Navigate to your project directory and run the following command:

     ```bash
     ./yaaac2 --mempool-report -I <include-path-1> ... -I <include-path-n> <deployment_file> -o <output-dir> -cls <class_info_file-1> ... -cls <class_info_file-m> [<other-yaaac-arguments>]
     ```

   - **Arguments Explanation**:
     - `-I <include-path>`: Include paths to your project files.
     - `<deployment_file>`: The deployment YAML file that describes your system.
     - `-o <output-dir>`: The output directory where the report will be saved.
     - `-cls <class_info_file>`: Class-info files for each interface type used.

2. **Find the report**:
   - After running the above command, the memory consumption report will be generated in the specified output directory.
   - The report will be stored as a YAML file named `<middleware>_mempool_report_<deployment_target_name>.yaml`.

### Step 3: Interpret the Mempool Report

The generated report will provide detailed information about the memory consumption:

- **total number of bytes used**: This represents the total memory consumed by all samples used by the senders.
- **sender information**: Each sender will have detailed memory usage including:
  - `mempool_size`: The total memory used by the samples of that sender.
  - `sample_size`: The size of each sample.
  - `sample_count`: The number of samples used by that sender.
  - **Communication details**: Includes information about how memory is used across different communication types (e.g., local, inbox, gateway).

Here’s an example output from the report:

```yaml
deployment_combined_samples_memory_usage: 41984
dmc_combined_number_of_samples: 20
senders:
   - Sender1:
      mempool_size: 41984
      sample_size: 1024
      sample_count: 41
      sample_count_details:
         - alloc: 1
         - sent: 10
         - local: 10
         - inbox: 20
         - gateway: 10
         - mta_gateway: 0
```

In the report above:

- The total memory consumption is **41984 bytes**.
- The `sample_count_details` provide insights into how the memory is distributed across different communication types, such as:
  - **alloc**: Number of samples allocated by the sender.
  - **sent**: Number of samples sent to the receivers.
  - **local**: Memory required for local receivers.
  - **inbox**: Memory required for inbox communication.
  - **gateway**: Memory required for gateway communication.
  - **mta_gateway**: Memory required for MTA Gateway communication.

### Step 4: Optimize Memory Usage (Optional)

If you find that the memory consumption is higher than expected, consider optimizing your system by:

- **Reducing Cache Size**: Lower the `cache_size` of the receivers to reduce memory consumption. Each receiver holds a buffer of samples, and reducing the cache size can lower memory usage, especially in systems with many receivers.
- **Adjusting Sample Allocation**: The sample count can be reduced by optimizing the allocation strategy for communication types, such as reducing the number of samples needed for inbox communication. You can adjust the `cache_size` and `port_policy` to minimize unnecessary sample usage.
- **Reviewing MTA Gateway Usage**: The MTA Gateway can be a significant contributor to memory usage, especially when it tracks all senders, even those with no receivers. Consider reducing the number of samples used by the MTA Gateway by configuring the `cache_size` in the **MeasurementConfig** section of the deployment file.
- **Enabling Optimizations**: Some optimizations, such as reusing allocated samples for local communication or using sent samples for local communication, can help reduce memory consumption.

Refer to the **Mempool Calculation User Documentation** for detailed guidance on optimizing the memory usage.

---

### Conclusion

By using the Mempool Report Generator in EDMS, you can easily calculate and manage memory consumption for communication between senders and receivers. In ROS 2, monitoring memory consumption typically requires external tools or custom implementations, while EDMS provides a more integrated and detailed approach to memory management within the system.

With these techniques and configurations, you can efficiently manage and optimize memory usage for your embedded systems, ensuring better performance and resource allocation.

## References

- [ROS 2 Executors](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- Refer to **EDMS Mempool Calculation** in the User Documentation