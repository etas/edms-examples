import matplotlib.pyplot as plt

# Example values from the results for Tutorial 3 and 4
# These values would normally be extracted and aggregated from the logs.

# ROS2 results (Interprocess and Intraprocess)
edms_interprocess_t3 = [1.28, 1.37, 1.26, 1.30, 1.36, 1.35]  # Interprocess
edms_intraprocess_t3 = [0.73, 0.70, 0.71, 0.64, 0.67, 0.68]  # Intraprocess

# EDMS results (Interprocess and Intraprocess)
ros_interprocess_t3 = [0.89, 1.02, 0.75, 1.13, 1.31, 0.79]  # Interprocess
ros_intraprocess_t3 = [0.67, 0.54, 0.55, 0.60, 0.57, 0.56]   # Intraprocess

# ROS2 results (Interprocess and Intraprocess)
edms_interprocess_t4 = [1.28, 1.37, 1.26, 1.30, 1.36, 1.35]  # Interprocess
edms_intraprocess_t4 = [1.28, 1.37, 1.26, 1.30, 1.36, 1.35]  # Intraprocess

# EDMS results (Interprocess and Intraprocess)
ros_interprocess_t4 = [24.12, 15.83, 16.94, 24.86, 27.67, 22.18]  # Interprocess
ros_intraprocess_t4 = [95.41, 84.06, 88.07, 95.27, 98.27, 103.59]   # Intraprocess

# Time axis
time = [59.44, 119.44, 179.44, 239.44, 299.44, 359.44]  # Example time in seconds

# ROS2 latency visualization
plt.figure(figsize=(12, 6))

# ROS2 Interprocess and Intraprocess latencies
plt.subplot(1, 2, 2)
plt.bar(time, edms_intraprocess_t4, width=40, color='blue', alpha=0.2)  # Bars for ROS2 Interprocess
plt.title('EDMS Latency: Intraprocess (One Activity) with 6 GByte/s')
plt.xlabel('Time (s)')
plt.ylabel('Average Latency (ms)')
plt.ylim(0, 104)
plt.legend()

# EDMS latency visualization
plt.subplot(1, 2, 1)
plt.bar(time, ros_intraprocess_t4, width=40, color='green', alpha=0.2)  # Bars for EDMS Tutorial 4
plt.title('ROS2 Latency: Intraprocess (One Process) with 6 GByte/s')
plt.xlabel('Time (s)')
plt.ylabel('Average Latency (ms)')
plt.ylim(0, 104)
plt.legend()

# Optimize layout and display
plt.tight_layout()
plt.show()

# How to Guides

In this section, you will find detailed guides to help you set up and configure specific features and tasks within ROS2 and EDMS. These guides provide step-by-step instructions for common tasks, demonstrating how to use key features of both middleware systems.

You can explore the following guides:

- [How to Enable Multithreading](How_to_Enable_Multithreading/README.md) - Learn how to enable multithreading in ROS2 and EDMS, including how to configure thread pools and core pinning for optimal performance.
- [How to Show Visualization](How_to_Show_Visualization/README.md) - A guide on how to visualize the communication structure in ROS2 using `rosgraph` and in EDMS using `YAAA-Vis`.
- [How to Switch Off Logging](How_to_Switch_Off_Logging/README.md) - This guide explains how to disable logging in ROS2 and EDMS, helping you reduce overhead and control logging verbosity.
- [How to Show Total Memory Consumption](How_to_Show_Total_Memory_Consumption/README.md) - This guide walks you through how to monitor memory usage in both ROS2 and EDMS, with a focus on calculating memory pools and tracking consumption.
