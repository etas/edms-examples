# How to Switch Off Logging?

In this guide, we will discuss how to switch off logging in both ROS 2 and EDMS. Logging can be useful for debugging and monitoring, but in certain scenarios you may want to disable it for performance reasons or to reduce output clutter.

## ROS 2: Using the `RCLCPP_LOG_LEVEL` Environment Variable

### Overview

In ROS 2, logging is controlled using the `rclcpp` library, which provides various log levels (e.g., INFO, WARN, ERROR). You can control the verbosity of logging output by setting the `RCLCPP_LOG_LEVEL` environment variable to different levels. To completely switch off logging, you can set this variable to `OFF`.

### Steps

1. **Set the Environment Variable**:
   - To disable logging completely in ROS 2, set the `RCLCPP_LOG_LEVEL` environment variable to `OFF` before running your ROS 2 application. In your terminal, run the following command:

     ```bash
     export RCLCPP_LOG_LEVEL=OFF
     ```

2. **Run Your ROS 2 Application**:
   - Once the environment variable is set, run your ROS 2 application as usual. All logging output will be suppressed.

     ```bash
     ros2 run <your_package> <your_executable>
     ```

3. **Verify Logging is Disabled**:
   - You should no longer see any log messages (e.g., INFO, WARN, ERROR) in the terminal output when running your application.

### Log Levels in ROS 2

- **DEBUG**: Detailed information, typically for developers.
- **INFO**: General runtime information.
- **WARN**: Warnings about potential issues.
- **ERROR**: Error messages.
- **OFF**: No logging output at all.

## EDMS: Turning Off Logging

### Overview

In EDMS, logging can be managed using the **Bosch MPLOG** library, which is used for default logging in EDMS. The logging system in EDMS is highly configurable, and you can control the verbosity of logs using different profiles. To completely turn off logging, you can set the profile to `OFF`.

### Steps

1. **Include the Logging Header**:
   - In your application, include the necessary header for logging:

     ```cpp
     #include "aos_logging/default_logging/logging.hpp"
     ```

2. **Initialize the Logging System**:
   - Initialize the default logging system by calling the `init` function. This is usually done at the start of your program:

     ```cpp
     aos_logging::default_logging::init();
     ```

3. **Set the Logging Profile to OFF**:
   - To disable logging completely, set the logging profile to `OFF` using the `setProfile` function:

     ```cpp
     aos_logging::default_logging::setProfile(aos_logging::default_logging::logging_types::Profile::OFF);
     ```

4. **Run Your EDMS Application**:
   - Once the logging profile is set to `OFF`, run your EDMS application as usual. Logging output will be completely suppressed.

### Example Code

Here is an example showing how to disable logging in EDMS:

```cpp
#include "aos_logging/default_logging/logging.hpp"

int main(int argc, char* argv[])
{
    // Initialize logging
    aos_logging::default_logging::init();

    // Disable logging by setting the profile to OFF
    aos_logging::default_logging::setProfile(aos_logging::default_logging::logging_types::Profile::OFF);

    // Your application code here...

    return 0;
}
```

### Logging Profiles in EDMS

- **OFF**: No logging output at all.
- **FATAL**: Only fatal errors are logged.
- **ERROR**: Errors are logged.
- **WARNING**: Warnings are logged.
- **INFO**: General information is logged.
- **DEBUG**: Detailed debugging information is logged.

You can choose the appropriate profile based on your requirements, and setting it to `OFF` ensures that no log messages will be output during the execution of your program.

## Conclusion

Disabling logging can be useful when you need to reduce output verbosity or improve performance. In ROS 2, logging is controlled via the `RCLCPP_LOG_LEVEL` environment variable, and in EDMS, it is managed through the `setProfile` function provided by the **Bosch MPLOG** library. By setting the appropriate level or profile to `OFF`, you can completely switch off logging in both systems.

## References

- [ROS 2 Logging](https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html)
- Refer to **Default Logging** in the EDMS User Documentation
