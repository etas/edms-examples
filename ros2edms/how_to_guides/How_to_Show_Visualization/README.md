# How to Show Node/Topic Graph?

In this guide, we will explore how to visualize and monitor node and topic connections in both ROS 2 and EDMS. We will cover how to use ROS 2's `rosgraph` via RViz and how to visualize the system architecture in EDMS using **YAAA-Vis**.

## ROS 2: Using `rosgraph` via RViz

### Overview

In ROS 2, visualizing the node and topic graph can help developers understand the structure of the ROS system and the relationships between different nodes and topics. The primary tool used for this purpose is RViz, which integrates with ROS 2’s `rosgraph` to provide a visual representation of the system.

### Steps

1. **Launch RViz**:
   - Open a terminal and run the following command:

     ```bash
     ros2 run rviz2 rviz2
     ```

2. **Add `rosgraph` Plugin**:
   - In RViz, click the "Add" button at the bottom of the "Displays" panel.
   - From the list of available display types, select the "Topic" display type.
   - Set the topic to `/rosout` to display the active nodes and topics.

3. **Visualize the Graph**:
   - The `rosgraph` display will show the nodes in the system and the topics they are publishing/subscribing to. You can zoom in/out and move around to explore different nodes and their connections.

4. **Customize the View**:
   - You can further customize the visualization by adding other displays like "TF" or "Odometry" to visualize additional data related to the nodes.

This method will give you a real-time graphical representation of your ROS 2 node/topic relationships.

---

## EDMS: Using YAAA-Vis

### Overview

In EDMS, **YAAA-Vis** (YAAA Visualization) is the tool used to visualize the system architecture and the relationships between different components such as Activities, Runnables, Gateways, and more. YAAA-Vis provides a detailed and interactive visualization of the entire system model, making it easy to view and explore the underlying architecture.

### Steps

1. **Build the Project**:
   - If you are working with a pre-existing EDMS project and want to visualize the architecture, you first need to build the project before using YAAA-Vis for visualization.
   - Open a terminal in the project directory and run the following commands:

     ```bash
     conan install -if install -pr:b x86_64_linux_gcc8_debug -pr:h x86_64_linux_gcc8_debug $(pwd)
     conan build if install -bf build $(pwd)
     ```

2. **Generate the YAAA Model**:
   - Once the project is built, the YAAA model will be available. The model should be in a JSON format, generated as part of the build process.

3. **Navigate to the Build Folder**:
   - After the build is completed, go to the build folder where the visualization files are located. Typically, you can find this in the following directory:

     ```bash
     cd build/visualization
     ```

4. **Launch YAAA-Vis**:
   - In the `visualization` folder, you will find HTML files for visualizing your system. Open these files in your preferred browser (we recommend Firefox for optimal performance). The YAAA-Vis tool will automatically visualize the architecture based on the JSON model generated.

5. **Explore the Architecture**:
   - YAAA-Vis will open in your web browser, presenting the system architecture. You can zoom in/out, click on different components like Activities, Runnables, and Gateways to get detailed information, and explore the complete **Activity Graph** and the other system components.

6. **Visualize Nodes and Topics**:
   - The YAAA-Vis visualization will show all components in your system architecture, including Nodes, Gateways, and their connections. You can view individual nodes, topics, and other system elements with detailed hover tooltips and connection lines.

7. **Customize the Visualization**:
   - **Zooming & Scrolling**: You can zoom in and out or scroll around the architecture to view different components and connections.
   - **Search Functionality**: Use the search bar to search for specific components (e.g., nodes, activities, etc.).
   - **Highlighting**: Hover over ports or components to highlight connections and routes in the architecture.

8. **Advanced Features**:
   - **Export**: You can export your current view or zoom level to share with others.
   - **Tooltips**: Hover over elements to see more detailed information about specific nodes, topics, and their connections.

### Benefits of YAAA-Vis

- **Interactive Visualization**: The system model is presented in an interactive, zoomable format, making it easy to explore large architectures.
- **Real-Time Updates**: Changes in the model can be reflected in real-time, providing up-to-date information about the system’s state.
- **Component-Specific Views**: YAAA-Vis allows you to focus on specific parts of the architecture, such as gateways or memory configurations, to analyze the system in detail.

---

## Conclusion

Both ROS 2 and EDMS offer robust tools for visualizing node and topic graphs. In ROS 2, you can use RViz with `rosgraph` for a straightforward visualization of the active nodes and topics. In EDMS, **YAAA-Vis** provides a more comprehensive and interactive view of the entire system architecture, including detailed visualizations of activities, runnables, and other system components.

Depending on your middleware choice, you can use these tools to gain a better understanding of your system and make informed decisions during development and debugging.

## References

- [ROS 2 rosgraph](http://wiki.ros.org/rosgraph)
- [EDMS YAAA-Vis User Manual](https://edms.etas.com/explanations/yaaa_vis.html)
