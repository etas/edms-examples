# ROS 2 Installation Guide

This guide provides a step-by-step walkthrough for installing ROS 2 on Ubuntu 22.04.

## Requirements

Before beginning the installation, ensure your system meets the following requirements:

- **Operating System**: Ubuntu 22.04 (other versions may require different instructions).
- **User Privileges**: You need `sudo` (administrator) privileges to install packages.
- **Internet Connection**: Required to download packages and dependencies.

## Installation Steps

### Step 1: Add ROS 2 Package Sources

1. **Update the package index** and install necessary dependencies:

```bash
sudo apt update && sudo apt install -y software-properties-common
```

2. **Enable the Universe repository**, which contains open-source community-maintained packages:

```bash
sudo add-apt-repository universe
```

3. **Install additional dependencies** required for downloading and verifying the ROS 2 packages:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

4. **Add the ROS 2 GPG key** to verify package authenticity:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

5. **Add the ROS 2 package repository** to your sources list:

```bash
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 2: Install ROS 2

1. **Update the package index** to include the new ROS 2 repository:

```bash
sudo apt update
```

2. **Install the ROS 2 desktop package**, which includes essential tools, libraries, and the RViz visualization tool:

```bash
sudo apt install ros-humble-desktop
```

### Step 3: Source the ROS 2 Setup Script

1. **Source the ROS 2 setup script** to add ROS 2 to your terminal session’s environment:

```bash
source /opt/ros/humble/setup.bash
```

2. **Optional:** To automatically source ROS 2 every time you open a new terminal, add this line to your [.bashrc] file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verification

To verify that ROS 2 has been installed correctly, open a new terminal (or source your .bashrc) and run the following command:

```bash
ros2 --help
```

If ROS 2 is installed correctly, you should see a list of ROS 2 commands and options.

This completes the installation of ROS 2 on your system. You are now ready to start developing with ROS 2. For more information, refer to the official [ROS 2 documentation](https://docs.ros.org/en/humble/index.html).
