# ArduinoBOT - ROS 2 Robotic Arm Project

![ArduinoBOT](https://img.shields.io/badge/ROS-2-brightgreen)
![Platform](https://img.shields.io/badge/platform-Linux-blue)
![License](https://img.shields.io/badge/license-Open%20Source-orange)

A comprehensive ROS 2 robotic arm project featuring simulation, control, motion planning, and remote operation capabilities using Gazebo, MoveIt2, and custom interfaces.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Usage Guide](#usage-guide)
- [Examples](#examples)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🔍 Overview

ArduinoBOT is a modular ROS 2 workspace designed for learning and developing robotic arm applications. It includes a complete simulation environment, control systems, motion planning capabilities, and example implementations for various ROS 2 concepts including publishers, subscribers, services, actions, and parameters.

### Key Components

- **3-DOF Robotic Arm**: A simulated robotic arm with gripper
- **Gazebo Integration**: Full physics simulation environment
- **MoveIt2 Integration**: Advanced motion planning and execution
- **ros2_control**: Hardware abstraction and control interfaces
- **Custom Messages/Services**: Specialized communication interfaces
- **Remote Control**: Alexa and task-based remote operation
- **Educational Examples**: Comprehensive ROS 2 learning materials

## ✨ Features

### Core Functionality
- 🤖 **3-DOF Robotic Arm Simulation** with gripper control
- 🎮 **Interactive Control** via RViz and joint state publisher
- 🌍 **Gazebo Physics Simulation** with realistic dynamics
- 🎯 **MoveIt2 Motion Planning** for complex trajectories
- 🔧 **ros2_control Integration** for hardware abstraction
- 📡 **Custom ROS 2 Interfaces** (messages, services, actions)

### Advanced Features
- 🗣️ **Voice Control** via Alexa integration
- 🎛️ **Remote Task Execution** through action servers
- 📊 **Real-time Monitoring** and visualization
- 🔄 **Coordinate Transformations** and utility functions
- 📚 **Educational Examples** covering all ROS 2 concepts

## 💻 System Requirements

### Operating System
- **Ubuntu 22.04 LTS** (Recommended)
- **ROS 2 Humble** or **Iron**

### Dependencies
- **Python 3.8+**
- **C++17** compiler
- **Gazebo Classic** or **Gazebo Fortress/Garden**
- **MoveIt2**
- **ros2_control**

### Hardware Requirements
- **RAM**: Minimum 4GB, Recommended 8GB+
- **CPU**: Multi-core processor (4+ cores recommended)
- **Graphics**: OpenGL 3.3+ compatible GPU

## 🚀 Installation

### 1. Prerequisites

First, ensure you have ROS 2 installed. If not, follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install required dependencies
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    git
```

### 2. Install ROS 2 Dependencies

```bash
# Install Gazebo and related packages
sudo apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-robot-state-publisher

# Install MoveIt2 and control packages
sudo apt install -y \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster
```

### 3. Clone and Build the Workspace

```bash
# Clone the repository
git clone https://github.com/pheonix-19/ArduinoBOT.git
cd ArduinoBOT

# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash

# Install dependencies using rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## 📁 Package Structure

```
src/
├── arduinobot_description/     # Robot model and visualization
│   ├── urdf/                  # Robot URDF/XACRO files
│   ├── meshes/                # 3D mesh files
│   ├── launch/                # Launch files for visualization
│   └── rviz/                  # RViz configurations
│
├── arduinobot_controller/      # Robot control configuration
│   ├── config/                # Controller parameters
│   └── launch/                # Control launch files
│
├── arduinobot_moveit/         # MoveIt2 motion planning
│   ├── config/                # MoveIt configuration
│   └── launch/                # Motion planning launch files
│
├── arduinobot_msgs/           # Custom ROS 2 interfaces
│   ├── msg/                   # Custom messages
│   ├── srv/                   # Custom services
│   └── action/                # Custom actions
│
├── arduinobot_py_examples/    # Python examples and tutorials
│   └── arduinobot_py_examples/
│       ├── simple_publisher.py
│       ├── simple_subscriber.py
│       ├── simple_service_server.py
│       ├── simple_action_server.py
│       ├── simple_parameter.py
│       └── simple_moveit_interface.py
│
├── arduinobot_cpp_examples/   # C++ examples and tutorials
│   └── src/                   # C++ source files
│
├── arduinobot_utils/          # Utility functions and tools
│   └── arduinobot_utils/
│       └── angle_conversion.py
│
└── arduinobot_remote/         # Remote control interfaces
    └── arduinobot_remote/
        ├── alexa_interface.py
        └── task_server.py
```

## 📖 Usage Guide

### Basic Visualization

Launch the robot model in RViz for visualization and manual joint control:

```bash
# Terminal 1: Launch robot visualization
ros2 launch arduinobot_description display.launch.py

# Optional: Specify custom URDF
ros2 launch arduinobot_description display.launch.py model:=/path/to/custom.urdf.xacro
```

**What this does:**
- Loads the robot model in RViz
- Starts joint state publisher GUI for manual control
- Allows real-time visualization of robot movements

### Gazebo Simulation

Launch the complete simulation environment:

```bash
# Terminal 1: Start Gazebo simulation
ros2 launch arduinobot_description gazebo.launch.py

# Terminal 2: Load and start controllers
ros2 launch arduinobot_controller controller.launch.py
```

**Features available:**
- Realistic physics simulation
- Interactive robot control
- Sensor integration
- Environmental interactions

### Motion Planning with MoveIt2

Start the motion planning interface:

```bash
# Terminal 1: Launch MoveIt2 interface
ros2 launch arduinobot_moveit moveit.launch.py

# Terminal 2: Run motion planning examples
ros2 run arduinobot_py_examples simple_moveit_interface
```

**Capabilities:**
- Interactive motion planning
- Collision detection and avoidance
- Trajectory optimization
- End-effector control

### Manual Robot Control

#### Joint Position Control

Control individual joints using ROS 2 topics:

```bash
# Control arm joints (joint_1, joint_2, joint_3)
ros2 topic pub /arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{
        joint_names: ['joint_1', 'joint_2', 'joint_3'],
        points: [{
            positions: [0.5, 0.3, -0.2],
            time_from_start: {sec: 2}
        }]
    }"

# Control gripper
ros2 topic pub /gripper_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "{data: [0.5]}"
```

#### Using Command Line Interface

```bash
# Open gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5]}"

# Close gripper
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"

# Move to home position
ros2 service call /arm_controller/switch_controller controller_manager_msgs/srv/SwitchController \
    "{start_controllers: ['arm_controller'], stop_controllers: [], strictness: 1}"
```

## 🎯 Examples

### 1. Publisher/Subscriber Pattern

**Publisher Example:**
```bash
# Terminal 1: Run subscriber
ros2 run arduinobot_py_examples simple_subscriber

# Terminal 2: Run publisher
ros2 run arduinobot_py_examples simple_publisher
```

**What happens:**
- Publisher sends messages every second
- Subscriber receives and logs messages
- Demonstrates basic ROS 2 communication

### 2. Service/Client Communication

**Service Server:**
```bash
# Terminal 1: Start service server
ros2 run arduinobot_py_examples simple_service_server

# Terminal 2: Call service
ros2 service call /add_two_ints arduinobot_msgs/srv/AddTwoInts "{a: 5, b: 3}"
```

**Response:**
```
sum: 8
```

### 3. Action Server/Client

**Action Server:**
```bash
# Terminal 1: Start action server
ros2 run arduinobot_py_examples simple_action_server

# Terminal 2: Send action goal
ros2 action send_goal /countdown arduinobot_msgs/action/Countdown "{seconds: 10}"
```

### 4. Parameter Management

```bash
# Run parameter node
ros2 run arduinobot_py_examples simple_parameter

# Get parameter value
ros2 param get /simple_parameter simple_int_param

# Set parameter value
ros2 param set /simple_parameter simple_int_param 42
```

### 5. Coordinate Transformation Utilities

```bash
# Convert Euler angles to quaternion
ros2 service call /euler_to_quaternion arduinobot_msgs/srv/EulerToQuaternion \
    "{roll: 0.0, pitch: 0.0, yaw: 1.57}"

# Convert quaternion to Euler angles
ros2 service call /quaternion_to_euler arduinobot_msgs/srv/QuaternionToEuler \
    "{x: 0.0, y: 0.0, z: 0.707, w: 0.707}"
```

## 🔧 Advanced Usage

### Custom Robot Configuration

Modify robot parameters by editing URDF files:

```bash
# Edit main robot description
nano src/arduinobot_description/urdf/arduino-bot.urdf.xacro

# Rebuild after changes
colcon build --packages-select arduinobot_description
source install/setup.bash
```

### Controller Configuration

Adjust control parameters:

```bash
# Edit controller configuration
nano src/arduinobot_controller/config/arduinobot_controllers.yaml

# Apply changes
colcon build --packages-select arduinobot_controller
```

### Adding Custom Behaviors

Create custom Python nodes:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
    
    def joint_callback(self, msg):
        # Custom control logic here
        self.get_logger().info(f'Joint positions: {msg.position}')

def main():
    rclpy.init()
    controller = CustomController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 📡 API Reference

### Custom Messages

#### `arduinobot_msgs/msg/` (if any custom messages exist)

### Custom Services

#### `AddTwoInts.srv`
```
int32 a
int32 b
---
int32 sum
```

#### `EulerToQuaternion.srv`
```
float64 roll
float64 pitch
float64 yaw
---
float64 x
float64 y
float64 z
float64 w
```

#### `QuaternionToEuler.srv`
```
float64 x
float64 y
float64 z
float64 w
---
float64 roll
float64 pitch
float64 yaw
```

### Custom Actions

#### `ArduinobotTask.action`
```
# Goal
string task_name
int32 task_number
---
# Result
bool success
string message
---
# Feedback
string feedback
```

### Launch File Parameters

#### `display.launch.py`
- `model`: Path to URDF file (default: arduino-bot.urdf.xacro)

#### `gazebo.launch.py`
- `model`: Path to URDF file
- `is_ignition`: Use Ignition Gazebo (default: true for ROS 2 Iron+)

#### `controller.launch.py`
- No parameters (uses default configuration)

## 🛠️ Troubleshooting

### Common Issues

#### 1. Build Failures

```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### 2. Gazebo Won't Start

```bash
# Check Gazebo installation
gz sim --version  # For newer versions
gazebo --version  # For Gazebo Classic

# Set environment variables
export GZ_SIM_RESOURCE_PATH=$PWD/src
```

#### 3. Controllers Not Loading

```bash
# Check controller manager status
ros2 control list_hardware_interfaces

# Restart controller manager
ros2 lifecycle set /controller_manager configure
ros2 lifecycle set /controller_manager activate
```

#### 4. MoveIt2 Planning Failures

```bash
# Check MoveIt configuration
ros2 launch arduinobot_moveit moveit.launch.py

# Verify planning scene
ros2 topic echo /planning_scene
```

### Debug Commands

```bash
# Check node status
ros2 node list
ros2 node info /node_name

# Monitor topics
ros2 topic list
ros2 topic echo /topic_name

# Service debugging
ros2 service list
ros2 service type /service_name

# Action debugging
ros2 action list
ros2 action info /action_name
```

### Log Analysis

```bash
# View build logs
cat log/latest_build/package_name/stdout.log

# Runtime logs
ros2 run rqt_console rqt_console

# Node-specific logs
ros2 run your_package your_node --ros-args --log-level DEBUG
```

## 🎓 Educational Resources

### Learning Path

1. **Basic ROS 2 Concepts**
   - Start with `simple_publisher.py` and `simple_subscriber.py`
   - Understand topic-based communication

2. **Service Communication**
   - Run `simple_service_server.py`
   - Learn request-response patterns

3. **Action Servers**
   - Explore `simple_action_server.py`
   - Understand long-running tasks with feedback

4. **Robot Visualization**
   - Use `display.launch.py`
   - Learn URDF and robot state publishing

5. **Simulation**
   - Progress to `gazebo.launch.py`
   - Understand physics simulation

6. **Motion Planning**
   - Use MoveIt2 integration
   - Learn trajectory planning and execution

### Tutorials and Exercises

1. **Modify Joint Limits**: Edit URDF files to change robot capabilities
2. **Add Sensors**: Integrate cameras or LIDAR to the robot
3. **Custom Controllers**: Implement PID or other control algorithms
4. **Voice Commands**: Extend Alexa integration for new commands
5. **Vision Processing**: Add computer vision capabilities

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork the Repository**
   ```bash
   git fork https://github.com/pheonix-19/ArduinoBOT.git
   ```

2. **Create Feature Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make Changes**
   - Follow ROS 2 coding standards
   - Add appropriate documentation
   - Include tests where applicable

4. **Test Your Changes**
   ```bash
   colcon build
   colcon test
   ```

5. **Submit Pull Request**
   - Provide clear description of changes
   - Include any relevant issue numbers

### Code Style Guidelines

- **Python**: Follow PEP 8
- **C++**: Follow ROS 2 C++ style guide
- **XML/YAML**: Proper indentation and structure
- **Documentation**: Include docstrings and comments

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/pheonix-19/ArduinoBOT/issues)
- **Discussions**: [GitHub Discussions](https://github.com/pheonix-19/ArduinoBOT/discussions)
- **Email**: am2836166@gmail.com

## 🙏 Acknowledgments

- ROS 2 Community for excellent documentation and tools
- MoveIt2 developers for motion planning capabilities
- Gazebo team for simulation environment
- Open Robotics for ROS ecosystem

---

**Happy Robot Programming! 🤖**

*For more advanced usage and detailed API documentation, please refer to the individual package documentation in each subdirectory.*
