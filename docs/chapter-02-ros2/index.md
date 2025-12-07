---
sidebar_position: 1
---

# Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

ROS 2 is a complete redesign of the original ROS, built to address the needs of modern robotics applications, including multi-robot systems, real-time control, and commercial products. It is built on top of the Data Distribution Service (DDS) standard, which provides a robust and scalable communication layer.

## Why ROS 2?

### Evolution from ROS 1

The original ROS (now called ROS 1) was created in 2007 and became the de facto standard for robotics research and development. However, as robotics applications evolved, certain limitations became apparent:

**ROS 1 Limitations:**
- Single point of failure (ROS Master)
- No native real-time support
- Limited multi-robot capabilities
- TCP-based communication not ideal for all scenarios
- No built-in security

**ROS 2 Improvements:**
- Decentralized architecture (no master node)
- Real-time capable
- Native multi-robot support
- DDS-based communication with QoS
- Security features built-in
- Better support for embedded systems
- Commercial-grade reliability

### Key Features of ROS 2

**Distributed System**
- Nodes can discover each other automatically
- No central point of failure
- Scales to multiple robots and computers

**Real-Time Performance**
- Deterministic communication
- Priority-based scheduling
- Low-latency message passing

**Quality of Service (QoS)**
- Configure reliability, durability, and latency
- Adapt communication to network conditions
- Match different application requirements

**Security**
- Authentication and encryption
- Access control
- Secure communication channels

**Cross-Platform**
- Linux, Windows, macOS
- Embedded systems
- Microcontrollers (with micro-ROS)

## ROS 2 Ecosystem

ROS 2 is more than just a communication framework—it's a complete ecosystem:

### Core Components

**rclcpp / rclpy**
- C++ and Python client libraries
- Write nodes in your preferred language
- Same concepts, different syntax

**Launch System**
- Start multiple nodes with one command
- Configure parameters
- Manage complex systems

**Build System (colcon)**
- Build ROS 2 packages
- Manage dependencies
- Support for multiple build types

**Command-Line Tools**
- `ros2 run`: Run nodes
- `ros2 topic`: Inspect topics
- `ros2 service`: Call services
- `ros2 bag`: Record and playback data

### Visualization and Debugging

**RViz2**
- 3D visualization of robot state
- Display sensor data
- Interactive markers

**rqt**
- Qt-based GUI tools
- Graph visualization
- Topic monitoring
- Parameter configuration

**Gazebo**
- Physics-based simulation
- Test robots safely
- Generate synthetic data

### Common Packages

**Navigation2 (Nav2)**
- Autonomous navigation stack
- Path planning and obstacle avoidance
- Localization and mapping

**MoveIt2**
- Motion planning for manipulators
- Kinematics and dynamics
- Collision detection

**ros2_control**
- Hardware abstraction layer
- Controller management
- Real-time control

## Who Uses ROS 2?

ROS 2 is used across industry, research, and education:

**Industry**
- Manufacturing automation
- Autonomous vehicles
- Service robots
- Agricultural robots
- Warehouse automation

**Research**
- Universities worldwide
- Research institutions
- AI and robotics labs

**Education**
- Robotics courses
- Competitions (RoboCup, etc.)
- Learning platforms

## ROS 2 Distributions

ROS 2 releases new distributions regularly, each with long-term support (LTS) versions:

**Recent Distributions:**
- **Humble Hawksbill** (2022, LTS) - Recommended for production
- **Iron Irwini** (2023)
- **Jazzy Jalisco** (2024, LTS)
- **Rolling Ridley** (continuous, latest features)

Each distribution is tied to specific Ubuntu LTS releases for maximum stability.

## Getting Started with ROS 2

### Installation

ROS 2 can be installed on:
- **Ubuntu** (recommended, best support)
- **Windows** (native support)
- **macOS** (community supported)
- **Docker** (cross-platform)

### Learning Path

1. **Understand core concepts**: Nodes, topics, services, actions
2. **Write simple nodes**: Publishers and subscribers
3. **Create packages**: Organize your code
4. **Use launch files**: Start complex systems
5. **Integrate sensors**: Work with real hardware
6. **Build applications**: Navigation, manipulation, etc.

## ROS 2 Philosophy

ROS 2 follows key design principles:

**Modularity**
- Small, focused nodes
- Reusable components
- Clear interfaces

**Flexibility**
- Support multiple languages
- Work with various hardware
- Adapt to different use cases

**Community-Driven**
- Open source
- Active community
- Extensive documentation

**Production-Ready**
- Reliability and stability
- Security features
- Commercial support available

## The Power of ROS 2

ROS 2 enables you to:

- **Build faster**: Leverage existing packages and tools
- **Scale easily**: From single robot to robot fleets
- **Integrate seamlessly**: Work with sensors, actuators, and algorithms
- **Deploy confidently**: Production-grade reliability
- **Collaborate effectively**: Standard interfaces and tools

## What's Next?

In the following sections, we'll explore:

- Core architecture and communication patterns
- DDS and the ROS graph
- Topics, services, and actions
- Python integration with rclpy
- URDF for robot description

By the end of this chapter, you'll have a solid understanding of ROS 2 and be ready to build your own robotic applications.

---

*ROS 2 is the foundation upon which modern robotics applications are built. Let's dive in!* 🤖
