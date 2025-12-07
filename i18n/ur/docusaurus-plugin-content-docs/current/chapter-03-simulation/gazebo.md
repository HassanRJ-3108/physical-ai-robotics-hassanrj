---
sidebar_position: 3
---

# Gazebo

Gazebo is an open-source robot simulator that is tightly integrated with the Robot Operating System (ROS). It is the de facto standard for simulation in the ROS ecosystem.

## Strengths

### Deep ROS Integration
Gazebo is designed to work seamlessly with ROS. It can:
- Subscribe to ROS topics to control the robot
- Publish sensor data back to the ROS graph
- Use ROS services and actions
- Integrate with ROS tools (RViz, rqt, etc.)

### Physics Simulation
Gazebo uses robust physics engines for realistic simulation:
- **ODE (Open Dynamics Engine)**: Default, good balance of accuracy and performance
- **Bullet**: Alternative physics engine
- **Simbody**: High-fidelity dynamics
- **DART**: Advanced dynamics and kinematics

### Wide Range of Sensors
Gazebo supports comprehensive sensor models:
- Cameras (monocular, stereo, depth)
- LiDAR and laser scanners
- IMUs (accelerometer, gyroscope)
- GPS
- Contact and force sensors
- Sonar and ultrasonic sensors

### Large Community
As the standard ROS simulator, Gazebo has:
- Extensive documentation and tutorials
- Large model database
- Active community support
- Many example worlds and robots

## Weaknesses

### Rendering Quality
While functional, the rendering quality in Gazebo is not as realistic as modern game engines like Unity. This can be a limitation for:
- Training vision-based AI models
- Photorealistic visualization
- Marketing and demonstrations

### User Interface
The user interface can be less intuitive and user-friendly compared to game engines. Learning curve includes:
- SDF (Simulation Description Format)
- Plugin development
- World building tools

## Getting Started with Gazebo

### Installation

```bash
# Install Gazebo (comes with ROS 2)
sudo apt install ros-humble-gazebo-ros-pkgs

# Or install standalone
sudo apt install gazebo
```

### Basic Usage

```bash
# Launch Gazebo
gazebo

# Launch with a world file
gazebo worlds/willowgarage.world

# Launch with ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

### Creating a Simple World

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Advanced Features

**Plugins**: Extend Gazebo functionality with C++ plugins

**Sensors**: Simulate realistic sensor noise and characteristics

**Actors**: Simulate dynamic human actors in the environment

**Distributed Simulation**: Run simulation across multiple machines

## Gazebo Versions

**Gazebo Classic** (Gazebo 11): Traditional version, widely used

**Ignition Gazebo** (now called Gazebo): Next generation, modular architecture

Gazebo excels for ROS-based robotics development and is the go-to choice for most ROS projects.
