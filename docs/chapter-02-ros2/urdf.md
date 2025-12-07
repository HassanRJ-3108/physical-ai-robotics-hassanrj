---
sidebar_position: 11
---

# URDF (Unified Robot Description Format)

The Unified Robot Description Format (URDF) is an XML format for describing the physical structure of a robot. It is essential for modeling, simulation, and visualization.

## What URDF Defines

- **Links**: The rigid parts of the robot
- **Joints**: The connections between the links
- **Visual properties**: The shape and appearance
- **Collision properties**: Shapes for collision detection
- **Inertial properties**: Mass and inertia

## Example URDF

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="1 1 0.5"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0" ixz="0"
               iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Joint Types

- **fixed**: No movement
- **revolute**: Rotating with limits
- **continuous**: Rotating without limits
- **prismatic**: Sliding joint

## Using URDF in ROS 2

```bash
# Visualize in RViz
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat robot.urdf)"
rviz2
```

URDF is essential for any robotics application requiring a robot model.
