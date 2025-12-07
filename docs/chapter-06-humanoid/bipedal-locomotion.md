---
sidebar_position: 3
---

# Bipedal Locomotion

Walking on two legs is a remarkably complex task that humans perform with ease, but it is one of the biggest challenges in humanoid robotics. Bipedal locomotion requires a delicate balance of control, perception, and planning.

## The Challenge of Balance

Unlike wheeled or quadruped robots, bipedal robots are inherently unstable. They must constantly maintain balance while moving, which requires:

- Precise control of joint torques
- Real-time sensing of orientation and forces
- Rapid response to disturbances
- Coordination of many degrees of freedom

## The Zero Moment Point (ZMP)

The Zero Moment Point (ZMP) is a key concept in bipedal locomotion. It is the point on the ground where the net moment of the inertial forces and the gravity forces has no horizontal component.

### ZMP Stability Criterion

To maintain balance, the ZMP must always be kept within the **support polygon**, which is the area on the ground formed by the robot's feet.

**Single Support**: When one foot is on the ground, the support polygon is that foot

**Double Support**: When both feet are on the ground, the support polygon is the area between them

**If ZMP moves outside the support polygon**: The robot will fall over

## Generating Stable Walking Gaits

There are several approaches to generating stable walking gaits for humanoid robots:

### 1. Trajectory-Based Methods

Pre-calculate trajectories for the robot's center of mass and feet that will keep the ZMP within the support polygon.

**Process**:
1. Define desired walking parameters (step length, speed)
2. Generate CoM and foot trajectories
3. Ensure ZMP stays in support polygon
4. Use inverse kinematics to find joint angles
5. Execute trajectory with joint controllers

**Advantages**: Predictable, well-understood

**Disadvantages**: Less adaptive to disturbances

### 2. Model-Based Methods

Use a dynamic model of the robot to calculate the joint torques required to maintain balance in real-time.

**Model Predictive Control (MPC)**:
- Predict future state of robot
- Optimize control inputs over time horizon
- Execute first control action
- Repeat at each time step

**Advantages**: Can handle disturbances, more robust

**Disadvantages**: Computationally intensive

### 3. Machine Learning-Based Methods

Reinforcement learning can be used to train a neural network to control the robot's walking gait.

**Process**:
1. Define reward function (walking speed, stability, energy)
2. Train in simulation with many iterations
3. Transfer learned policy to real robot
4. Fine-tune on real hardware

**Advantages**: Can discover novel gaits, adaptive

**Disadvantages**: Requires extensive training, sim-to-real gap

## Walking Phases

A typical walking cycle consists of:

**Stance Phase**: Foot is on the ground, supporting the robot

**Swing Phase**: Foot is in the air, moving forward

**Double Support**: Brief period when both feet are on ground

**Single Support**: Only one foot on ground (most unstable)

## Advanced Locomotion

Beyond basic walking, humanoid robots are developing:

- **Running**: Both feet off ground simultaneously
- **Jumping**: Explosive movements with flight phase
- **Climbing stairs**: Navigating vertical obstacles
- **Rough terrain**: Adapting to uneven surfaces
- **Recovery**: Regaining balance after pushes

Bipedal locomotion remains one of the most challenging and active areas of research in humanoid robotics.
