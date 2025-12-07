---
sidebar_position: 2
---

# Core Architecture

At its heart, ROS 2 is a distributed system of processes (called "nodes") that communicate with each other to perform complex tasks. Understanding the core architecture is essential for building robust robotic applications.

## The Node-Based Architecture

### What is a Node?

A **node** is the fundamental building block of a ROS 2 system. It is a process that performs a specific task, such as:

- Controlling a motor
- Reading data from a sensor
- Planning a path for the robot to follow
- Visualizing data
- Processing images
- Managing robot state

### Node Characteristics

**Single Responsibility**
- Each node should do one thing well
- Focused functionality
- Easier to debug and maintain

**Independence**
- Nodes run as separate processes
- Can be started and stopped independently
- Failure of one node doesn't crash others

**Communication**
- Nodes communicate via messages
- Well-defined interfaces
- Language-agnostic (C++, Python, etc.)

### Example Nodes in a Robot System

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Camera Driver  │────▶│  Image Processor │────▶│ Object Detector │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                           │
                                                           ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Motor Control  │◀────│  Path Planner    │◀────│  Decision Maker │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

## Distributed System Design

ROS 2 is designed as a distributed system, meaning nodes can run:

- On the same computer
- On different computers connected by network
- On embedded devices
- In containers or virtual machines

### Benefits of Distribution

**Scalability**
- Add more computing power by adding machines
- Distribute processing load
- Scale to robot fleets

**Flexibility**
- Run computationally intensive tasks on powerful servers
- Run real-time control on embedded systems
- Mix and match hardware

**Robustness**
- Redundancy for critical functions
- Graceful degradation if components fail
- Easier testing and development

## Process Management

### Lifecycle Management

ROS 2 provides lifecycle management for nodes:

**Lifecycle States:**
1. **Unconfigured**: Node exists but not ready
2. **Inactive**: Configured but not running
3. **Active**: Fully operational
4. **Finalized**: Shutting down

**Benefits:**
- Controlled startup and shutdown
- State management
- Error recovery
- Deterministic behavior

### Node Composition

Nodes can be composed in different ways:

**Separate Processes** (default)
- Each node runs independently
- Maximum isolation
- Easier debugging

**Single Process (Composition)**
- Multiple nodes in one process
- Lower overhead
- Faster communication
- Better for resource-constrained systems

## Communication Infrastructure

The architecture supports multiple communication patterns:

### Publish-Subscribe (Topics)
- One-to-many communication
- Asynchronous
- Continuous data streams

### Request-Response (Services)
- One-to-one communication
- Synchronous
- Transactional interactions

### Goal-Based (Actions)
- Long-running tasks
- Feedback during execution
- Cancellable

## Namespaces and Remapping

ROS 2 provides powerful tools for organizing and configuring nodes:

### Namespaces

Organize nodes hierarchically:

```
/robot1/camera_driver
/robot1/motor_controller
/robot2/camera_driver
/robot2/motor_controller
```

**Benefits:**
- Avoid name conflicts
- Support multiple robots
- Logical organization

### Remapping

Change topic/service names at runtime:

```bash
ros2 run pkg node --ros-args -r old_name:=new_name
```

**Use Cases:**
- Reuse nodes with different topics
- Connect systems without code changes
- Testing and integration

## Parameters

Nodes can have configurable parameters:

**Parameter Types:**
- Integers, floats, strings, booleans
- Arrays
- Nested structures

**Parameter Sources:**
- Command line
- YAML files
- Launch files
- Runtime reconfiguration

**Example:**
```python
self.declare_parameter('max_speed', 1.0)
max_speed = self.get_parameter('max_speed').value
```

## Logging and Diagnostics

Built-in tools for monitoring and debugging:

### Logging Levels
- **DEBUG**: Detailed information
- **INFO**: General information
- **WARN**: Warning messages
- **ERROR**: Error conditions
- **FATAL**: Critical failures

### Diagnostics
- System health monitoring
- Performance metrics
- Error reporting

## The ROS 2 Architecture Advantage

This architecture provides:

**Modularity**
- Build complex systems from simple components
- Reuse existing nodes
- Easy to extend

**Testability**
- Test nodes in isolation
- Mock interfaces for testing
- Replay recorded data

**Maintainability**
- Clear separation of concerns
- Well-defined interfaces
- Easy to update components

**Scalability**
- Start small, grow as needed
- Distribute across hardware
- Support robot fleets

Understanding this architecture is key to designing effective ROS 2 systems that are robust, scalable, and maintainable.
