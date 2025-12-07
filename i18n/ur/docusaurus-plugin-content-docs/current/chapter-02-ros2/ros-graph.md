---
sidebar_position: 5
---

# The ROS Graph

The ROS Graph is the network of ROS 2 nodes and their connections. It is a conceptual representation of how the different parts of the system are communicating with each other.

## Visualizing the Graph

Tools like `rqt_graph` can be used to visualize the ROS Graph and debug the communication between nodes.

```bash
# Install rqt_graph
sudo apt install ros-humble-rqt-graph

# Run it
rqt_graph
```

## Graph Components

The graph consists of:
- **Nodes**: Processes performing tasks
- **Topics**: Named buses for messages
- **Services**: Request-response endpoints
- **Actions**: Long-running task endpoints
- **Parameters**: Configuration values

## Introspection Tools

```bash
# List all topics
ros2 topic list

# List all services
ros2 service list

# List all nodes
ros2 node list
```

Understanding the ROS Graph helps you design better architectures and debug communication issues.
