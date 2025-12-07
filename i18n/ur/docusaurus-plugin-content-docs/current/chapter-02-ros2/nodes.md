---
sidebar_position: 4
---

# Nodes

A node is the fundamental building block of a ROS 2 system. It is a process that performs a specific task, such as controlling a motor, reading sensor data, or planning paths.

Nodes are typically written in C++ or Python using the ROS 2 client libraries (`rclcpp` and `rclpy`). They can be combined to create complex robot behaviors.

## Creating a Node in Python

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node has been started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Characteristics

**Single Responsibility**: Each node should perform one task well

**Independence**: Nodes run as separate processes

**Communication**: Nodes communicate via well-defined interfaces

Nodes are modular, reusable, and form the building blocks of ROS 2 systems.
