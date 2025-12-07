---
sidebar_position: 10
---

# Python Integration (rclpy)

`rclpy` is the official Python client library for ROS 2. It provides a Pythonic interface to all the core ROS 2 concepts.

## Why Python for ROS 2?

**Advantages:**
- Rapid prototyping
- Easy to learn
- Rich ecosystem of libraries (NumPy, OpenCV, PyTorch)
- Great for AI/ML integration

## Hello World Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Concepts

### Initialization
```python
rclpy.init(args=args)
rclpy.shutdown()
```

### Spinning
```python
rclpy.spin(node)  # Spin indefinitely
rclpy.spin_once(node)  # Spin once
```

### Logging
```python
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
```

Python's flexibility makes it perfect for rapid development and AI integration.
