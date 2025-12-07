---
sidebar_position: 7
---

# Topics

Topics are the most common communication pattern in ROS 2. They provide an asynchronous, one-to-many communication channel based on the publish-subscribe model.

## How Topics Work

- A node can **publish** messages to a topic
- Any number of other nodes can **subscribe** to that topic to receive the messages

Topics are ideal for continuous data streams, such as sensor data (e.g., camera images, LiDAR scans) or robot state information (e.g., joint positions, odometry).

## Message Types

Each topic has a specific **message type**, which defines the structure of the data being sent.

### Common Message Types

```bash
# Standard messages
std_msgs/String
std_msgs/Int32

# Sensor messages
sensor_msgs/Image
sensor_msgs/LaserScan

# Geometry messages
geometry_msgs/Twist
geometry_msgs/Pose
```

## Publishing Example

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.callback)
    
    def callback(self):
        msg = String()
        msg.data = 'Hello!'
        self.pub.publish(msg)
```

## Subscribing Example

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(
            String, 'topic', self.callback, 10)
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
```

Topics are the backbone of ROS 2 communication.
