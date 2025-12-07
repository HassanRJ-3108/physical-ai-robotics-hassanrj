---
sidebar_position: 3
---

# DDS (Data Distribution Service)

Unlike the original ROS, which used a custom communication protocol, ROS 2 is built on top of DDS. DDS is an industry-standard middleware for real-time and embedded systems. It provides a publish-subscribe communication model that is decentralized, scalable, and highly reliable.

## What is DDS?

**Data Distribution Service (DDS)** is an Object Management Group (OMG) standard for data-centric publish-subscribe messaging. It's used in mission-critical systems across aerospace, defense, automotive, and industrial automation.

### Key Features

**Decentralized Architecture**
- No central broker or master
- Peer-to-peer communication
- Automatic discovery
- No single point of failure

**Real-Time Performance**
- Low latency (microseconds)
- Deterministic behavior
- Priority-based delivery
- Deadline guarantees

**Scalability**
- Thousands of nodes
- High-bandwidth data streams
- Efficient multicast
- Optimized for large systems

**Reliability**
- Guaranteed delivery options
- Fault tolerance
- Automatic recovery
- Data persistence

## Why ROS 2 Uses DDS

By using DDS, ROS 2 inherits many powerful features:

### 1. Decentralized Discovery

Nodes can automatically discover each other on the network without a central master.

**How it works:**
- Nodes announce their presence via multicast
- Other nodes discover and connect automatically
- No configuration needed
- Works across network segments

**Benefits:**
- No ROS Master (unlike ROS 1)
- More robust (no single point of failure)
- Easier deployment
- Better for distributed systems

### 2. Quality of Service (QoS)

ROS 2 provides a rich set of QoS policies that allow developers to fine-tune the communication between nodes for different use cases.

**QoS Policies:**

**Reliability**
- **Best Effort**: Fast, may lose messages (sensor data)
- **Reliable**: Guaranteed delivery (commands, state)

**Durability**
- **Volatile**: Only for current subscribers
- **Transient Local**: Late joiners get recent messages

**History**
- **Keep Last N**: Store N most recent messages
- **Keep All**: Store all messages (until limits)

**Deadline**
- Expected update rate
- Detect slow publishers

**Lifespan**
- How long messages are valid
- Automatic expiration

**Example Use Cases:**

```python
# Sensor data: fast, can tolerate loss
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Commands: must be reliable
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL
)
```

### 3. Interoperability

Different DDS implementations from different vendors can interoperate with each other.

**Supported DDS Vendors:**
- **eProsima Fast DDS** (default)
- **RTI Connext DDS**
- **Eclipse Cyclone DDS**
- **GurumNetworks GurumDDS**

**Benefits:**
- Choose best implementation for your needs
- Not locked into single vendor
- Commercial support available
- Mix implementations in same system

## DDS Concepts in ROS 2

### Topics and Data Writers/Readers

In DDS terminology:
- **Publisher** → Data Writer
- **Subscriber** → Data Reader
- **Topic** → Topic (same concept)

ROS 2 abstracts these details, but understanding them helps with advanced use cases.

### Domains

DDS uses **domain IDs** to separate communication:

```bash
# Set domain ID
export ROS_DOMAIN_ID=42
```

**Use Cases:**
- Isolate multiple robot systems
- Separate development and production
- Avoid interference between teams

**Domain IDs:** 0-232 (0 is default)

### Discovery

DDS discovery happens in two phases:

**1. Participant Discovery**
- Nodes announce themselves
- Exchange endpoint information
- Build network topology

**2. Endpoint Discovery**
- Match publishers and subscribers
- Establish connections
- Negotiate QoS

## Performance Considerations

### Network Efficiency

**Multicast**
- One message to multiple subscribers
- Reduces network traffic
- Requires network support

**Shared Memory**
- Zero-copy for same-host communication
- Extremely fast
- Automatic when available

**Fragmentation**
- Large messages split automatically
- Reassembled at receiver
- Configurable limits

### Tuning DDS

Advanced users can tune DDS for specific scenarios:

**Transport Selection**
- UDP (default, fast)
- TCP (reliable, slower)
- Shared memory (fastest, same host)

**Buffer Sizes**
- Send/receive buffers
- History depth
- Resource limits

**Threading**
- Dedicated threads for I/O
- Priority configuration
- CPU affinity

## Security

DDS provides built-in security features:

**DDS Security**
- Authentication (who can join)
- Access control (who can publish/subscribe)
- Encryption (protect data in transit)
- Signing (verify message integrity)

**ROS 2 Integration**
- SROS2 (Secure ROS 2)
- Certificate-based authentication
- Fine-grained permissions
- Encrypted communication

## Monitoring DDS

Tools for inspecting DDS communication:

**ROS 2 Tools**
```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /topic_name

# Monitor bandwidth
ros2 topic bw /topic_name

# Echo messages
ros2 topic echo /topic_name
```

**DDS-Specific Tools**
- Wireshark (with DDS plugin)
- Vendor-specific tools
- Performance analyzers

## Common DDS Issues and Solutions

### Discovery Problems

**Issue**: Nodes can't find each other

**Solutions:**
- Check firewall settings
- Verify network connectivity
- Use same domain ID
- Check multicast support

### Performance Issues

**Issue**: High latency or low throughput

**Solutions:**
- Tune QoS settings
- Enable shared memory
- Adjust buffer sizes
- Use appropriate reliability

### Resource Exhaustion

**Issue**: Running out of memory or connections

**Solutions:**
- Limit history depth
- Configure resource limits
- Clean up unused nodes
- Monitor system resources

## The Power of DDS

DDS gives ROS 2:

✅ **No single point of failure**
✅ **Real-time capabilities**
✅ **Flexible QoS for different use cases**
✅ **Proven in mission-critical systems**
✅ **Vendor choice and support**
✅ **Built-in security**
✅ **Excellent performance**

Understanding DDS helps you leverage ROS 2's full potential and build robust, high-performance robotic systems.
