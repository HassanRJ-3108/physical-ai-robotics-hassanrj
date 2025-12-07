---
sidebar_position: 6
---

# Communication Patterns

ROS 2 provides several communication patterns for nodes to exchange data. Choosing the right pattern for your use case is crucial for building efficient systems.

## Overview

ROS 2 supports three main communication patterns:

1. **Topics** - Publish/Subscribe (one-to-many, asynchronous)
2. **Services** - Request/Response (one-to-one, synchronous)
3. **Actions** - Goal-based (long-running, with feedback)

## When to Use Each Pattern

### Use Topics When:
- Continuous data streams (sensor data, robot state)
- Multiple subscribers need the same data
- Fire-and-forget messaging
- High-frequency updates

### Use Services When:
- Short, transactional interactions
- Request-response pattern
- Querying current state
- Triggering one-time actions

### Use Actions When:
- Long-running tasks
- Need progress feedback
- Task can be cancelled
- Final result is important

## Comparison

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| Pattern | Pub/Sub | Req/Res | Goal-based |
| Direction | One-to-many | One-to-one | One-to-one |
| Timing | Async | Sync | Async |
| Feedback | No | No | Yes |
| Cancellable | No | No | Yes |

Understanding these patterns is essential for designing effective ROS 2 systems.
