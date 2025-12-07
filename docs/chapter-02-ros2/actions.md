---
sidebar_position: 9
---

# Actions

Actions are used for long-running, asynchronous tasks that provide feedback during their execution.

## How Actions Work

- An **action client** sends a **goal** to an **action server**
- The action server starts executing the goal and provides periodic **feedback** to the client
- When the task is complete, the server sends a final **result** to the client
- The action client can also **cancel** the goal at any time

Actions are ideal for tasks like navigation, manipulation, or following trajectories.

## Action Structure

An action consists of three parts:

1. **Goal**: What you want to achieve
2. **Feedback**: Progress updates during execution
3. **Result**: Final outcome when complete

## Example

```python
from rclpy.action import ActionServer

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci',
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        # Execute task and send feedback
        feedback_msg = Fibonacci.Feedback()
        # ... send feedback
        goal_handle.publish_feedback(feedback_msg)
        
        # Return result
        goal_handle.succeed()
        result = Fibonacci.Result()
        return result
```

Actions provide the perfect balance for long-running tasks with progress tracking.
