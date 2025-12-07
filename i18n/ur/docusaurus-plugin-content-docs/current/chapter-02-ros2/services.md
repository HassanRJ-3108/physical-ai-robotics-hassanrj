---
sidebar_position: 8
---

# Services

Services provide a synchronous, one-to-one communication pattern based on the request-response model.

## How Services Work

- A **client** node sends a request to a **server** node
- The server processes the request and sends back a response
- The client waits for the response (blocking call)

Services are used for short, transactional interactions, such as querying state or triggering actions.

## Service Example

### Server

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('server')
        self.srv = self.create_service(
            AddTwoInts, 'add', self.callback)
    
    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### Client

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('client')
        self.client = self.create_client(AddTwoInts, 'add')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

## Command Line

```bash
# Call a service
ros2 service call /add example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

Services are perfect for request-response interactions.
