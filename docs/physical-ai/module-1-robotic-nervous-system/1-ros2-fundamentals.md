---
sidebar_position: 1
---

# Chapter 1: ROS 2 Architecture & Core Concepts

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is the middleware that serves as the "nervous system" of your humanoid robot. It manages communication between all robot components: sensors, processors, and actuators.

### The Nervous System Analogy

Just like your biological nervous system coordinates your brain with your body:
- **Brain** = AI algorithms running on edge computers
- **Spinal cord** = ROS 2 (message passing, coordination)
- **Sensors** = Cameras, LiDAR, IMUs, depth cameras
- **Muscles** = Motors, actuators, grippers

Without ROS 2, these components can't communicate. With ROS 2, they work as one integrated system.

### ROS 2 Core Concepts

#### 1. **Nodes**
A node is an independent process that performs a specific task.

Examples in your humanoid robot:
- Vision processing node (detects objects)
- Motor control node (moves joints)
- Navigation node (plans paths)
- AI inference node (runs LLM)

**Key principle**: Each node is independent. If one crashes, others keep running.

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('Robot node started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. **Topics**
Topics are data streams. One node publishes data, others subscribe.

**Publisher example** (motor sends joint angles):
```python
from geometry_msgs.msg import Twist

self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

msg = Twist()
msg.linear.x = 0.5  # Move forward
self.publisher.publish(msg)
```

**Subscriber example** (sensor reads data):
```python
from sensor_msgs.msg import Image

self.subscription = self.create_subscription(
    Image,
    'camera/image_raw',
    self.image_callback,
    10
)

def image_callback(self, msg):
    # Process camera image
    pass
```

#### 3. **Services**
Services are request-response patterns. One node asks, another replies.

**Service definition**:
```
# GetRobotState.srv
---
bool is_moving
float32 battery_level
```

**Service server**:
```python
srv = self.create_service(GetRobotState, 'get_state', self.handle_get_state)

def handle_get_state(self, request, response):
    response.is_moving = True
    response.battery_level = 0.85
    return response
```

**Service client**:
```python
client = self.create_client(GetRobotState, 'get_state')
future = client.call_async(GetRobotState.Request())
response = future.result()
print(f"Battery: {response.battery_level}")
```

## ROS 2 Executor Pattern

The `rclpy.spin()` function creates an event loop that:
1. Listens for messages on subscribed topics
2. Calls callbacks when data arrives
3. Processes service requests
4. Allows your robot to react in real-time

```python
def main():
    rclpy.init()
    node = MyRobotNode()
    
    # This loop runs forever, processing callbacks
    rclpy.spin(node)
    
    rclpy.shutdown()
```

## Quality of Service (QoS)

ROS 2 allows tuning message delivery guarantees:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable delivery (for critical commands)
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE)
self.create_subscription(Twist, 'cmd_vel', callback, qos)

# Best-effort (for sensor data that's always updating)
qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
self.create_subscription(Image, 'camera/image', callback, qos)
```

## Summary

ROS 2 gives your humanoid robot:
- ✅ **Distributed computing** - Many processes working together
- ✅ **Real-time communication** - Sensors → decisions → actions in milliseconds
- ✅ **Fault isolation** - One failed component doesn't crash everything
- ✅ **Scalability** - Add new nodes without modifying existing ones
- ✅ **Language agnostic** - Mix Python, C++, Rust nodes seamlessly

In the next chapters, you'll build actual ROS 2 applications and create URDF descriptions for humanoid robots.
