---
sidebar_position: 3
---

# Chapter 3: Your First ROS 2 Node

## Setting Up Your Environment

### Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt-get update
sudo curl -sSL https://raw.githubusercontent.com/ros/ros.key | sudo apt-key add -
sudo apt-get install -y ros-humble-desktop-full

# Add ROS 2 to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Create a ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source setup
source install/setup.bash
```

## Creating Your First Package

A **package** is a collection of ROS 2 code and resources.

```bash
cd ~/ros2_ws/src

# Create package with Python
ros2 pkg create my_first_package --build-type ament_python --dependencies rclpy std_msgs

# Directory structure created:
# my_first_package/
# ├── package.xml
# ├── setup.py
# ├── my_first_package/
# │   └── __init__.py
# └── resource/
```

## Building a Simple Publisher

A **publisher** sends data to a topic.

### Example: Motor Velocity Publisher

```python
# ~/ros2_ws/src/my_first_package/my_first_package/motor_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        
        # Create publisher
        # Topic name: /motor/velocity
        # Message type: Float64MultiArray
        # Queue size: 10 messages
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/motor/velocity',
            10
        )
        
        # Create timer - publish every 100ms (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [
            0.5 * (self.counter % 20) / 10,  # Joint 1: 0 to 1
            0.3,                               # Joint 2: constant
            -0.2,                              # Joint 3: constant
            0.1 * (-1)**(self.counter % 2)   # Joint 4: oscillating
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing motor commands: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building a Simple Subscriber

A **subscriber** listens to messages on a topic.

### Example: Motor Feedback Subscriber

```python
# ~/ros2_ws/src/my_first_package/my_first_package/motor_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        
        # Create subscriber
        # Topic name: /motor/feedback
        # Message type: Float64MultiArray
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/motor/feedback',
            self.feedback_callback,
            10
        )

    def feedback_callback(self, msg):
        # Called whenever message arrives on topic
        self.get_logger().info(f'Received motor feedback: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Publisher and Subscriber

### Terminal 1: Start publisher

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_first_package motor_publisher
```

### Terminal 2: Start subscriber

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_first_package motor_subscriber
```

## Creating a Service (Request-Reply)

### Service Definition

```yaml
# ~/ros2_ws/src/my_first_package/srv/ComputeIK.srv

# Request
geometry_msgs/Point end_effector_position
---
# Response
float64[] joint_angles
bool success
```

### Service Server

```python
# ~/ros2_ws/src/my_first_package/my_first_package/ik_server.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import numpy as np

# Need to create custom message - simulating here
class IKResponse:
    def __init__(self):
        self.joint_angles = []
        self.success = False

class IKServer(Node):
    def __init__(self):
        super().__init__('ik_server')
        
        # Create service
        self.service = self.create_service(
            self.compute_ik_srv,
            '/robot/compute_ik',
            self.compute_ik_callback
        )
        self.get_logger().info('IK Service ready')

    def compute_ik_callback(self, request, response):
        # Simple inverse kinematics (mock implementation)
        x = request.end_effector_position.x
        y = request.end_effector_position.y
        z = request.end_effector_position.z
        
        # Mock IK calculation
        r = np.sqrt(x**2 + y**2)
        joint_angles = [
            np.arctan2(y, x),           # Base rotation
            np.arctan2(z, r),           # Shoulder
            np.arctan2(z, r) * 0.5,    # Elbow
            0.0                         # Wrist
        ]
        
        response.joint_angles = joint_angles
        response.success = True
        
        self.get_logger().info(f'Computed IK: {joint_angles}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = IKServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
# ~/ros2_ws/src/my_first_package/my_first_package/ik_client.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import asyncio

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')

    async def request_ik(self, x, y, z):
        # Note: Simplified version - full implementation needs custom message
        self.get_logger().info(f'Requesting IK for position: ({x}, {y}, {z})')
        
        # In real implementation:
        # future = client.call_async(request)
        # await future
        # result = future.result()

async def main(args=None):
    rclpy.init(args=args)
    node = IKClient()
    
    # Request IK solution
    await node.request_ik(0.3, 0.2, 0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
```

## Debugging ROS 2 Applications

### List Active Nodes

```bash
ros2 node list
# Output:
# /motor_publisher
# /motor_subscriber
```

### Inspect Topics

```bash
# List all topics
ros2 topic list

# Show topic information
ros2 topic info /motor/velocity

# Echo topic data
ros2 topic echo /motor/velocity

# Publish to topic manually
ros2 topic pub /motor/velocity std_msgs/msg/Float64MultiArray "{data: [0.5, 0.3, -0.2, 0.1]}"
```

### Inspect Services

```bash
# List all services
ros2 service list

# Show service information
ros2 service info /robot/compute_ik

# Call service
ros2 service call /robot/compute_ik ...
```

### Monitor System

```bash
# Real-time graph of node/topic connections
rqt_graph

# Console viewer
rqt_console

# Node dashboard
rqt
```

## ROS 2 Launch Files

For complex systems, use **launch files** to start multiple nodes:

```python
# ~/ros2_ws/src/my_first_package/launch/motor_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='motor_publisher',
            name='motor_pub'
        ),
        Node(
            package='my_first_package',
            executable='motor_subscriber',
            name='motor_sub'
        ),
        Node(
            package='my_first_package',
            executable='ik_server',
            name='ik_service'
        ),
    ])
```

### Run launch file

```bash
ros2 launch my_first_package motor_system.launch.py
```

## Key Takeaways

✅ **Nodes are executable processes with specific tasks**  
✅ **Publishers send data continuously to topics**  
✅ **Subscribers listen to topics and react to messages**  
✅ **Services provide request-reply communication**  
✅ **Launch files simplify starting complex systems**  
✅ **ROS 2 tools help debug and visualize systems**  

## Exercises

1. Modify the publisher to read motor velocities from a file
2. Create a subscriber that saves received data to a CSV file
3. Build a service that computes forward kinematics
4. Write a launch file that starts 3 different subscriber nodes

---

**Next**: [Chapter 4: URDF - Describing Your Robot](/docs/physical-ai/chapter-4-urdf)
