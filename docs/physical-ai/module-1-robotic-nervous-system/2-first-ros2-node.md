---
sidebar_position: 2
---

# Chapter 2: Your First ROS 2 Node

## Building Your First Node

Let's create a simple robot controller that publishes velocity commands to move a humanoid robot forward.

### Setup

```bash
# Install ROS 2 (if not already installed)
sudo apt install ros-humble-desktop

# Create a workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws

# Create a package
ros2 pkg create --build-type ament_python robot_controller
cd robot_controller
```

### Create Your First Node

Create `robot_controller/robot_controller/motor_controller.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MotorController(Node):
    """Controls the humanoid robot's movement"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Create a publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.get_logger().info('Motor Controller initialized')
        self.get_logger().info('Publishing to /cmd_vel topic')
        
        # Timer to send commands every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.counter = 0
    
    def publish_velocity(self):
        """Send movement commands to the robot"""
        
        # Create a Twist message (standard ROS 2 velocity command)
        msg = Twist()
        
        # Linear velocity (move forward 0.5 m/s)
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Angular velocity (rotate slowly)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1  # Rotate 0.1 rad/s
        
        # Publish the message
        self.publisher.publish(msg)
        
        self.counter += 1
        self.get_logger().info(
            f'Published velocity command #{self.counter}: '
            f'forward={msg.linear.x} m/s, rotate={msg.angular.z} rad/s'
        )

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create the node
    node = MotorController()
    
    # Keep the node running
    rclpy.spin(node)
    
    # Shutdown when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Update setup.py

Edit `setup.py` to include your node:

```python
entry_points={
    'console_scripts': [
        'motor_controller = robot_controller.motor_controller:main',
    ],
},
```

### Build and Run

```bash
# Go to workspace root
cd ~/robot_ws

# Build the package
colcon build

# Source the setup script
source install/setup.bash

# Run your node
ros2 run robot_controller motor_controller
```

**Output:**
```
[INFO] Motor Controller initialized
[INFO] Publishing to /cmd_vel topic
[INFO] Published velocity command #1: forward=0.5 m/s, rotate=0.1 rad/s
[INFO] Published velocity command #2: forward=0.5 m/s, rotate=0.1 rad/s
```

## Monitoring Topics with ros2 CLI

In another terminal, watch the published messages:

```bash
# See all topics
ros2 topic list

# See message frequency and data
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

## Advanced: Subscribing to Sensor Data

Create a sensor subscriber node:

```python
from sensor_msgs.msg import Image, Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Subscribe to IMU data (Inertial Measurement Unit)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    
    def image_callback(self, msg):
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}'
        )
    
    def imu_callback(self, msg):
        self.get_logger().info(
            f'Accel: x={msg.linear_acceleration.x:.2f} m/s²'
        )
```

## Summary

You now understand:
- ✅ How to create a ROS 2 Node
- ✅ How to publish messages to topics
- ✅ How to subscribe to topics
- ✅ The ROS 2 execution model with `rclpy.spin()`
- ✅ Message types (Twist for velocities, Image for cameras, Imu for sensors)

Next: Learn URDF to describe humanoid robot structure!
