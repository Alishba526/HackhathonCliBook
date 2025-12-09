---
sidebar_position: 2
---

# Chapter 2: ROS 2 Architecture & Core Concepts

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is the industry-standard middleware for robotics development. It provides:

- **Inter-process communication** - Nodes exchange data
- **Hardware abstraction** - Control different robots with same code
- **Message-passing system** - Loosely coupled components
- **Development tools** - Visualization, simulation, debugging

Think of ROS 2 as the "nervous system" of a robot—enabling different components (motors, sensors, processors) to communicate efficiently.

## ROS 2 vs ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | TCP/IP only | DDS (more flexible) |
| **Real-time** | Not guaranteed | Real-time capable |
| **Multi-robot** | Complex | Native support |
| **Security** | None | Built-in |
| **Windows Support** | Poor | Excellent |
| **Production Ready** | Mature but legacy | Modern standard |

**ROS 2 is the industry standard for new projects.**

## The ROS 2 Ecosystem

```
┌─────────────────────────────────────────────┐
│         Application Layer                   │
│  (Your robot control code)                  │
├─────────────────────────────────────────────┤
│  ROS 2 Core Libraries                       │
│  ├─ rclpy (Python client)                   │
│  ├─ rclcpp (C++ client)                     │
│  └─ rcl (middleware abstraction)            │
├─────────────────────────────────────────────┤
│  Middleware (DDS - Data Distribution Service) │
│  (Handles all node communication)           │
├─────────────────────────────────────────────┤
│  OS Layer (Ubuntu 22.04)                    │
└─────────────────────────────────────────────┘
```

## Core Concepts

### 1. Nodes

A **node** is an executable process that performs a specific task.

**Example nodes in a robot system**:
- Motor controller node - Sends commands to motors
- Camera driver node - Reads images from camera
- Perception node - Processes images for object detection
- Navigation node - Plans and executes robot movement
- State machine node - Coordinates overall robot behavior

**Key principle**: Each node is independent and can be started/stopped without affecting others.

### 2. Topics

A **topic** is a named communication channel where nodes publish or subscribe to data.

**One-to-many communication pattern**:

```
Publisher Node                 ROS 2 Topic              Subscriber Nodes
(Camera)         ───publish──→ /camera/image ←─subscribe─ (Vision)
                                                        (Display)
                                                        (Tracking)
```

**Multiple subscribers can listen to the same topic.**

**Example topics**:
- `/camera/image` - RGB images
- `/sensor/lidar` - LiDAR point clouds
- `/motor/position` - Joint angles
- `/navigation/goal` - Target positions

### 3. Services

A **service** is a request-reply communication pattern—useful for one-time computations.

```
Client Node              ROS 2 Service               Server Node
(Main Control)  ─request→ /compute/inverse_kinematics ─reply→ (IK Solver)
```

Unlike topics (fire-and-forget), services **wait for a response**.

**Example services**:
- `/robot/grasp_object` - Request: object ID | Response: success
- `/planner/path_plan` - Request: start, goal | Response: path
- `/camera/take_snapshot` - Request: filename | Response: status

### 4. Actions

An **action** is a service with progress feedback—used for long-running tasks.

```
Client                  ROS 2 Action                 Server
(Commander) ─goal→ /robot/move_to_location ←─feedback─ (Navigator)
                                            ←─result─
```

Actions provide:
- Initial goal
- Periodic feedback (progress)
- Final result

**Example actions**:
- `/robot/navigate` - Navigate to goal with continuous feedback
- `/arm/pick_and_place` - Pick object from A, place at B
- `/autonomous_humanoid/walk` - Walk to destination

## Message Types

Messages are the data packets sent through topics and services.

### Standard Message Types

```python
# Geometry messages
from geometry_msgs.msg import Point, Pose, Twist

# Sensor messages
from sensor_msgs.msg import Image, PointCloud2, Imu

# Standard messages
from std_msgs.msg import String, Float64, Bool
```

### Example: Motor Command Message

```python
from std_msgs.msg import Float64MultiArray

# Send motor velocities
msg = Float64MultiArray()
msg.data = [0.5, 0.3, -0.2, 0.1]  # 4 joint velocities

publisher.publish(msg)
```

### Example: Sensor Data Message

```python
from sensor_msgs.msg import Imu

msg = Imu()
msg.header.frame_id = "base_link"
msg.linear_acceleration.x = 9.81  # m/s²
msg.angular_velocity.z = 0.5      # rad/s

publisher.publish(msg)
```

## Coordinate Frames & Transforms

Every robot needs a **frame of reference**:

```
world
  ├─ base_link (robot origin)
  │   ├─ camera_link (camera mounted on robot)
  │   └─ lidar_link (LiDAR sensor)
  └─ base_footprint (ground projection)
```

**tf2** library handles transformations between frames:

```python
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

# Convert point from camera frame to world frame
tf_buffer = Buffer()
listener = TransformListener(tf_buffer)

# Get transform from camera to world
transform = tf_buffer.lookup_transform("world", "camera_link", Time())
```

## QoS (Quality of Service)

ROS 2 allows fine-tuning how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable, in-order delivery (slower)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history_depth=10
)

# Best-effort delivery (faster, for video streams)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history_depth=1
)

publisher = node.create_publisher(String, "topic", qos_profile)
```

**Use RELIABLE for critical commands, BEST_EFFORT for sensor streams.**

## ROS 2 Distributions

| Distribution | Release Year | Support |
|-------------|------------|---------|
| Humble | 2022 | LTS (until 2027) |
| Iron | 2023 | Standard |
| Jazzy | 2024 | Standard |

**We recommend ROS 2 Humble** for stability and long-term support.

## Key Takeaways

✅ **ROS 2 provides middleware for robot communication**  
✅ **Nodes are independent processes with specific tasks**  
✅ **Topics enable one-to-many publish-subscribe communication**  
✅ **Services enable request-reply communication**  
✅ **Actions are for long-running tasks with feedback**  
✅ **Coordinate frames track spatial relationships**  

## Next Chapter

We'll implement these concepts in Python by building our first ROS 2 nodes.

---

**Next**: [Chapter 3: Your First ROS 2 Node](/docs/physical-ai/chapter-3-first-node)
