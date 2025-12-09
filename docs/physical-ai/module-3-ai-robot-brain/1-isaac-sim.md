---
sidebar_position: 1
---

# Chapter 6: NVIDIA Isaac - The AI Robot Brain

## What is NVIDIA Isaac?

**NVIDIA Isaac** is a comprehensive robotics platform that provides:
- **Isaac Sim** - Photorealistic simulation with synthetic data generation
- **Isaac ROS** - Hardware-accelerated vision and perception algorithms
- **Isaac Manipulator** - Pre-trained models for object manipulation
- **Nav2 Integration** - Path planning for humanoid navigation

## Isaac Sim: Photorealistic Simulation

### Installation

```bash
# Download from NVIDIA (free with registration)
# https://www.nvidia.com/en-us/isaac/

# Or use Docker
docker pull nvcr.io/nvidia/isaac-sim:2024.1

# Run Isaac Sim
docker run --gpus all -it --rm \
  -v ~/isaac_data:/home/user/isaac_data \
  nvcr.io/nvidia/isaac-sim:2024.1
```

### Creating a Humanoid World in Isaac Sim

Isaac Sim uses **USD (Universal Scene Description)** format:

```python
# Create file: create_humanoid_world.py
import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_ground_plane()

# Add robot (pre-built humanoid model)
robot_prim_path = "/World/humanoid"
add_reference_to_stage(
    usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/2024.1/Isaac/Robots/Humanoids/H1/h1.usd",
    prim_path=robot_prim_path
)

# Simulate for 1000 frames
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

Run it:
```bash
python create_humanoid_world.py
```

### Synthetic Data Generation

Isaac Sim generates unlimited labeled training data:

```python
import numpy as np
from PIL import Image
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})  # Headless for speed

import omni
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core import World
from omni.isaac.sensor import Camera

world = World()
world.scene.add_ground_plane()

# Add camera
camera = Camera(
    prim_path="/World/camera",
    resolution=(640, 480),
    translation=[1.0, 1.0, 1.0],
    orientation=euler_angles_to_quat([0, 0.7, 0])
)

# Add some objects to detect
objects = []
for i in range(10):
    # Random cube
    pos = [i * 0.5, np.random.rand() * 2, 0.5]
    add_reference_to_stage(
        usd_path="omniverse://localhost/NVIDIA/Assets/Isaac/2024.1/Isaac/Props/Primitives/Cube.usd",
        prim_path=f"/World/object_{i}",
        position=pos
    )

# Capture and save 1000 annotated frames
for frame in range(1000):
    world.step(render=False)
    
    # Get camera image
    rgb_data = camera.get_rgb()
    depth_data = camera.get_depth()
    
    # Save image
    Image.fromarray((rgb_data * 255).astype(np.uint8)).save(
        f"training_data/rgb_{frame:04d}.png"
    )
    
    # Save depth
    (depth_data * 1000).astype(np.uint16).tobytes() # 16-bit depth
    
    # Save annotations (object positions, labels, etc.)
    # ... annotation code ...

print("Generated 1000 training images with automatic labels!")

simulation_app.close()
```

## Isaac ROS: Hardware-Accelerated Perception

### VSLAM (Visual Simultaneous Localization and Mapping)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path, Odometry
from cv_bridge import CvBridge
import cv2

class IsaacVSLAMNode(Node):
    """
    Visual SLAM using Isaac ROS accelerated kernels
    GPU-accelerated visual odometry from camera images
    """
    
    def __init__(self):
        super().__init__('isaac_vslam')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to camera info (intrinsics)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publish odometry and map
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        self.bridge = CvBridge()
        self.prev_frame = None
        self.path_poses = []
        
        self.get_logger().info("Isaac VSLAM Node started")
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        if self.prev_frame is None:
            self.prev_frame = frame
            return
        
        # Feature detection (ORB - fast, GPU-friendly)
        orb = cv2.ORB_create(nfeatures=500)
        kp1, des1 = orb.detectAndCompute(self.prev_frame, None)
        kp2, des2 = orb.detectAndCompute(frame, None)
        
        if des1 is None or des2 is None:
            return
        
        # Feature matching
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Extract matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches[:20]])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches[:20]])
        
        # Compute Essential Matrix (camera motion)
        E, mask = cv2.findEssentialMat(pts1, pts2)
        
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2)
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "camera"
        
        # Position
        odom_msg.pose.pose.position.x = float(t[0])
        odom_msg.pose.pose.position.y = float(t[1])
        odom_msg.pose.pose.position.z = float(t[2])
        
        self.odometry_pub.publish(odom_msg)
        
        self.prev_frame = frame
    
    def camera_info_callback(self, msg):
        """Store camera intrinsics for VSLAM"""
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacVSLAMNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Object Detection with Isaac ROS Perception

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from cv_bridge import CvBridge
import torch
from torchvision import models, transforms
import cv2

class IsaacObjectDetector(Node):
    """
    GPU-accelerated object detection using YOLOv8
    Optimized with NVIDIA TensorRT
    """
    
    def __init__(self):
        super().__init__('isaac_detector')
        
        # Load YOLOv8 model (optimized for Jetson)
        self.model = torch.hub.load(
            'ultralytics/yolov8',
            'custom',
            path='yolov8n.pt',  # nano model for speed
            force_reload=False
        )
        
        # Set to GPU
        self.model.to('cuda')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.detect_callback,
            10
        )
        
        # Publish detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("Object Detector initialized (GPU-accelerated)")
    
    def detect_callback(self, msg):
        """Detect objects in image"""
        
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run inference
        with torch.no_grad():
            results = self.model(frame)
        
        # Create detection array
        detections = Detection2DArray()
        detections.header.stamp = msg.header
        detections.header.frame_id = "camera"
        
        # Extract bounding boxes
        for *box, conf, cls in results.xyxy[0]:
            det = Detection2D()
            det.results[0].id = str(int(cls))
            det.results[0].score = float(conf)
            
            # Bounding box
            x1, y1, x2, y2 = box
            det.bbox.center.x = float((x1 + x2) / 2)
            det.bbox.center.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            
            detections.detections.append(det)
        
        # Publish
        self.detection_pub.publish(detections)
        
        self.get_logger().info(
            f"Detected {len(detections.detections)} objects"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IsaacObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Path Planning with Nav2

### Setup Nav2 for Humanoid Navigation

```bash
sudo apt install ros-humble-nav2 ros-humble-nav2-bringup
```

Create `nav2_humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # Nav2 Costmap Server
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='global_costmap',
            parameters=[{
                'use_sim_time': True,
                'global_frame': 'map',
                'robot_base_frame': 'base_link',
                'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                'inflation_layer.inflation_radius': 0.5,
            }]
        ),
        
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

### Use Nav2 to Navigate

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path

class HumanoidNavigator(Node):
    """Navigate humanoid robot using Nav2"""
    
    def __init__(self):
        super().__init__('humanoid_navigator')
        self.navigator = BasicNavigator()
    
    def navigate_to(self, x, y, theta):
        """Send robot to target position"""
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        
        self.navigator.setInitialPose(PoseStamped())
        self.navigator.goToPose(goal_pose)
        
        self.get_logger().info(f'Navigating to ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigator()
    
    # Navigate robot
    node.navigate_to(x=5.0, y=3.0, theta=0.0)
    
    # Wait for completion
    while not node.navigator.isNavComplete():
        rclpy.spin_once(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

NVIDIA Isaac provides:
- ✅ **Photorealistic simulation** - Isaac Sim for testing
- ✅ **Synthetic data** - Unlimited labeled training data
- ✅ **GPU acceleration** - Fast perception algorithms
- ✅ **Ready models** - Pre-trained detection, SLAM, control
- ✅ **Path planning** - Nav2 integration for navigation

Next: Combine language understanding with robotics using Vision-Language-Action models!
