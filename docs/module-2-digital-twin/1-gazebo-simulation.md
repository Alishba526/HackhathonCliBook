---
sidebar_position: 1
---

# Chapter 4: Physics Simulation with Gazebo

## What is Gazebo?

**Gazebo** is a powerful physics simulator that:
- Simulates gravity, friction, and collisions
- Renders 3D environments in real-time
- Integrates with ROS 2 for robot control
- Lets you test algorithms before using real robots

Think of Gazebo as a virtual test lab where you can crash test your code without breaking expensive hardware.

## Gazebo Fundamentals

### Starting Gazebo

```bash
# Install Gazebo Harmonic
sudo apt-get install ignition-gazebo

# Launch Gazebo
gazebo
```

### Creating a Gazebo World

Gazebo uses SDF (Simulation Description Format) files. Here's a simple world:

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="robot_world">
    
    <!-- Physics engine -->
    <physics name="default_physics" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-1 -1 -1</direction>
    </light>
    
    <!-- Your robot will be inserted here -->
    <include>
      <uri>model://humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
    
  </world>
</sdf>
```

## Simulating Your Humanoid Robot

### Launch with ROS 2

Create `launch/gazebo_humanoid.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    robot_desc_dir = get_package_share_directory('robot_description')
    world_file = robot_desc_dir + '/worlds/robot_world.sdf'
    urdf_file = robot_desc_dir + '/urdf/humanoid.urdf'
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),
        
        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        
        # Diff drive controller for movement
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
        ),
        
        # Your custom controller node
        Node(
            package='robot_controller',
            executable='motor_controller',
            output='screen'
        ),
    ])
```

Run it:
```bash
ros2 launch robot_description gazebo_humanoid.launch.py
```

## Sensor Simulation

### Simulating a Camera

Add to your URDF:

```xml
<!-- Camera link -->
<link name="camera">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
</link>

<!-- Mount camera on head -->
<joint name="head_to_camera" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
</joint>
```

### Gazebo Camera Plugin

Add to SDF world file:

```xml
<plugin
    filename="gz-sim-camera-system"
    name="gz::sim::systems::Camera">
</plugin>

<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>camera</topic>
</sensor>
```

### Subscribe to Camera in ROS 2

```python
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process image (detect objects, etc.)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        self.get_logger().info(f'Processed frame: {gray.shape}')
```

## Simulating LiDAR

```xml
<!-- LiDAR link -->
<link name="lidar">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder length="0.08" radius="0.05"/>
    </geometry>
  </visual>
</link>

<joint name="torso_to_lidar" type="fixed">
  <parent link="torso"/>
  <child link="lidar"/>
  <origin xyz="0 0 0.6" rpy="0 0 0"/>
</joint>
```

LiDAR plugin in Gazebo:

```xml
<sensor name="gpu_lidar" type="gpu_lidar">
  <pose>0 0 0.6 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>6.28</max_angle>
      </horizontal>
      <vertical>
        <samples>128</samples>
        <resolution>1</resolution>
        <min_angle>-1.047</min_angle>
        <max_angle>1.047</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.02</resolution>
    </range>
  </ray>
</sensor>
```

## Simulating IMU (Inertial Measurement Unit)

```xml
<!-- IMU link -->
<link name="imu">
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="torso_to_imu" type="fixed">
  <parent link="torso"/>
  <child link="imu"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

IMU plugin:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
  <topic>imu</topic>
</sensor>
```

Subscribe in ROS 2:

```python
from sensor_msgs.msg import Imu

def __init__(self):
    self.imu_sub = self.create_subscription(
        Imu,
        '/imu',
        self.imu_callback,
        10
    )

def imu_callback(self, msg):
    accel = msg.linear_acceleration
    angular_vel = msg.angular_velocity
    
    self.get_logger().info(
        f'Accel: {accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}'
    )
```

## Collision Detection

Gazebo automatically detects collisions. Access them via ROS 2:

```python
from gazebo_msgs.srv import GetContactsInfo

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.client = self.create_client(
            GetContactsInfo,
            '/gazebo/get_contacts'
        )
        
        self.timer = self.create_timer(0.1, self.check_collisions)
    
    def check_collisions(self):
        if not self.client.service_is_ready():
            return
        
        future = self.client.call_async(GetContactsInfo.Request())
        
        def callback(future):
            contacts = future.result().contacts
            if contacts:
                self.get_logger().warn(f'Collision detected! {len(contacts)} contacts')
        
        future.add_done_callback(callback)
```

## Summary

Gazebo provides:
- ✅ **Realistic Physics** - Gravity, friction, inertia
- ✅ **Sensor Simulation** - Cameras, LiDAR, IMU
- ✅ **ROS 2 Integration** - Direct topic communication
- ✅ **Safe Testing** - Test algorithms without hardware risk
- ✅ **Reproducibility** - Deterministic simulations for debugging

Next: Create high-fidelity 3D environments with Unity!
