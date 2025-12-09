---
sidebar_position: 5
---

# Chapter 5: Physics Simulation with Gazebo

## Introduction to Gazebo

**Gazebo** is the industry-standard physics simulator for robotics. It provides:
- Accurate rigid-body physics simulation
- Multi-robot simulation support
- Sensor simulation (cameras, LiDAR, IMUs)
- Plugin architecture for extending functionality
- Integration with ROS 2

### Why Simulate?

```
Train in Simulation (Fast, Safe, Cheap)
         ↓
         ↓ (Sim-to-Real Transfer)
         ↓
Deploy to Real Robots (Slow, Risky, Expensive)
```

**Benefits of simulation**:
- 1000x faster than real-time (train in seconds)
- No risk of hardware damage
- Reproducible experiments
- Easy parameter tuning
- Cheap and accessible

## Setting Up Gazebo

### Installation

```bash
# Install Gazebo and ROS 2 integration
sudo apt-get install -y gazebo-harmonic
sudo apt-get install -y ros-humble-gazebo-ros2-control
```

### Launch Gazebo

```bash
# Start Gazebo with empty world
gazebo

# Or via ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

## Gazebo Worlds

A **world** file (`.world`) defines:
- Physics engine settings
- Environmental properties (gravity, friction)
- Objects in the scene
- Lighting

### Basic World File

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="robot_world">
    
    <!-- Physics engine -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Sun for lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

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
          </material>
        </visual>
      </link>
    </model>

    <!-- A box obstacle -->
    <model name="obstacle">
      <pose>5 5 0.25 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Sensor Simulation

### Camera Simulation

```xml
<link name="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.01</near>
        <far>100</far>
      </clip>
    </camera>
    
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
        <remapping>~/image_raw:=image_raw</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</link>
```

### LiDAR Simulation

```xml
<link name="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30</max>
        <resolution>0.1</resolution>
      </range>
    </ray>
    
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_frame</frame_name>
    </plugin>
  </sensor>
</link>
```

### IMU Simulation

```xml
<link name="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</link>
```

## Actuator Control

### Motor Controller Plugin

```xml
<plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
  <ros>
    <namespace>/</namespace>
  </ros>
  <robot_param>robot_description</robot_param>
  <robot_sim_type>gazebo_ros2_control/GazeboSystem</robot_sim_type>
</plugin>
```

## Running Gazebo with ROS 2

### Launch File

```python
# launch/gazebo_sim.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'robot_world.world')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py'
            ]),
            launch_arguments={'world': world_file}.items(),
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'robot'],
            output='screen'
        ),
    ])
```

### Run Simulation

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_description gazebo_sim.launch.py
```

## Controlling the Simulated Robot

### Send Motor Commands

```python
# control_robot.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publisher for motor commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_callback)
        self.time_counter = 0

    def control_callback(self):
        msg = Twist()
        
        # Make robot move in a circle
        msg.linear.x = 0.3   # Forward velocity
        msg.angular.z = 0.5  # Angular velocity
        
        self.cmd_vel_pub.publish(msg)
        self.time_counter += 1
        
        if self.time_counter % 10 == 0:
            self.get_logger().info('Sending motor commands...')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Tuning

### Physics Engine Optimization

```xml
<physics name="default_physics" type="ode">
  <!-- Smaller timestep = more accurate but slower -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real time factor: >1 = faster than real time -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Iterations for solver accuracy -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <precon_iters>0</precon_iters>
      <sor>1.3</sor>
      <use_dynamic_moi_rescaling>false</use_dynamic_moi_rescaling>
    </solver>
  </ode>
</physics>
```

### Multi-threading

Enable multi-threaded physics:

```xml
<physics name="default_physics" type="ode">
  <ode>
    <solver>
      <parallel_method>OpenMP</parallel_method>
      <parallel_threads>4</parallel_threads>
    </solver>
  </ode>
</physics>
```

## Key Takeaways

✅ **Gazebo provides physics simulation for robotics**  
✅ **Worlds define environments with physics and objects**  
✅ **Sensors (camera, LiDAR, IMU) can be simulated**  
✅ **Control robots through ROS 2 topics**  
✅ **Simulation enables safe and fast prototyping**  
✅ **Tuning physics parameters balances accuracy vs speed**  

## Exercises

1. Create a complex world with obstacles and multiple objects
2. Add a simulated camera and process its output
3. Implement LiDAR-based obstacle detection
4. Build a simple autonomous navigation node

---

**Next**: [Chapter 6: NVIDIA Isaac - Advanced Perception](/docs/physical-ai/chapter-6-isaac)
