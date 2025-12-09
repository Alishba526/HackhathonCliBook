---
sidebar_position: 3
---

# Chapter 3: URDF - Describing Your Humanoid Robot

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML standard that describes:
- Robot structure (links and joints)
- Physical properties (mass, size, materials)
- Sensor locations (cameras, LiDAR, IMU)
- Actuator constraints (joint limits, friction)

Think of URDF as a blueprint that tells Gazebo and ROS 2 how your robot is built.

## URDF Structure

### Basic Components

```xml
<?xml version="1.0"?>
<robot name="humanoid_v1">
  
  <!-- Links: Physical bodies -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" 
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    
    <!-- Visual appearance -->
    <visual>
      <geometry>
        <box size="0.3 0.3 0.8"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <!-- Collision geometry -->
    <collision>
      <geometry>
        <box size="0.3 0.3 0.8"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joints: Connections between links -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.2 0.0" rpy="0 0 0"/>
    
    <!-- Range of motion -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
    
    <!-- Friction properties -->
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

</robot>
```

## A Complete Humanoid Example

Here's a simplified humanoid with torso, arms, and head:

```xml
<?xml version="1.0"?>
<robot name="humanoid_simple">
  
  <!-- Torso (main body) -->
  <link name="torso">
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" 
               iyy="0.5" iyz="0.0" izz="0.3"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.3 0.4 1.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.4 1.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" 
               iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Neck joint connecting head to torso -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotate around Z axis -->
    <limit lower="-1.57" upper="1.57" effort="5" velocity="2.0"/>
    <dynamics damping="0.05"/>
  </joint>
  
  <!-- Left Arm -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
  </link>
  
  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" 
               iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.2 0.25 0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>  <!-- Rotate around X axis -->
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.5"/>
    <dynamics damping="0.1"/>
  </joint>
  
  <!-- Upper arm joint -->
  <joint name="left_shoulder_to_upper_arm" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotate around Y axis -->
    <limit lower="-2.0" upper="2.0" effort="15" velocity="1.5"/>
    <dynamics damping="0.05"/>
  </joint>
  
  <!-- Right Arm (mirror of left) -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
  </link>
  
  <link name="right_upper_arm">
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" 
               iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.2 0.25 0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.5"/>
    <dynamics damping="0.1"/>
  </joint>
  
  <joint name="right_shoulder_to_upper_arm" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="15" velocity="1.5"/>
    <dynamics damping="0.05"/>
  </joint>
  
  <!-- Materials definition -->
  <material name="blue">
    <color rgba="0.0 0.5 1.0 1.0"/>
  </material>
  
  <material name="skin">
    <color rgba="0.9 0.8 0.7 1.0"/>
  </material>
  
</robot>
```

## Understanding URDF Concepts

### Links
```xml
<link name="left_hand">
  <!-- Inertial: mass and moment of inertia -->
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
  
  <!-- Visual: how it looks in simulation -->
  <visual>
    <geometry>
      <mesh filename="package://robot_description/meshes/hand.stl"/>
    </geometry>
  </visual>
  
  <!-- Collision: how it interacts physically -->
  <collision>
    <geometry>
      <box size="0.1 0.05 0.2"/>
    </geometry>
  </collision>
</link>
```

### Joint Types

```xml
<!-- Revolute (hinge joint) - rotates around one axis -->
<joint name="elbow" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="10" velocity="1.0"/>
</joint>

<!-- Prismatic (sliding joint) - moves along one axis -->
<joint name="gripper" type="prismatic">
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.1" effort="50" velocity="0.5"/>
</joint>

<!-- Fixed (no movement) -->
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
</joint>
```

## Using URDF in ROS 2

### Load URDF in a Launch File

```python
# launch/display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = get_package_share_directory('robot_description')
    urdf_file += '/urdf/humanoid.urdf'
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', get_package_share_directory('robot_description') + '/rviz/config.rviz']
        )
    ])
```

## Summary

URDF enables:
- ✅ **Simulation** - Test robots in Gazebo before building
- ✅ **Visualization** - See robot structure in RViz
- ✅ **Physics** - Realistic mass, inertia, collisions
- ✅ **Joint Control** - Define what can move
- ✅ **Sensor Mounting** - Place cameras, LiDAR, IMU on robot

Next: Simulate your humanoid in Gazebo!
