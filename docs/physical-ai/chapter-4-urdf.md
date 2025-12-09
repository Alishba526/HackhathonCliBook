---
sidebar_position: 4
---

# Chapter 4: URDF - Robot Description Format

## What is URDF?

**URDF** (Unified Robot Description Format) is an XML language that describes:
- Robot structure (links and joints)
- Physical properties (mass, dimensions, friction)
- Visual appearance (for simulation)
- Collision properties (for physics simulation)
- Sensor placements

Think of URDF as the "blueprint" of your robot.

### Why URDF Matters

```
URDF File → ROS 2 Parser → Robotic System Understanding
              ↓
         Gazebo uses for simulation
         NVIDIA Isaac uses for digital twin
         Motion planners use for kinematics
         Visualization tools use for display
```

## Basic URDF Structure

### Minimal Example: Single Link

```xml
<?xml version="1.0" ?>
<robot name="simple_robot">
  
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" 
               iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
</robot>
```

### Two-Link System with Joint

```xml
<?xml version="1.0" ?>
<robot name="two_link_robot">
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.15"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.14159" effort="10" velocity="0.5"/>
  </joint>

</robot>
```

## Link Properties

### Visual Geometry

Shapes available in URDF:

```xml
<!-- Box -->
<box size="length width height"/>

<!-- Cylinder -->
<cylinder radius="0.1" length="0.5"/>

<!-- Sphere -->
<sphere radius="0.1"/>

<!-- Mesh (from 3D model) -->
<mesh filename="package://robot_description/meshes/arm.stl" scale="1 1 1"/>
```

### Collision Geometry

Collision shapes should be **simpler than visual shapes** for performance:

```xml
<collision>
  <!-- Use simplified shapes even if visual uses meshes -->
  <geometry>
    <box size="0.1 0.1 0.5"/>
  </geometry>
</collision>
```

### Inertial Properties

For physics simulation, specify mass and inertia tensor:

```xml
<inertial>
  <mass value="2.5"/>  <!-- kg -->
  <origin xyz="0 0 0.1"/>  <!-- Center of mass offset -->
  
  <!-- Inertia tensor (kg⋅m²) -->
  <inertia 
    ixx="0.02"  iyy="0.02"  izz="0.01"
    ixy="0"     ixz="0"     iyz="0"/>
</inertial>
```

**Inertia calculation** for simple shapes:

| Shape | Formula |
|-------|---------|
| Box | Ixx = m/12 × (y² + z²) |
| Cylinder | Ixx = m × r²/2 |
| Sphere | Ixx = 2/5 × m × r² |

## Joint Types

### Revolute (Rotating) Joint

```xml
<joint name="shoulder" type="revolute">
  <parent link="base"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.5"/>
  <axis xyz="0 0 1"/>  <!-- Rotation around Z-axis -->
  <limit lower="-1.57" upper="1.57" 
          effort="50" velocity="1.0"/>
</joint>
```

### Prismatic (Sliding) Joint

```xml
<joint name="slide" type="prismatic">
  <parent link="base"/>
  <child link="carriage"/>
  <origin xyz="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Linear motion along X-axis -->
  <limit lower="0" upper="1.0"
          effort="100" velocity="0.5"/>
</joint>
```

### Fixed Joint

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.1 0 0.05" rpy="0 0.3 0"/>
</joint>
```

### Other Joints

- **Continuous**: Rotates indefinitely (no limits)
- **Planar**: 2D movement in a plane
- **Floating**: 6 DOF (used for world root)

## Complete Humanoid Example

```xml
<?xml version="1.0" ?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="torso_color">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" 
               iyy="0.5" iyz="0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Left Upper Arm -->
  <link name="l_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Upper Arm -->
  <link name="r_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="l_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="l_upper_arm"/>
    <origin xyz="0.15 0.1 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Right Shoulder Joint -->
  <joint name="r_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="r_upper_arm"/>
    <origin xyz="-0.15 0.1 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="l_upper_leg">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" 
               iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Hip Joint -->
  <joint name="l_hip" type="revolute">
    <parent link="torso"/>
    <child link="l_upper_leg"/>
    <origin xyz="0.1 0 -0.25"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="1.0"/>
  </joint>

</robot>
```

## Using URDF in ROS 2

### Load URDF in Launch File

```python
# launch/display.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Load URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Robot state publisher (broadcasts transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 10.0
            }]
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])
```

### Verify URDF

```bash
# Check for errors
check_urdf robot.urdf

# Convert to PDF graph
urdf_to_graphviz robot.urdf > robot.pdf

# Test in RViz
ros2 launch my_package display.launch.py
```

## Key Takeaways

✅ **URDF describes robot structure, properties, and appearance**  
✅ **Links represent rigid bodies, joints connect them**  
✅ **Collision shapes should be simpler than visual shapes**  
✅ **Inertial properties are critical for physics simulation**  
✅ **Different joint types enable different motions**  
✅ **URDF integrates with Gazebo, Isaac, and planning tools**  

## Exercises

1. Create a URDF for a 2-DOF robot arm
2. Add visual meshes from STL files
3. Verify inertial properties using check_urdf
4. Visualize the robot in RViz with proper coordinate frames

---

**Next**: [Chapter 5: Physics Simulation with Gazebo](/docs/physical-ai/chapter-5-gazebo)
