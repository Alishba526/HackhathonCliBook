---
sidebar_position: 8
---

# Capstone Project: Autonomous Humanoid with Voice Commands

## Project Overview

Build an autonomous humanoid robot that can:
1. **Listen** to voice commands
2. **Understand** natural language instructions
3. **Plan** a sequence of actions
4. **Execute** those actions in simulation
5. **Verify** success using computer vision

### Final Deliverable

A simulated humanoid robot that performs complex tasks based on voice commands, demonstrating the complete Physical AI stack.

## Architecture

```
┌─────────────────────────────────────┐
│   Voice Input (Whisper)             │ (speech → text)
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Language Understanding (GPT-4)    │ (text → action plan)
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Vision-based Verification (CLIP)  │ (verify feasibility)
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Motor Control (ROS 2)             │ (action plan → motor commands)
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Gazebo Simulation                 │ (execute in physics)
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│   Vision Feedback                   │ (verify task completion)
└─────────────────────────────────────┘
```

## Project Phases

### Phase 1: Setup and Simulation (Week 8-9)

#### 1.1: Create the Simulated Humanoid

```python
# ~/ros2_ws/src/autonomous_humanoid/humanoid_sim.py

from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_folder
import numpy as np

class HumanoidSimulator:
    def __init__(self):
        self.simulation_app = SimulationApp({"headless": False})
        self.world = World(stage_units_in_meters=1.0)
        
        # Load humanoid robot from NVIDIA's library
        assets_root_path = get_assets_root_folder()
        humanoid_usd = assets_root_path + "/Isaac/2023.1.1/Isaac3rdParty/NVIDIA/Humanoid_Pro/H1.usd"
        
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/Humanoid",
                usd_path=humanoid_usd,
                position=np.array([0, 0, 0.8])
            )
        )

    def step(self):
        self.world.step(render=True)
        return self.world.is_playing()

    def close(self):
        self.simulation_app.close()

def main():
    simulator = HumanoidSimulator()
    
    # Run simulation
    for i in range(1000):
        if not simulator.step():
            break
    
    simulator.close()

if __name__ == '__main__':
    main()
```

#### 1.2: Connect to ROS 2

```python
# Create bridge between Isaac Sim and ROS 2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidBridge(Node):
    def __init__(self, simulator):
        super().__init__('humanoid_bridge')
        self.simulator = simulator
        
        # Subscribe to control commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_callback,
            10
        )

    def velocity_callback(self, msg):
        # Convert Twist to humanoid motion
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Compute walking pattern
        joint_targets = self.compute_gait(linear_x, angular_z)
        self.apply_joint_targets(joint_targets)

    def joint_callback(self, msg):
        self.apply_joint_targets(msg.data)

    def compute_gait(self, forward_speed, turning_speed):
        """Compute bipedal walking gait"""
        # Simplified gait computation
        t = self.simulator.world.current_time
        
        hip_angles = forward_speed * np.sin(2 * np.pi * t)
        knee_angles = forward_speed * (np.cos(2 * np.pi * t) - 1)
        
        return np.array([hip_angles, knee_angles, 0, 0, hip_angles, knee_angles])

    def apply_joint_targets(self, targets):
        # Apply to simulated robot
        pass
```

### Phase 2: Voice to Action Pipeline (Week 10)

#### 2.1: Voice Capture

```python
# ~/ros2_ws/src/autonomous_humanoid/voice_interface.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import sounddevice as sd
import scipy.io.wavfile as wavfile
import numpy as np
import threading

class VoiceInterfaceNode(Node):
    def __init__(self):
        super().__init__('voice_interface')
        
        openai.api_key = "your-api-key-here"
        
        self.command_pub = self.create_publisher(String, '/voice/command', 10)
        self.listening = True
        
        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def listen_loop(self):
        while self.listening:
            try:
                # Record audio
                self.get_logger().info('Listening...')
                audio = sd.rec(
                    int(16000 * 5),  # 5 seconds
                    samplerate=16000,
                    channels=1,
                    dtype=np.int16
                )
                sd.wait()
                
                # Save audio
                wavfile.write('command.wav', 16000, audio)
                
                # Transcribe with Whisper
                with open('command.wav', 'rb') as f:
                    transcript = openai.Audio.transcribe("whisper-1", f)
                
                command = transcript['text']
                if command.strip():
                    self.get_logger().info(f'Heard: {command}')
                    
                    # Publish command
                    msg = String()
                    msg.data = command
                    self.command_pub.publish(msg)
                    
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.2: Language Understanding

```python
# ~/ros2_ws/src/autonomous_humanoid/language_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LanguagePlannerNode(Node):
    def __init__(self):
        super().__init__('language_planner')
        
        openai.api_key = "your-api-key-here"
        
        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice/command',
            self.command_callback,
            10
        )
        
        # Publish action plan
        self.plan_pub = self.create_publisher(String, '/action_plan', 10)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Processing: {command}')
        
        # Generate action plan
        plan = self.generate_plan(command)
        
        # Publish plan
        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.plan_pub.publish(plan_msg)

    def generate_plan(self, command):
        """Use GPT-4 to generate robot action plan"""
        
        system_prompt = """You are a robot planning system. Convert natural language commands into 
structured robot action sequences.

Available Actions:
- move_to(x, y, z): Move arm to position
- grasp(): Close gripper
- release(): Open gripper  
- navigate_to(x, y): Move robot base
- look_at(x, y, z): Turn camera toward point
- wait(seconds): Pause

You MUST respond with valid JSON array of actions only, no other text.
Example response: [{"action": "navigate_to", "params": {"x": 5, "y": 0}}, {"action": "grasp", "params": {}}]
"""
        
        user_prompt = f'Command: "{command}"'
        
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3
        )
        
        response_text = response['choices'][0]['message']['content'].strip()
        
        try:
            # Extract JSON if there's extra text
            import re
            json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
            else:
                plan = json.loads(response_text)
            
            return plan
        except:
            self.get_logger().error(f'Failed to parse plan: {response_text}')
            return []

def main(args=None):
    rclpy.init(args=args)
    node = LanguagePlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Phase 3: Execution and Feedback (Week 11)

#### 3.1: Action Executor

```python
# ~/ros2_ws/src/autonomous_humanoid/action_executor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')
        
        # Subscribe to action plans
        self.plan_sub = self.create_subscription(
            String,
            '/action_plan',
            self.plan_callback,
            10
        )
        
        # Control publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # Feedback publisher
        self.feedback_pub = self.create_publisher(String, '/execution_feedback', 10)

    def plan_callback(self, msg):
        plan = json.loads(msg.data)
        self.execute_plan(plan)

    def execute_plan(self, plan):
        for i, action in enumerate(plan):
            self.get_logger().info(f'Executing action {i+1}/{len(plan)}: {action["action"]}')
            
            action_type = action.get('action')
            params = action.get('params', {})
            
            try:
                if action_type == 'move_to':
                    self.move_to_position(params['x'], params['y'], params['z'])
                elif action_type == 'grasp':
                    self.grasp()
                elif action_type == 'release':
                    self.release()
                elif action_type == 'navigate_to':
                    self.navigate_to(params['x'], params['y'])
                elif action_type == 'look_at':
                    self.look_at(params['x'], params['y'], params['z'])
                elif action_type == 'wait':
                    time.sleep(params.get('seconds', 1))
                
                # Send feedback
                feedback = String()
                feedback.data = f"✓ Completed: {action_type}"
                self.feedback_pub.publish(feedback)
                
            except Exception as e:
                self.get_logger().error(f'Error executing {action_type}: {e}')

    def move_to_position(self, x, y, z):
        """Move arm to target position"""
        # Compute IK and send joint commands
        joint_angles = self.compute_inverse_kinematics(x, y, z)
        
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.arm_cmd_pub.publish(msg)
        
        time.sleep(1)  # Wait for arm to reach

    def grasp(self):
        """Close gripper"""
        msg = Float64MultiArray()
        msg.data = [0.5]  # Close
        self.arm_cmd_pub.publish(msg)
        time.sleep(0.5)

    def release(self):
        """Open gripper"""
        msg = Float64MultiArray()
        msg.data = [0.0]  # Open
        self.arm_cmd_pub.publish(msg)
        time.sleep(0.5)

    def navigate_to(self, x, y):
        """Navigate robot base to position"""
        msg = Twist()
        msg.linear.x = 0.3  # Forward
        msg.angular.z = 0.1  # Turning
        self.cmd_vel_pub.publish(msg)
        
        time.sleep(3)

    def look_at(self, x, y, z):
        """Turn camera toward point"""
        pass

    def compute_inverse_kinematics(self, x, y, z):
        """Compute joint angles for target position"""
        # Simplified IK
        import numpy as np
        r = np.sqrt(x**2 + y**2)
        
        angles = [
            np.arctan2(y, x),
            np.arctan2(z, r),
            np.arctan2(z, r) * 0.5,
            0.0
        ]
        
        return angles

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 3.2: Vision Feedback

```python
# ~/ros2_ws/src/autonomous_humanoid/vision_feedback.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from transformers import CLIPProcessor, CLIPModel
import json

class VisionFeedbackNode(Node):
    def __init__(self):
        super().__init__('vision_feedback')
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.feedback_pub = self.create_publisher(String, '/vision_feedback', 10)
        
        # Load CLIP for understanding objects
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Analyze image
        analysis = self.analyze_image(cv_image)
        
        # Publish feedback
        feedback = String()
        feedback.data = json.dumps(analysis)
        self.feedback_pub.publish(feedback)

    def analyze_image(self, image):
        """Analyze image for task completion"""
        
        # Prepare inputs
        image_input = self.processor(
            images=image,
            return_tensors="pt"
        )['pixel_values'].to(self.device)
        
        # Check for common objects
        objects_to_check = ["red cube", "box", "table", "gripper", "hand", "object"]
        
        with torch.no_grad():
            image_features = self.model.get_image_features(image_input)
            
            detections = {}
            for obj in objects_to_check:
                text_input = self.processor(
                    text=[f"a {obj}"],
                    return_tensors="pt",
                    padding=True
                ).to(self.device)
                
                text_features = self.model.get_text_features(**text_input)
                
                # Normalize
                image_features_norm = image_features / image_features.norm(dim=-1, keepdim=True)
                text_features_norm = text_features / text_features.norm(dim=-1, keepdim=True)
                
                # Compute similarity
                similarity = (image_features_norm @ text_features_norm.T)[0][0].item()
                
                if similarity > 0.3:
                    detections[obj] = float(similarity)
        
        return {
            "timestamp": float(rclpy.clock.Clock().now().nanoseconds),
            "objects_detected": detections
        }

def main(args=None):
    rclpy.init(args=args)
    node = VisionFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Phase 4: Integration and Testing (Week 12)

#### 4.1: Main Orchestrator

```python
# ~/ros2_ws/src/autonomous_humanoid/main_orchestrator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator')
        
        self.state = "idle"
        self.current_task = None
        
        # Publisher for starting execution
        self.execute_pub = self.create_publisher(String, '/action_plan', 10)
        
        # Subscriber for feedback
        self.feedback_sub = self.create_subscription(
            String,
            '/execution_feedback',
            self.feedback_callback,
            10
        )
        
        self.voice_sub = self.create_subscription(
            String,
            '/voice/command',
            self.voice_callback,
            10
        )

    def voice_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'\n🎤 Command: "{command}"\n')
        self.current_task = command
        self.state = "processing"

    def feedback_callback(self, msg):
        feedback = msg.data
        if "✓" in feedback:
            self.get_logger().info(f'  {feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4.2: Launch File

```python
# ~/ros2_ws/src/autonomous_humanoid/launch/full_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Humanoid simulator
        Node(
            package='autonomous_humanoid',
            executable='humanoid_sim',
            name='humanoid_sim'
        ),
        
        # Voice interface
        Node(
            package='autonomous_humanoid',
            executable='voice_interface',
            name='voice_interface'
        ),
        
        # Language planner
        Node(
            package='autonomous_humanoid',
            executable='language_planner',
            name='language_planner'
        ),
        
        # Action executor
        Node(
            package='autonomous_humanoid',
            executable='action_executor',
            name='action_executor'
        ),
        
        # Vision feedback
        Node(
            package='autonomous_humanoid',
            executable='vision_feedback',
            name='vision_feedback'
        ),
        
        # Orchestrator
        Node(
            package='autonomous_humanoid',
            executable='orchestrator',
            name='orchestrator'
        ),
    ])
```

## Running the Capstone

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_humanoid
source install/setup.bash

# Start the entire system
ros2 launch autonomous_humanoid full_system.launch.py
```

## Example Commands to Try

```
"Move your arm to the right"
"Pick up the red cube"
"Walk to the table"
"Look at the camera"
"Open your hand"
"Pick up the blue sphere and place it on the table"
```

## Evaluation Criteria

1. **Voice Recognition** (20 points)
   - Accurately captures voice commands
   - Robust to background noise

2. **Language Understanding** (20 points)
   - Correctly parses natural language
   - Handles complex multi-step tasks

3. **Execution** (30 points)
   - Actions execute smoothly in simulation
   - Proper sequencing of tasks
   - Error handling

4. **Vision Integration** (20 points)
   - Provides real-time feedback
   - Verifies task completion

5. **Code Quality** (10 points)
   - Well-documented
   - Modular design
   - Error handling

## Bonus Features

- **Advanced Gait**: Implement realistic bipedal walking
- **Obstacle Avoidance**: Navigate around objects
- **Multi-Robot Coordination**: Control multiple robots
- **Sim-to-Real Transfer**: Deploy to real robot
- **Complex Tasks**: Multi-stage tasks with dependencies

---

**Congratulations! You've completed the Physical AI & Humanoid Robotics textbook.** 🚀

Next step: Deploy your project to GitHub and submit to the hackathon!
