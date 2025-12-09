---
sidebar_position: 1
---

# Chapter 7: Vision-Language-Action Models

## What is VLA?

**Vision-Language-Action (VLA)** systems combine three capabilities:
- **Vision** - See and understand the world (cameras, object detection)
- **Language** - Understand natural language commands (LLMs)
- **Action** - Translate understanding into robot movements (ROS 2 control)

This creates robots that respond to natural language: *"Clean the table"* → robot understands, plans, and executes.

## The VLA Pipeline

```
Human: "Pick up the red cup"
     ↓
[Speech-to-Text: Whisper]
     ↓
Text: "Pick up the red cup"
     ↓
[Vision: CLIP + Object Detection]
     ↓
Scene Understanding: "Red cup at (0.5, 0.3, 0.8)"
     ↓
[Language Model: GPT-4]
     ↓
Action Plan: ["move_to(0.5, 0.3, 1.0)", "lower_gripper()", "close_gripper()"]
     ↓
[ROS 2 Controllers]
     ↓
Robot executes plan
```

## Part 1: Voice Input with Whisper

### Speech-to-Text with OpenAI Whisper

```bash
pip install openai-whisper sounddevice numpy scipy
```

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np

class VoiceCommandNode(Node):
    """Listen to voice commands and transcribe with Whisper"""
    
    def __init__(self):
        super().__init__('voice_command')
        
        # Load Whisper model
        self.model = whisper.load_model("base")  # Options: tiny, base, small, medium, large
        
        # Publisher for transcribed text
        self.command_pub = self.create_publisher(String, '/voice_command', 10)
        
        # Timer to record audio
        self.timer = self.create_timer(1.0, self.record_and_transcribe)
        
        self.get_logger().info("Voice command node ready (listening...)")
    
    def record_and_transcribe(self):
        """Record 5 seconds of audio and transcribe"""
        
        # Record 5 seconds at 16kHz
        sample_rate = 16000
        duration = 5
        
        self.get_logger().info("Recording...")
        audio = sd.rec(
            int(sample_rate * duration),
            samplerate=sample_rate,
            channels=1,
            dtype=np.int16
        )
        sd.wait()
        
        # Normalize audio
        audio_float = audio.astype(np.float32) / 32768.0
        
        # Transcribe
        result = self.model.transcribe(audio_float)
        text = result['text'].strip()
        
        if text:
            self.get_logger().info(f"You said: {text}")
            
            # Publish command
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 2: Vision Understanding with CLIP

### Object Detection and Understanding

```python
import torch
from PIL import Image
import clip

class VisionUnderstandingNode(Node):
    """Understand scenes using CLIP vision-language model"""
    
    def __init__(self):
        super().__init__('vision_understanding')
        
        # Load CLIP (Vision-Language model)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.understand_image,
            10
        )
        
        # Publish scene understanding
        self.scene_pub = self.create_publisher(String, '/scene_description', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Vision understanding node ready")
    
    def understand_image(self, msg):
        """Analyze image and describe objects"""
        
        # Convert ROS image to PIL
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # Objects to look for
        objects = ["red cup", "blue ball", "person", "table", "chair", "book"]
        
        with torch.no_grad():
            # Prepare image
            image = self.preprocess(pil_image).unsqueeze(0).to(self.device)
            
            # Prepare text labels
            text = clip.tokenize(objects).to(self.device)
            
            # Get similarity scores
            logits_per_image, _ = self.model(image, text)
            probabilities = logits_per_image.softmax(dim=-1)
        
        # Find highest probability
        top_idx = probabilities[0].argmax().item()
        top_object = objects[top_idx]
        top_prob = probabilities[0][top_idx].item()
        
        scene_desc = f"Detected: {top_object} (confidence: {top_prob:.2%})"
        
        self.get_logger().info(scene_desc)
        
        # Publish
        msg = String()
        msg.data = scene_desc
        self.scene_pub.publish(msg)
```

## Part 3: LLM Planning with GPT-4

### Translate Natural Language to Robot Actions

```python
import openai

class LLMPlannerNode(Node):
    """Use GPT-4 to plan robot actions from natural language"""
    
    def __init__(self):
        super().__init__('llm_planner')
        
        # Set OpenAI API key
        openai.api_key = "your-api-key"
        
        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.plan_actions,
            10
        )
        
        # Publish action sequence
        self.action_pub = self.create_publisher(String, '/action_sequence', 10)
        
        self.system_prompt = """
        You are a robot action planner. Convert natural language commands into 
        a sequence of ROS 2 action calls.
        
        Available actions:
        - move_forward(distance_m)
        - turn(angle_degrees)
        - pickup_object(object_name)
        - place_object(location)
        - open_gripper()
        - close_gripper()
        
        Example:
        Command: "Pick up the red cup from the table"
        Actions:
        1. move_forward(1.0)
        2. detect_object("red cup")
        3. move_to_object("red cup")
        4. open_gripper()
        5. pickup_object("red cup")
        
        Respond with only the action sequence, one per line.
        """
        
        self.get_logger().info("LLM Planner ready")
    
    def plan_actions(self, msg):
        """Plan robot actions using GPT-4"""
        
        command = msg.data
        self.get_logger().info(f"Planning actions for: {command}")
        
        try:
            # Call GPT-4
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.3,  # More deterministic
                max_tokens=200
            )
            
            action_sequence = response['choices'][0]['message']['content']
            
            self.get_logger().info(f"Action sequence:\n{action_sequence}")
            
            # Publish
            msg = String()
            msg.data = action_sequence
            self.action_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"LLM planning failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Action Execution with ROS 2

### Execute Planned Actions on Robot

```python
import re
from geometry_msgs.msg import Twist

class ActionExecutor(Node):
    """Execute action sequence on real or simulated robot"""
    
    def __init__(self):
        super().__init__('action_executor')
        
        # Subscribers
        self.action_sub = self.create_subscription(
            String,
            '/action_sequence',
            self.execute_actions,
            10
        )
        
        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        
        self.get_logger().info("Action executor ready")
    
    def execute_actions(self, msg):
        """Parse and execute action sequence"""
        
        action_sequence = msg.data
        actions = [line.strip() for line in action_sequence.split('\n') if line.strip()]
        
        for action in actions:
            self.get_logger().info(f"Executing: {action}")
            self.execute_single_action(action)
    
    def execute_single_action(self, action):
        """Execute one action"""
        
        # Parse action format: action_name(arg1, arg2, ...)
        match = re.match(r'(\w+)\((.*)\)', action)
        if not match:
            self.get_logger().warn(f"Invalid action format: {action}")
            return
        
        action_name = match.group(1)
        args_str = match.group(2)
        args = [arg.strip().strip('"\'') for arg in args_str.split(',')]
        
        if action_name == 'move_forward':
            distance = float(args[0])
            self.move_forward(distance)
        
        elif action_name == 'turn':
            angle = float(args[0])
            self.turn(angle)
        
        elif action_name == 'open_gripper':
            self.send_gripper_command('OPEN')
        
        elif action_name == 'close_gripper':
            self.send_gripper_command('CLOSE')
        
        elif action_name == 'pickup_object':
            obj = args[0]
            self.pickup_object(obj)
    
    def move_forward(self, distance):
        """Move robot forward"""
        msg = Twist()
        msg.linear.x = 0.5  # 0.5 m/s
        
        # Estimate time needed
        duration = distance / 0.5  # seconds
        
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(msg)
    
        # Stop
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def turn(self, angle_degrees):
        """Turn robot"""
        msg = Twist()
        angle_rad = angle_degrees * 3.14159 / 180
        msg.angular.z = 0.5  # 0.5 rad/s
        
        duration = angle_rad / 0.5
        
        for _ in range(int(duration * 10)):
            self.cmd_vel_pub.publish(msg)
        
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def send_gripper_command(self, command):
        """Open or close gripper"""
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
    
    def pickup_object(self, object_name):
        """Sequence to pick up an object"""
        self.open_gripper()
        self.move_forward(0.3)
        self.send_gripper_command('CLOSE')

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete VLA System Integration

### Launch Everything Together

```python
# launch/vla_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Voice input
        Node(
            package='robot_vla',
            executable='voice_command_node',
            output='screen'
        ),
        
        # Vision understanding
        Node(
            package='robot_vla',
            executable='vision_understanding_node',
            output='screen'
        ),
        
        # LLM planning
        Node(
            package='robot_vla',
            executable='llm_planner_node',
            output='screen'
        ),
        
        # Action execution
        Node(
            package='robot_vla',
            executable='action_executor_node',
            output='screen'
        ),
    ])
```

Run it:
```bash
ros2 launch robot_vla vla_system.launch.py
```

## Summary

Vision-Language-Action systems enable:
- ✅ **Natural language control** - Talk to your robot like a human
- ✅ **Scene understanding** - Vision-language models "see" like humans
- ✅ **Intelligent planning** - LLMs reason about complex tasks
- ✅ **Real robot execution** - Commands translate to actual movements
- ✅ **Adaptive behavior** - Can learn new tasks from examples

Next: Build the capstone—an autonomous humanoid robot!
