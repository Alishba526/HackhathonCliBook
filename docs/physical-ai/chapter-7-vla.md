---
sidebar_position: 7
---

# Chapter 7: Vision-Language-Action Models

## The VLA Paradigm

**Vision-Language-Action (VLA)** models represent the frontier of robotics—where language understanding, visual perception, and motor control converge.

### The Idea

```
User Command (Language)
        ↓
LLM Understands Task
        ↓
Outputs Action Sequence
        ↓
Vision Model Verifies
        ↓
Robot Executes
        ↓
Feedback Loop
```

**Example**:
- **Input**: "Pick up the red cube and place it in the box"
- **LLM Processing**: Breaks into subtasks
- **Action Sequence**: [locate_object, approach, grasp, lift, place, release]
- **Vision Check**: Confirms object detected and reached
- **Execution**: Motor commands sent to robot

## Voice-to-Action Pipeline

### Step 1: Speech-to-Text with Whisper

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import numpy as np
import sounddevice as sd
import scipy.io.wavfile as wavfile

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        
        # Setup OpenAI API
        openai.api_key = "your-api-key"
        
        self.text_pub = self.create_publisher(String, '/voice/text', 10)
        
        # Audio parameters
        self.sample_rate = 16000
        self.duration = 5  # Record for 5 seconds

    def record_audio(self):
        # Record from microphone
        audio_data = sd.rec(
            int(self.sample_rate * self.duration),
            samplerate=self.sample_rate,
            channels=1,
            dtype=np.int16
        )
        sd.wait()
        return audio_data

    def transcribe(self, audio_file):
        # Use OpenAI Whisper
        with open(audio_file, 'rb') as f:
            transcript = openai.Audio.transcribe("whisper-1", f)
        return transcript['text']

    def main_loop(self):
        self.get_logger().info('Listening for voice commands...')
        
        while True:
            # Record audio
            audio_data = self.record_audio()
            
            # Save to file
            wavfile.write('speech.wav', self.sample_rate, audio_data)
            
            # Transcribe
            text = self.transcribe('speech.wav')
            self.get_logger().info(f'Recognized: {text}')
            
            # Publish
            msg = String()
            msg.data = text
            self.text_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Language Understanding with GPT

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class LanguageUnderstandingNode(Node):
    def __init__(self):
        super().__init__('language_understanding')
        
        openai.api_key = "your-api-key"
        
        # Subscribe to voice text
        self.text_sub = self.create_subscription(
            String,
            '/voice/text',
            self.text_callback,
            10
        )
        
        # Publish action plan
        self.action_pub = self.create_publisher(String, '/robot/action_plan', 10)

    def text_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Processing: {command}')
        
        # Call GPT to understand the command
        action_plan = self.plan_actions(command)
        
        # Publish plan
        plan_msg = String()
        plan_msg.data = json.dumps(action_plan)
        self.action_pub.publish(plan_msg)

    def plan_actions(self, command):
        """Use GPT to convert natural language to robot actions"""
        
        prompt = f"""You are a robot control system. Given a natural language command, 
break it into a sequence of robot actions.

Command: "{command}"

Available actions:
- move_to(x, y, z): Move end effector to position
- grasp(): Close gripper
- release(): Open gripper
- rotate_gripper(angle): Rotate end effector
- wait(seconds): Wait

Respond as JSON array of actions:
[{{"action": "move_to", "params": {{"x": 0.3, "y": 0.2, "z": 0.5}}}}, ...]
"""
        
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot control planner."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7
        )
        
        # Parse response
        action_text = response['choices'][0]['message']['content']
        try:
            actions = json.loads(action_text)
            return actions
        except:
            self.get_logger().error(f"Failed to parse actions: {action_text}")
            return []

def main(args=None):
    rclpy.init(args=args)
    node = LanguageUnderstandingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Vision-based Action Verification

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
from transformers import CLIPProcessor, CLIPModel
from cv_bridge import CvBridge
import json
import cv2

class ActionVerificationNode(Node):
    def __init__(self):
        super().__init__('action_verification')
        
        # Subscribe to action plan and camera
        self.action_sub = self.create_subscription(
            String,
            '/robot/action_plan',
            self.action_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )
        
        # Publish execution feedback
        self.feedback_pub = self.create_publisher(String, '/robot/feedback', 10)
        
        # Load CLIP model for vision-language understanding
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        self.model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
        
        self.bridge = CvBridge()
        self.current_image = None
        self.current_plan = None

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def action_callback(self, msg):
        self.current_plan = json.loads(msg.data)
        self.verify_and_execute_plan()

    def verify_and_execute_plan(self):
        """Verify each action against current scene"""
        
        for action in self.current_plan:
            # Get current image
            if self.current_image is None:
                continue
            
            # Verify object exists if action requires it
            if 'object' in action.get('params', {}):
                object_name = action['params']['object']
                
                if self.check_object_in_scene(object_name):
                    self.get_logger().info(f'✓ Object "{object_name}" detected')
                    feedback = f"Verified: {object_name} is present"
                else:
                    self.get_logger().warn(f'✗ Object "{object_name}" NOT detected')
                    feedback = f"Warning: {object_name} not found in current view"
                
                # Publish feedback
                msg = String()
                msg.data = feedback
                self.feedback_pub.publish(msg)

    def check_object_in_scene(self, object_name):
        """Use CLIP to check if object is in scene"""
        
        # Prepare text and image inputs
        text_inputs = self.processor(
            text=[f"a {object_name}"],
            return_tensors="pt",
            padding=True
        ).to(self.device)
        
        image_input = self.processor(
            images=self.current_image,
            return_tensors="pt"
        )['pixel_values'].to(self.device)
        
        # Compute embeddings
        with torch.no_grad():
            image_features = self.model.get_image_features(image_input)
            text_features = self.model.get_text_features(**text_inputs)
        
        # Normalize
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        
        # Similarity score
        similarity = (image_features @ text_features.T)[0][0].item()
        
        return similarity > 0.3  # Threshold

def main(args=None):
    rclpy.init(args=args)
    node = ActionVerificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Motor Control Execution

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
import json
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # Subscribe to action plan
        self.action_sub = self.create_subscription(
            String,
            '/robot/action_plan',
            self.execute_action,
            10
        )
        
        # Publishers for different actuators
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm/velocity', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper/velocity', 10)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def execute_action(self, msg):
        plan = json.loads(msg.data)
        
        for action in plan:
            action_type = action.get('action')
            params = action.get('params', {})
            
            if action_type == 'move_to':
                self.move_to(params['x'], params['y'], params['z'])
            elif action_type == 'grasp':
                self.grasp()
            elif action_type == 'release':
                self.release()
            elif action_type == 'rotate_gripper':
                self.rotate_gripper(params['angle'])
            elif action_type == 'wait':
                time.sleep(params.get('seconds', 1))

    def move_to(self, x, y, z):
        """Move end effector to target position"""
        # Use inverse kinematics to compute joint angles
        joint_angles = self.compute_ik(x, y, z)
        
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.arm_pub.publish(msg)
        
        self.get_logger().info(f'Moving to ({x}, {y}, {z})')

    def grasp(self):
        """Close gripper"""
        msg = Float64MultiArray()
        msg.data = [1.0]  # Close gripper
        self.gripper_pub.publish(msg)
        self.get_logger().info('Grasping')

    def release(self):
        """Open gripper"""
        msg = Float64MultiArray()
        msg.data = [0.0]  # Open gripper
        self.gripper_pub.publish(msg)
        self.get_logger().info('Releasing')

    def rotate_gripper(self, angle):
        """Rotate end effector"""
        msg = Float64MultiArray()
        msg.data = [angle]
        self.arm_pub.publish(msg)

    def compute_ik(self, x, y, z):
        """Compute inverse kinematics"""
        # Mock implementation
        return [0.0, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Full Integration: Autonomous Humanoid

```python
# Complete autonomous system

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AutonomousHumanoidNode(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid')
        
        # System state
        self.state = "idle"
        self.current_task = None

    def voice_command_received(self, command):
        """Main entry point for voice commands"""
        
        self.get_logger().info(f'🎤 Received: {command}')
        
        # Step 1: Parse with LLM
        actions = self.parse_command(command)
        self.get_logger().info(f'📋 Parsed {len(actions)} actions')
        
        # Step 2: Verify with vision
        verified_actions = self.verify_actions(actions)
        self.get_logger().info(f'✓ Verified {len(verified_actions)} actions')
        
        # Step 3: Execute actions
        for action in verified_actions:
            self.execute_action(action)
        
        self.get_logger().info('✓ Task completed')

    def parse_command(self, command):
        """Use LLM to parse natural language"""
        # Implementation from LanguageUnderstandingNode
        pass

    def verify_actions(self, actions):
        """Use vision to verify feasibility"""
        # Implementation from ActionVerificationNode
        return actions

    def execute_action(self, action):
        """Execute single action"""
        # Implementation from MotorControlNode
        pass
```

## Key Applications

### 1. Home Assistance
```
Voice: "Clean the living room"
→ Navigate to room
→ Search for trash
→ Pick up items
→ Place in trash bin
```

### 2. Manufacturing
```
Voice: "Assemble part A and part B"
→ Locate parts
→ Grasp part A
→ Align with part B
→ Apply force
→ Verify assembly
```

### 3. Search & Rescue
```
Voice: "Find and retrieve the medical kit"
→ Navigate through debris
→ Search for kit
→ Identify location
→ Grasp and carry
→ Return to base
```

## Key Takeaways

✅ **VLA models combine language, vision, and action**  
✅ **Whisper enables accurate speech-to-text**  
✅ **LLMs break down natural language into actions**  
✅ **Vision models verify actions are feasible**  
✅ **Motor controllers execute verified actions**  
✅ **Feedback loops enable error recovery**  

## Exercises

1. Implement a complete voice-to-action pipeline
2. Add error handling and retry logic
3. Create multi-step task sequences
4. Build a demo with real-time visualization

---

**Next**: [Chapter 8: Capstone Project - Autonomous Humanoid](/docs/physical-ai/capstone)
