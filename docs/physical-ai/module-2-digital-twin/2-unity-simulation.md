---
sidebar_position: 2
---

# Chapter 5: High-Fidelity Simulation with Unity

## Why Unity for Robotics?

While Gazebo excels at physics accuracy, **Unity** provides:
- **Photorealistic graphics** - Detailed 3D environments
- **Human-robot interaction** - Test social navigation
- **Semantic understanding** - AI can "reason about" scenes
- **Fast iteration** - Visual feedback for development

## Setup: Unity Robotics

### Install Unity Hub

1. Download from `unity.com/download`
2. Create a Unity account
3. Install Unity 2022 LTS or newer

### Install ROS-TCP Connector

In Unity, use the Package Manager:
```
Window → TextMesh Pro → Import TMP Essentials
Window → Package Manager → Add package from git URL
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

### Configure ROS Connection

1. Create an empty GameObject called "RosConnector"
2. Add component: `ROS Connector`
3. Set:
   - ROS IP Address: `127.0.0.1` (localhost)
   - ROS Port: `5005`

## Creating a Humanoid in Unity

### Step 1: Model the Robot

Create a hierarchy:
```
Humanoid
├── Torso (Cube, scale 0.3 × 0.4 × 1.2)
├── Head (Sphere, radius 0.15)
│   └── Camera (Camera component)
├── LeftArm
│   ├── LeftShoulder (Sphere)
│   └── LeftForearm (Cylinder)
└── RightArm
    ├── RightShoulder (Sphere)
    └── RightForearm (Cylinder)
```

### Step 2: Add Physics

For each body part:
1. Add `Rigidbody` component
2. Set mass (torso: 15kg, arms: 2kg each)
3. Add `Capsule Collider` for realistic collisions

### Step 3: Add Joints

Connect body parts with `ConfigurableJoint`:

```csharp
using UnityEngine;

public class RobotJointController : MonoBehaviour
{
    public ConfigurableJoint joint;
    public float targetAngle = 0f;
    public float forceLimit = 100f;
    
    void FixedUpdate()
    {
        // Calculate torque needed to reach target angle
        float currentAngle = transform.localEulerAngles.x;
        float error = targetAngle - currentAngle;
        
        // Apply torque proportional to error
        Rigidbody rb = joint.GetComponent<Rigidbody>();
        Vector3 torque = Vector3.right * error * forceLimit;
        rb.AddTorque(torque);
    }
}
```

## Subscribing to ROS Topics

### Create a ROS Subscriber Script

```csharp
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class TwistSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("/cmd_vel", OnTwistReceived);
    }
    
    void OnTwistReceived(TwistMsg twist)
    {
        // Extract linear and angular velocities
        float forward = (float)twist.linear.x;
        float rotate = (float)twist.angular.z;
        
        // Apply to robot
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.velocity = new Vector3(forward, 0, 0);
        rb.angularVelocity = new Vector3(0, rotate, 0);
        
        Debug.Log($"Moving forward: {forward}, rotating: {rotate}");
    }
}
```

### Publishing Robot State

```csharp
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;

public class OdometryPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string odometryTopic = "/odom";
    private float publishRate = 30f;
    private float lastPublishTime;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(odometryTopic);
    }
    
    void FixedUpdate()
    {
        if (Time.time - lastPublishTime > 1f / publishRate)
        {
            PublishOdometry();
            lastPublishTime = Time.time;
        }
    }
    
    void PublishOdometry()
    {
        // Get current position and velocity
        Rigidbody rb = GetComponent<Rigidbody>();
        
        var odomMsg = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = (uint)Time.time },
                frame_id = "odom"
            },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg
                    {
                        x = transform.position.x,
                        y = transform.position.y,
                        z = transform.position.z
                    }
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg
                    {
                        x = rb.velocity.x,
                        y = rb.velocity.y,
                        z = rb.velocity.z
                    }
                }
            }
        };
        
        ros.Publish(odometryTopic, odomMsg);
    }
}
```

## Unity Camera Integration

### Subscribe to ROS Image and Render in Unity

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class ImageDisplayer : MonoBehaviour
{
    public RawImage displayImage;
    private Texture2D texture;
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>("/camera/image_raw", OnImageReceived);
        
        texture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        displayImage.texture = texture;
    }
    
    void OnImageReceived(ImageMsg imageMsg)
    {
        // Convert ROS image to Unity texture
        texture.LoadRawTextureData(imageMsg.data);
        texture.Apply();
    }
}
```

### Publish Camera Images from Unity

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class UnityCamera : MonoBehaviour
{
    public Camera captureCamera;
    private ROSConnection ros;
    private Texture2D texture;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>("/unity/camera/image");
        
        texture = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }
    
    void Update()
    {
        // Render camera to texture
        RenderTexture rt = new RenderTexture(640, 480, 24);
        captureCamera.targetTexture = rt;
        captureCamera.Render();
        
        // Read texture
        RenderTexture.active = rt;
        texture.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture.Apply();
        
        // Publish to ROS
        var imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = (uint)Time.time },
                frame_id = "camera"
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            is_bigendian = false,
            step = 640 * 3,
            data = texture.GetRawTextureData()
        };
        
        ros.Publish("/unity/camera/image", imageMsg);
    }
}
```

## Advanced: Physics-Based Humanoid Locomotion

```csharp
public class HumanoidLocomotion : MonoBehaviour
{
    [SerializeField] private Rigidbody torsoRb;
    [SerializeField] private Rigidbody leftFootRb;
    [SerializeField] private Rigidbody rightFootRb;
    [SerializeField] private float walkSpeed = 1f;
    [SerializeField] private float stepHeight = 0.3f;
    
    private float stepCycle = 0f;
    private const float StepDuration = 0.5f;
    
    void FixedUpdate()
    {
        stepCycle += Time.fixedDeltaTime / StepDuration;
        if (stepCycle > 1f) stepCycle -= 1f;
        
        // Alternating leg movement (bipedal gait)
        float leftLift = Mathf.Sin(stepCycle * Mathf.PI) * stepHeight;
        float rightLift = Mathf.Sin((stepCycle + 0.5f) * Mathf.PI) * stepHeight;
        
        // Move torso forward
        torsoRb.velocity = new Vector3(walkSpeed, torsoRb.velocity.y, 0);
        
        // Lift legs in walking pattern
        Vector3 leftFootPos = leftFootRb.position;
        leftFootPos.y = Mathf.Max(leftFootPos.y + leftLift, 0);
        leftFootRb.MovePosition(leftFootPos);
        
        Vector3 rightFootPos = rightFootRb.position;
        rightFootPos.y = Mathf.Max(rightFootPos.y + rightLift, 0);
        rightFootRb.MovePosition(rightFootPos);
    }
}
```

## Summary

Unity robotics provides:
- ✅ **Photorealistic simulation** - Better for visualizing human-robot interaction
- ✅ **Advanced graphics** - Test perception in realistic scenarios
- ✅ **ROS integration** - Full bidirectional communication
- ✅ **Game engine power** - Animators, shaders, particles
- ✅ **Fast development** - Visual editor for scene design

Next: Deploy advanced AI perception with NVIDIA Isaac!
