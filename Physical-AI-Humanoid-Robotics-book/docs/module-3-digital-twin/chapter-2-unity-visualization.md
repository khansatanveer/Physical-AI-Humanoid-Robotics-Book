# Chapter 2: Unity Environment Building and Visualization

## Learning Objectives
After completing this chapter, you will be able to:
- Set up a Unity project for digital twin visualization
- Create interactive 3D environments that complement Gazebo physics
- Build compelling visual representations of digital twins
- Implement real-time synchronization between Gazebo and Unity
- Validate that visualization meets performance targets (< 100ms response time)

## Introduction

In the previous chapter, we established the physics foundation of our digital twin using Gazebo. Now, we'll focus on the visualization aspect using Unity, one of the most powerful real-time 3D engines available. A complete digital twin combines accurate physics simulation (Gazebo) with compelling visual representation (Unity) to mirror real-world robot behavior.

Unity provides high-quality rendering, intuitive development tools, and excellent performance for real-time visualization. For digital twins, Unity handles the visual aspects - how the robot and environment appear, how users interact with the simulation, and how sensor data is visualized.

## Setting Up Unity for Digital Twin Applications

### Prerequisites
Before starting this chapter, ensure you have:
1. Unity 2023.2 LTS (Long Term Support) installed
2. Basic understanding of Unity concepts (scenes, GameObjects, components)
3. Completed Chapter 1 (Gazebo Physics Simulation)
4. ROS 2 and Gazebo installed for integration testing

### Unity Project Structure for Digital Twins

For digital twin applications, organize your Unity project as follows:
```
Assets/
├── Scenes/              # Unity scene files for different environments
├── Models/              # 3D models for robots and environment
├── Materials/           # Material definitions for visual properties
├── Scripts/             # Custom scripts for integration and behavior
├── Plugins/             # External libraries for ROS integration
├── Resources/           # Runtime-loadable assets
└── Prefabs/             # Reusable GameObject templates
```

## Exercise 1: Basic Unity Scene Setup

Let's start by creating a basic Unity scene that will serve as the foundation for our digital twin visualization.

### Step 1: Create New Unity Project

1. Open Unity Hub
2. Click "New Project"
3. Select the "3D (Built-in Render Pipeline)" template
4. Name your project "DigitalTwinVisualization"
5. Choose a location to save the project
6. Click "Create Project"

### Step 2: Configure Project Settings

1. Go to Edit → Project Settings → Time
   - Set Maximum Allowed Timestep to 0.333 (for 30 FPS minimum)
   - Set Fixed Timestep to 0.02 (50 FPS target for physics sync)

2. Go to Edit → Project Settings → Quality
   - Adjust settings based on target hardware requirements
   - For digital twin applications, prioritize performance over visual quality

### Step 3: Basic Scene Setup

Create a basic scene with:
- Main Camera positioned for viewing the digital twin
- Directional Light to match Gazebo's lighting
- Basic plane for ground reference
- UI elements for displaying simulation status

```csharp
// Example script to position camera for digital twin viewing
using UnityEngine;

public class DigitalTwinCamera : MonoBehaviour
{
    public Transform target; // The robot or object to focus on
    public Vector3 offset = new Vector3(-5, 3, -5);
    public float smoothSpeed = 0.125f;

    void LateUpdate()
    {
        if (target != null)
        {
            Vector3 desiredPosition = target.position + offset;
            Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            transform.position = smoothedPosition;

            transform.LookAt(target);
        }
    }
}
```

## Exercise 2: Creating Environment Assets

Digital twins require accurate visual representation of the environment. Let's create some basic environment elements.

### Step 1: Ground Plane and Basic Obstacles

1. Create a plane as the ground (GameObject → 3D Object → Plane)
2. Add basic obstacles like cubes or cylinders to represent the Gazebo world
3. Apply materials that match the Gazebo environment for consistency

### Step 2: Coordinate System Alignment

Unity and ROS/Gazebo use different coordinate systems:
- **ROS/Gazebo**: X forward, Y left, Z up
- **Unity**: X right, Y up, Z forward

This requires conversion when synchronizing positions between systems:

```csharp
// Helper functions to convert between coordinate systems
public static class CoordinateConverter
{
    // Convert from ROS coordinate system to Unity
    public static Vector3 RosToUnity(Vector3 rosVector)
    {
        return new Vector3(rosVector.z, rosVector.x, rosVector.y);
    }

    // Convert from Unity coordinate system to ROS
    public static Vector3 UnityToRos(Vector3 unityVector)
    {
        return new Vector3(unityVector.y, unityVector.z, unityVector.x);
    }

    // Convert rotation from ROS to Unity
    public static Quaternion RosToUnity(Quaternion rosQuaternion)
    {
        return new Quaternion(rosQuaternion.w, rosQuaternion.z, rosQuaternion.x, rosQuaternion.y);
    }
}
```

## Exercise 3: Robot Visualization Model

Now let's create a visualization model for our humanoid robot that matches the physics model from Chapter 1.

### Step 1: Import Robot Model

For digital twins, the visual model should match the physical model as closely as possible:
1. Create primitive shapes (cylinders, spheres, boxes) that match the URDF dimensions
2. Position and scale them according to the URDF specifications
3. Apply appropriate materials and colors

### Step 2: Robot Joint Visualization

Create a hierarchy that matches the URDF joint structure:
- Base link (body)
  - Head
  - Left arm (with sub-joints)
    - Upper arm
    - Lower arm
  - Right arm (with sub-joints)
    - Upper arm
    - Lower arm
  - Left leg (with sub-joints)
    - Upper leg
    - Lower leg
  - Right leg (with sub-joints)
    - Upper leg
    - Lower leg

```csharp
// Robot controller script to synchronize joint positions
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [System.Serializable]
    public class JointInfo
    {
        public string jointName;
        public Transform jointTransform;
        public float position; // Current joint position
        public float minLimit = -1.57f;
        public float maxLimit = 1.57f;
    }

    public JointInfo[] joints;

    // Update joint positions based on ROS messages
    public void UpdateJointPositions(float[] jointPositions)
    {
        for (int i = 0; i < Mathf.Min(joints.Length, jointPositions.Length); i++)
        {
            float clampedPosition = Mathf.Clamp(jointPositions[i], joints[i].minLimit, joints[i].maxLimit);
            joints[i].position = clampedPosition;

            // Apply rotation based on joint type (simplified for revolute joints)
            if (joints[i].jointTransform != null)
            {
                joints[i].jointTransform.localRotation = Quaternion.Euler(0, 0, clampedPosition * Mathf.Rad2Deg);
            }
        }
    }
}
```

## Exercise 4: Real-time Synchronization

The core of a digital twin is real-time synchronization between physics simulation and visualization.

### Step 1: ROS Integration Setup

To connect Unity with ROS, you'll typically use:
- ROS# (ROS bridge for Unity) - Unity package for ROS communication
- WebSocket connections to rosbridge_suite
- Custom integration layer

```csharp
// Example of a basic ROS connection script
using UnityEngine;
using RosSharp.RosBridgeClient;

public class DigitalTwinConnector : MonoBehaviour
{
    public string rosBridgeServerUrl = "ws://localhost:9090";
    private RosSocket rosSocket;

    void Start()
    {
        ConnectToRosBridge();
    }

    void ConnectToRosBridge()
    {
        RosBridgeClient.Protocols.WebSocketNetProtocol webSocket =
            new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeServerUrl);

        rosSocket = new RosSocket(webSocket);

        // Subscribe to robot state topic
        rosSocket.Subscribe<sensor_msgs.JointState>(
            "/basic_humanoid/joint_states",
            UpdateRobotJoints);
    }

    void UpdateRobotJoints(sensor_msgs.JointState jointState)
    {
        // Update Unity visualization based on ROS joint states
        // This method should update the RobotController with new positions
    }

    void OnDestroy()
    {
        rosSocket?.Close();
    }
}
```

### Step 2: Synchronization Pipeline

Create a pipeline for real-time synchronization:
1. ROS publishes joint states at high frequency (50Hz+)
2. Unity receives and processes these messages
3. Robot visualization updates in real-time
4. Performance is maintained above 30 FPS

## Exercise 5: Performance Optimization

For digital twin applications, maintaining performance is crucial. Unity visualization should respond in < 100ms to maintain the illusion of real-time operation.

### Optimization Techniques

1. **Level of Detail (LOD)**: Use simpler models when the camera is far away
2. **Occlusion Culling**: Don't render objects not visible to the camera
3. **Static Batching**: Combine static objects to reduce draw calls
4. **Dynamic Batching**: Unity automatically batches small dynamic objects

```csharp
// Performance monitoring script
using UnityEngine;

public class PerformanceMonitor : MonoBehaviour
{
    private float[] frameTimes = new float[60];
    private int frameIndex = 0;

    void Update()
    {
        frameTimes[frameIndex] = Time.unscaledDeltaTime;
        frameIndex = (frameIndex + 1) % frameTimes.Length;

        // Check if we're meeting the < 100ms response requirement
        if (frameTimes[frameIndex] > 0.1f) // 100ms threshold
        {
            Debug.LogWarning("Frame time exceeded 100ms: " + frameTimes[frameIndex]);
        }
    }

    public float GetAverageFrameTime()
    {
        float sum = 0;
        for (int i = 0; i < frameTimes.Length; i++)
        {
            sum += frameTimes[i];
        }
        return sum / frameTimes.Length;
    }

    public int GetFps()
    {
        return Mathf.RoundToInt(1.0f / GetAverageFrameTime());
    }
}
```

## Exercise 6: Interactive Elements

Digital twin visualizations often need interactive elements for user engagement.

### Adding Interaction

1. **Camera Control**: Allow users to move around the environment
2. **Robot Control**: Allow users to control the robot in simulation
3. **Information Display**: Show robot state, sensor readings, etc.
4. **Simulation Controls**: Play, pause, reset simulation

```csharp
// Camera controller for user interaction
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float lookSensitivity = 2f;

    private float rotationX = 0f;
    private float rotationY = 0f;

    void Update()
    {
        // WASD for movement
        float moveX = Input.GetAxis("Horizontal") * moveSpeed * Time.deltaTime;
        float moveZ = Input.GetAxis("Vertical") * moveSpeed * Time.deltaTime;

        // Handle rotation
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            rotationX += Input.GetAxis("Mouse X") * lookSensitivity;
            rotationY -= Input.GetAxis("Mouse Y") * lookSensitivity;
            rotationY = Mathf.Clamp(rotationY, -90f, 90f);

            transform.localRotation = Quaternion.AngleAxis(rotationX, Vector3.up);
            transform.localRotation *= Quaternion.AngleAxis(rotationY, Vector3.left);
        }

        // Move in the direction the camera is facing
        Vector3 forward = transform.TransformDirection(Vector3.forward);
        Vector3 right = transform.TransformDirection(Vector3.right);

        transform.position += (forward * moveZ + right * moveX);
    }
}
```

## Validation: Testing Your Unity Visualization

To validate that your Unity visualization is working correctly:

1. **Visual Consistency**: Verify that Unity models match Gazebo physics models
2. **Synchronization**: Check that robot movements in Unity match physics simulation
3. **Performance**: Ensure frame rate stays above 30 FPS and response time < 100ms
4. **Coordinate Alignment**: Confirm that positions and orientations match between systems

Run the following validation steps:

```bash
# Performance test - monitor Unity frame rate
# Check that visualization updates respond to physics changes in < 100ms
# Verify coordinate system conversions are correct
# Test interaction elements work as expected
```

## Troubleshooting Common Unity Issues

### Visualization Doesn't Match Physics
- Check coordinate system conversions between ROS and Unity
- Verify that visual models match URDF dimensions
- Ensure joint angles are applied correctly

### Poor Performance
- Simplify meshes and reduce polygon count
- Use Level of Detail (LOD) for complex models
- Check for unnecessary update loops in scripts
- Monitor draw calls and batching

### Synchronization Problems
- Verify ROS connection is stable
- Check that message frequencies match expectations
- Confirm time synchronization between systems
- Test network latency if systems are on different machines

## Summary

In this chapter, you learned how to create compelling visual representations for your digital twin:
- How to set up Unity projects for digital twin applications
- How to create visual models that match physics simulations
- How to implement real-time synchronization between Gazebo and Unity
- How to optimize performance to meet the < 100ms response requirement
- How to add interactive elements for user engagement

The visualization component completes the core digital twin concept by providing the visual representation that users interact with. In the next chapter, we'll explore sensor simulation to complete the perception pipeline of our digital twin.

## Next Steps

Continue to Chapter 3: Sensor Simulation and Integration to learn how to simulate various sensors (LiDAR, Depth Cameras, IMUs) that provide environmental awareness for your digital twin.