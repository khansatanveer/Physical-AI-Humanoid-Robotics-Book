# Unity Scene Structure for Digital Twin Visualization

## Overview
This document describes the Unity scene structure for the digital twin visualization system. The scene is designed to complement the Gazebo physics simulation by providing real-time visual representation of the robot and environment.

## Scene Hierarchy

```
DigitalTwinScene
├── Main Camera
│   ├── DigitalTwinCamera (script)
│   └── CameraController (script)
├── Directional Light
│   └── DigitalTwinLighting (script)
├── Ground Plane
│   └── GroundMaterial
├── Environment
│   ├── Obstacle1 (Cube)
│   ├── Obstacle2 (Cylinder)
│   ├── Ramp (Capsule)
│   └── Platform (Cube)
├── Robot Visualization
│   ├── Torso
│   ├── Head
│   ├── LeftUpperArm
│   ├── LeftLowerArm
│   ├── RightUpperArm
│   ├── RightLowerArm
│   ├── LeftUpperLeg
│   ├── LeftLowerLeg
│   ├── RightUpperLeg
│   └── RightLowerLeg
├── Performance Monitor
│   └── PerformanceMonitor (script)
├── Digital Twin Connector
│   └── DigitalTwinConnector (script)
├── Robot Controller
│   └── RobotController (script)
└── UI Canvas
    ├── JointSliders
    ├── PoseButtons
    ├── StatusText
    ├── FPSText
    └── RobotStateText
```

## Key Components Description

### 1. Main Camera
- **Purpose**: Primary viewpoint for the digital twin visualization
- **Scripts**:
  - DigitalTwinCamera: Smooth following of robot target
  - CameraController: User interaction and navigation
- **Properties**: Configured for 60° FOV with smooth movement

### 2. Directional Light
- **Purpose**: Primary illumination matching Gazebo lighting conditions
- **Scripts**: DigitalTwinLighting for realistic lighting setup
- **Configuration**: Includes main, fill, and rim lighting for depth

### 3. Ground Plane
- **Purpose**: Reference surface for the digital twin environment
- **Material**: GroundMaterial with appropriate color and properties
- **Scale**: 10x10 units to match Gazebo world dimensions

### 4. Environment Objects
- **Obstacle1 (Box)**: Static obstacle matching Gazebo world
- **Obstacle2 (Cylinder)**: Cylindrical obstacle in the environment
- **Ramp**: Inclined surface for robot navigation testing
- **Platform**: Elevated surface for robot interaction

### 5. Robot Visualization
- **Structure**: Hierarchical GameObjects matching URDF joint structure
- **Scripts**: RobotVisualization for joint position updates
- **Materials**: RobotMaterial for consistent appearance

### 6. Performance Monitor
- **Purpose**: Track rendering performance metrics
- **Scripts**: PerformanceMonitor to measure frame times
- **Target**: `<100ms` response time for real-time visualization

### 7. Digital Twin Connector
- **Purpose**: Interface with ROS/Gazebo simulation
- **Scripts**: DigitalTwinConnector for message handling
- **Function**: Receives joint states and updates visualization

### 8. Robot Controller
- **Purpose**: Handle robot pose and interaction
- **Scripts**: RobotController for pose management
- **Features**: Predefined poses and joint control

### 9. UI Canvas
- **Purpose**: Interactive controls and information display
- **Elements**: Joint sliders, pose buttons, status displays
- **Function**: User interaction with the digital twin

## Coordinate System Conversion

The Unity scene implements coordinate system conversion between ROS/Gazebo and Unity:

- **ROS/Gazebo**: X forward, Y left, Z up
- **Unity**: X right, Y up, Z forward

Conversion functions:
```csharp
// ROS to Unity conversion
Vector3 RosToUnity(Vector3 rosVector) {
    return new Vector3(rosVector.z, rosVector.x, rosVector.y);
}

// Unity to ROS conversion
Vector3 UnityToRos(Vector3 unityVector) {
    return new Vector3(unityVector.y, unityVector.z, unityVector.x);
}
```

## Performance Considerations

The scene is optimized for real-time performance:
- Static objects are marked for batching
- Materials use efficient shaders
- LOD systems implemented for complex models
- Target: >30 FPS with `<100ms` response time

## Integration Points

The Unity scene connects to external systems through:
- ROS bridge for joint state updates
- Performance monitoring systems
- User interaction interfaces
- Synchronization with physics simulation