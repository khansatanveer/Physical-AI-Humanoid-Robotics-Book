# Unity Scenes for Digital Twin Module

This directory contains Unity scene files for the Digital Twin (Gazebo & Unity) module. The scenes provide visualization for the physics simulation running in Gazebo.

## Scene Structure

### BasicVisualization.unity
- Main camera setup for viewing the digital twin
- Lighting system matching Gazebo environment
- Placeholder for humanoid robot model visualization
- Basic environment visualization

### Scene Configuration Requirements
- Unity 2023.2 LTS or later
- Physics settings matching Gazebo parameters (gravity: -9.81 m/s²)
- Collision detection enabled for accurate visualization
- Proper coordinate system alignment (ROS coordinate frame)

## Integration Notes
- Scene objects should correspond to Gazebo models
- Transform synchronization between Gazebo and Unity
- Real-time visualization of sensor data
- Performance optimization for real-time rendering (<100ms response time)