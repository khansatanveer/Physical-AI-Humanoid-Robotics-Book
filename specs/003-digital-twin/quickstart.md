# Quickstart: Digital Twin (Gazebo & Unity)

## Prerequisites

Before starting with the Digital Twin module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed and configured
2. **Gazebo Harmonic** simulation environment
3. **Unity 2023.2 LTS** (Personal edition is sufficient)
4. **Python 3.11+** with pip package manager
5. Basic understanding of robotics concepts (covered in Module 1 & 2)

## Setup Environment

### 1. Clone and Prepare Repository
```bash
# Navigate to your robotics book directory
cd Physical-AI-Humanoid-Robotics-book

# Install Python dependencies
pip install -r requirements.txt

# Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 topic list
```

### 2. Verify Gazebo Installation
```bash
# Test Gazebo
gz sim --version
gz sim -v 4 empty.sdf  # Should open Gazebo GUI with empty world
```

### 3. Verify Unity Installation
```bash
# Unity should be installed and license configured
# For Unity Personal (free): https://store.unity.com/personal
```

## Running Your First Digital Twin Simulation

### 1. Start ROS 2 Environment
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source your workspace
source install/setup.bash
```

### 2. Launch Basic Gazebo Simulation
```bash
# Launch a simple humanoid robot in Gazebo
ros2 launch example_robot_gazebo example_robot.launch.py
```

### 3. Connect Unity Visualization (Coming in Chapter 2)
The integration between Gazebo physics and Unity visualization will be covered in Chapter 2. For now, verify that:

- Gazebo shows the robot with proper physics
- Robot responds to gravity and maintains balance
- You can see the robot state in RViz (alternative visualization)

## First Exercise: Basic Robot Physics

### Goal
Understand how gravity and collisions work in Gazebo simulation.

### Steps
1. Launch the empty world in Gazebo:
   ```bash
   gz sim -v 4 empty.sdf
   ```

2. Spawn a simple humanoid model:
   ```bash
   ros2 run example_robot_spawn spawn_robot.py
   ```

3. Observe how the robot:
   - Falls due to gravity
   - Collides with the ground
   - Maintains joint constraints

### Expected Outcome
The robot should fall from the air, land on the ground, and remain stable with proper joint limits preventing unnatural poses.

## Testing Your Setup

Run the following to verify everything is working:

```bash
# Test 1: ROS 2 communication
ros2 topic list | grep -E "joint|robot"

# Test 2: Gazebo simulation
gz topic -l | grep -E "clock|physics"

# Test 3: Basic movement
ros2 run example_robot_control move_robot.py --ros-args -p target_x:=1.0
```

## Troubleshooting

### Common Issues

1. **Gazebo fails to start**
   - Check if GPU drivers are properly installed
   - Try running with software rendering: `LIBGL_ALWAYS_SOFTWARE=1 gz sim`

2. **ROS 2 packages not found**
   - Ensure you've sourced the ROS 2 setup file
   - Check that your workspace is built: `colcon build`

3. **Robot model missing textures**
   - Verify model files are in the correct directory
   - Check that the model path is correctly specified in URDF

## Next Steps

After completing this quickstart:
1. Proceed to Chapter 1: Gazebo Physics Simulation
2. Learn about collision detection and physical constraints
3. Build your first interactive environment in Unity (Chapter 2)
4. Implement sensor simulation (Chapter 3)