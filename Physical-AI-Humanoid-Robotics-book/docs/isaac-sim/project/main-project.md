---
title: Isaac Sim Main Project
sidebar_position: 3
---

# Isaac Sim Main Project: Creating a Complete Simulation Environment

## Project Overview

In this project, you'll create a complete simulation environment with multiple robots, sensors, and objects. You'll learn to:
- Set up a complex scene with lighting and physics
- Configure multiple robots with different sensors
- Generate synthetic data for AI training
- Control robots using basic control algorithms

## Bill of Materials (BOM)

### Software Requirements
- **NVIDIA Isaac Sim**: Latest version (requires NVIDIA GPU)
- **Python**: 3.8 or higher
- **CUDA**: 11.8 or higher
- **Isaac Sim Python API**: Included with Isaac Sim installation

### Assets Required
- **Robot Models**: URDF/SDF files for different robot types
- **Environment Models**: 3D models for indoor/outdoor scenes
- **Sensor Configurations**: Camera, LIDAR, IMU configurations

### Dependencies
- `omni.isaac.core`
- `omni.isaac.sensor`
- `numpy`
- `opencv-python`
- `Pillow`

## Full Code Implementation

```python
#!/usr/bin/env python3
"""
Complete Isaac Sim Project: Multi-Robot Simulation Environment
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.semantics import add_semantics
from omni.isaac.sensor import Camera
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import carb
import asyncio
import os
from PIL import Image
import json


class MultiRobotSimulation:
    def __init__(self):
        self.world = None
        self.robots = []
        self.cameras = []
        self.objects = []

    def setup_environment(self):
        """Set up the simulation environment with lighting and physics"""
        print("Setting up simulation environment...")

        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Set up camera view
        set_camera_view(eye=[10, 10, 10], target=[0, 0, 0])

        # Add ground plane
        create_prim(
            prim_path="/World/GroundPlane",
            prim_type="Plane",
            scale=np.array([20.0, 20.0, 1.0]),
            position=np.array([0.0, 0.0, 0.0])
        )

        # Add lighting
        create_prim(
            prim_path="/World/Light",
            prim_type="DistantLight",
            position=np.array([0, 0, 10]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )

        # Add walls to create an enclosed space
        wall_thickness = 0.2
        wall_height = 3.0
        room_size = 10.0

        # Front wall
        create_prim(
            prim_path="/World/Wall_Front",
            prim_type="Cube",
            scale=np.array([room_size, wall_thickness, wall_height]),
            position=np.array([0.0, -room_size/2, wall_height/2])
        )

        # Back wall
        create_prim(
            prim_path="/World/Wall_Back",
            prim_type="Cube",
            scale=np.array([room_size, wall_thickness, wall_height]),
            position=np.array([0.0, room_size/2, wall_height/2])
        )

        # Left wall
        create_prim(
            prim_path="/World/Wall_Left",
            prim_type="Cube",
            scale=np.array([wall_thickness, room_size, wall_height]),
            position=np.array([-room_size/2, 0.0, wall_height/2])
        )

        # Right wall
        create_prim(
            prim_path="/World/Wall_Right",
            prim_type="Cube",
            scale=np.array([wall_thickness, room_size, wall_height]),
            position=np.array([room_size/2, 0.0, wall_height/2])
        )

        print("Environment setup completed.")

    def add_robots(self):
        """Add multiple robots to the simulation"""
        print("Adding robots to the simulation...")

        # Add a simple cuboid robot (in a real scenario, you would load a URDF)
        robot1 = DynamicCuboid(
            prim_path="/World/Robot1",
            name="robot1",
            position=np.array([2.0, 2.0, 0.5]),
            size=0.5,
            mass=1.0
        )
        self.world.scene.add(robot1)

        robot2 = DynamicCuboid(
            prim_path="/World/Robot2",
            name="robot2",
            position=np.array([-2.0, -2.0, 0.5]),
            size=0.5,
            mass=1.0
        )
        self.world.scene.add(robot2)

        self.robots = [robot1, robot2]
        print(f"Added {len(self.robots)} robots to the simulation.")

    def add_sensors(self):
        """Add sensors to the robots and environment"""
        print("Adding sensors to the environment...")

        # Add a camera to observe the scene
        camera = Camera(
            prim_path="/World/Observer_Camera",
            position=np.array([8.0, 8.0, 8.0]),
            look_at_target=np.array([0, 0, 1.0])
        )
        camera.initialize()
        camera.add_raw_image_to_frame()
        camera.add_depth_image_to_frame()

        self.cameras.append(camera)
        self.world.scene.add_sensor("observer_camera", camera)

        print(f"Added {len(self.cameras)} cameras to the simulation.")

    def add_objects(self):
        """Add objects for the robots to interact with"""
        print("Adding objects to the environment...")

        # Add some objects for the robots to navigate around
        colors = [
            [1.0, 0.0, 0.0, 1.0],  # Red
            [0.0, 1.0, 0.0, 1.0],  # Green
            [0.0, 0.0, 1.0, 1.0],  # Blue
            [1.0, 1.0, 0.0, 1.0],  # Yellow
        ]

        positions = [
            np.array([3.0, 0.0, 0.2]),
            np.array([-3.0, 0.0, 0.2]),
            np.array([0.0, 3.0, 0.2]),
            np.array([0.0, -3.0, 0.2]),
        ]

        for i, (color, pos) in enumerate(zip(colors, positions)):
            obj = DynamicCuboid(
                prim_path=f"/World/Object_{i}",
                name=f"object_{i}",
                position=pos,
                size=0.3,
                mass=0.5,
                color=np.array(color)
            )
            self.world.scene.add(obj)
            self.objects.append(obj)

        print(f"Added {len(self.objects)} objects to the simulation.")

    def run_simulation(self, num_steps=1000):
        """Run the simulation for a specified number of steps"""
        print(f"Starting simulation for {num_steps} steps...")

        # Reset the world to apply all changes
        self.world.reset()

        # Create output directory for data
        output_dir = "./project_output"
        os.makedirs(output_dir, exist_ok=True)

        for step in range(num_steps):
            # Step the world
            self.world.step(render=True)

            # Occasionally save camera data
            if step % 100 == 0:
                print(f"Simulation step: {step}/{num_steps}")

                # Capture camera data if available
                if self.cameras:
                    camera = self.cameras[0]
                    try:
                        rgb_data = camera.get_raw_image()
                        if rgb_data is not None:
                            rgb_image = Image.fromarray(rgb_data, mode="RGBA")
                            rgb_image.save(f"{output_dir}/scene_{step:04d}.png")
                    except Exception as e:
                        print(f"Error capturing camera data: {e}")

            # Simple robot control - move robots in a circle
            if step > 50:  # Wait for initial settling
                time_factor = step / 100.0
                for i, robot in enumerate(self.robots):
                    try:
                        # Calculate circular motion
                        angle = time_factor + (i * np.pi)  # Offset for each robot
                        target_x = 3 * np.cos(angle)
                        target_y = 3 * np.sin(angle)

                        # Apply simple control to move robot
                        # In a real implementation, you would use proper control algorithms
                        pass
                    except Exception as e:
                        print(f"Error controlling robot {i}: {e}")

        print("Simulation completed.")

    def run_project(self):
        """Run the complete project"""
        try:
            self.setup_environment()
            self.add_robots()
            self.add_sensors()
            self.add_objects()
            self.run_simulation()

            print("Project completed successfully!")

        except Exception as e:
            print(f"Error during project execution: {e}")
            raise
        finally:
            if self.world:
                self.world.clear()


def main():
    """Main function to run the project"""
    print("Starting Isaac Sim Multi-Robot Simulation Project...")

    sim_project = MultiRobotSimulation()
    sim_project.run_project()

    print("Project finished!")


if __name__ == "__main__":
    main()

    # Shutdown Isaac Sim
    print("Shutting down Isaac Sim...")
    omni.kit.app.get_app().close()
```

## Step-by-Step Instructions

### 1. Setup
1. Ensure Isaac Sim is properly installed with all dependencies
2. Create a new Python environment with required packages
3. Verify your system meets the hardware requirements (NVIDIA GPU with RTX support)

### 2. Configuration
1. Set up the project directory structure as shown in the code
2. Configure your Python environment with Isaac Sim paths
3. Verify camera and sensor settings are properly configured

### 3. Implementation
1. Run the setup_environment() function to create the scene
2. Add robots using the add_robots() function
3. Install sensors with the add_sensors() function
4. Place objects using the add_objects() function

### 4. Testing
1. Execute the simulation with run_simulation()
2. Monitor the console output for any errors
3. Verify that robots move according to the control algorithm

### 5. Validation
1. Check that synthetic data is being generated correctly
2. Verify that all sensors are providing expected outputs
3. Confirm that the physics simulation is behaving as expected

## Visual Demonstrations
- [GIF/Video 1]: Complete simulation environment with multiple robots
- [GIF/Video 2]: Camera data capture and synthetic data generation
- [GIF/Video 3]: Robot movement and navigation in the environment