#!/usr/bin/env python3
"""
Basic Isaac Sim Example
Demonstrates fundamental Isaac Sim concepts including:
- World initialization
- Adding objects to the scene
- Basic physics simulation
- Sensor data collection
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import carb


def main():
    # Initialize the world
    print("Initializing Isaac Sim world...")
    my_world = World(stage_units_in_meters=1.0)

    # Set up a basic camera view
    set_camera_view(eye=[5, 5, 5], target=[0, 0, 0])

    # Add a ground plane
    print("Adding ground plane...")
    create_prim(
        prim_path="/World/GroundPlane",
        prim_type="Plane",
        scale=np.array([10.0, 10.0, 1.0]),
        position=np.array([0.0, 0.0, -0.1])
    )

    # Add a simple cube that will fall due to gravity
    print("Adding falling cube...")
    create_prim(
        prim_path="/World/Cube",
        prim_type="Cube",
        scale=np.array([0.2, 0.2, 0.2]),
        position=np.array([0.0, 0.0, 2.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0])
    )

    # Reset the world to apply changes
    my_world.reset()

    print("Starting simulation...")

    # Run the simulation for a number of steps
    for i in range(500):
        # Step the world forward
        my_world.step(render=True)

        # Print progress every 100 steps
        if i % 100 == 0:
            print(f"Simulation step: {i}")

        # Get current time
        current_time = my_world.current_time
        if i % 200 == 0:
            print(f"Current simulation time: {current_time:.3f}s")

    print("Simulation completed!")

    # Clean up
    my_world.clear()


if __name__ == "__main__":
    # Run the example
    main()

    # Shutdown Isaac Sim
    print("Shutting down Isaac Sim...")
    omni.kit.app.get_app().close()