#!/usr/bin/env python3
"""
Synthetic Data Generation Example
Demonstrates how to collect and export synthetic data from Isaac Sim
including RGB images, depth maps, and segmentation masks.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.semantics import add_semantics
import numpy as np
import carb
import cv2
import os
from PIL import Image
import json


def setup_scene():
    """Set up a basic scene with objects for data generation"""
    # Add a ground plane
    create_prim(
        prim_path="/World/GroundPlane",
        prim_type="Plane",
        scale=np.array([5.0, 5.0, 1.0]),
        position=np.array([0.0, 0.0, 0.0])
    )

    # Add different colored cubes
    colors = [
        [1.0, 0.0, 0.0, 1.0],  # Red
        [0.0, 1.0, 0.0, 1.0],  # Green
        [0.0, 0.0, 1.0, 1.0],  # Blue
        [1.0, 1.0, 0.0, 1.0],  # Yellow
        [1.0, 0.0, 1.0, 1.0],  # Magenta
    ]

    for i, color in enumerate(colors):
        create_prim(
            prim_path=f"/World/Cube_{i}",
            prim_type="Cube",
            scale=np.array([0.3, 0.3, 0.3]),
            position=np.array([i - 2, 0.0, 0.3]),
            color=np.array(color)
        )
        # Add semantics to each cube for segmentation
        add_semantics(prim_path=f"/World/Cube_{i}", semantic_label=str(i), type_label="cube")


def capture_synthetic_data():
    """Capture synthetic data using a camera"""
    # Initialize the world
    print("Initializing Isaac Sim world for data generation...")
    my_world = World(stage_units_in_meters=1.0)

    # Set up the scene
    setup_scene()

    # Set up camera view
    set_camera_view(eye=[3, 3, 3], target=[0, 0, 0])

    # Create a camera prim
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([2.5, 2.5, 2.5]),
        look_at_target=np.array([0, 0, 0.5])
    )

    # Add the camera to the scene
    my_world.scene.add_sensor("camera", camera)

    # Reset the world
    my_world.reset()

    # Enable all sensor interfaces
    camera.initialize()
    camera.add_raw_image_to_frame()
    camera.add_depth_image_to_frame()
    camera.add_instance_segmentation_to_frame()

    print("Starting data capture...")

    # Create output directory
    output_dir = "./synthetic_data_output"
    os.makedirs(output_dir, exist_ok=True)

    # Capture data for multiple frames
    for frame in range(10):  # Capture 10 frames
        print(f"Capturing frame {frame + 1}/10")

        # Step the world
        my_world.step(render=True)

        # Get camera data
        rgb_data = camera.get_raw_image()
        depth_data = camera.get_depth()
        seg_data = camera.get_instance_segmentation()

        # Save RGB image
        if rgb_data is not None:
            rgb_image = Image.fromarray(rgb_data, mode="RGBA")
            rgb_image.save(f"{output_dir}/rgb_frame_{frame:03d}.png")

        # Save depth data
        if depth_data is not None:
            # Normalize depth for visualization
            depth_vis = ((depth_data - depth_data.min()) / (depth_data.max() - depth_data.min()) * 255).astype(np.uint8)
            depth_image = Image.fromarray(depth_vis, mode="L")
            depth_image.save(f"{output_dir}/depth_frame_{frame:03d}.png")

        # Save segmentation data
        if seg_data is not None:
            seg_image = Image.fromarray(seg_data, mode="I")
            seg_image.save(f"{output_dir}/seg_frame_{frame:03d}.png")

        # Save metadata
        metadata = {
            "frame": frame,
            "timestamp": float(my_world.current_time),
            "camera_position": camera.get_world_pose()[0].tolist(),
            "camera_orientation": camera.get_world_pose()[1].tolist(),
        }

        with open(f"{output_dir}/metadata_frame_{frame:03d}.json", "w") as f:
            json.dump(metadata, f, indent=2)

        # Move one of the cubes to create variation
        if frame < 5:
            # Move the first cube in a circular pattern
            angle = (frame / 5.0) * 2 * np.pi
            new_pos = np.array([np.cos(angle) * 1.5, np.sin(angle) * 1.5, 0.3])
            # Note: In a real scenario, you would update the cube's position here

    print(f"Data capture completed! Output saved to {output_dir}")

    # Clean up
    my_world.clear()


def main():
    """Main function to run the synthetic data generation example"""
    try:
        capture_synthetic_data()
        print("Synthetic data generation completed successfully!")
    except Exception as e:
        print(f"Error during synthetic data generation: {e}")
        raise


if __name__ == "__main__":
    main()

    # Shutdown Isaac Sim
    print("Shutting down Isaac Sim...")
    omni.kit.app.get_app().close()