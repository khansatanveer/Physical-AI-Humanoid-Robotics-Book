"""
Common utilities for Isaac Sim examples
"""
import os
import sys
import numpy as np
from typing import Dict, List, Tuple, Optional


def get_project_root() -> str:
    """Get the root directory of the project"""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))



def setup_isaac_environment():
    """Setup common environment variables and paths for Isaac Sim"""
    # Add Isaac Sim paths if available
    pass


def load_robot_config(robot_name: str) -> Dict:
    """Load configuration for a specific robot"""
    # This would load robot-specific configurations
    return {
        "name": robot_name,
        "config_path": f"configs/{robot_name}.yaml",
        "urdf_path": f"models/{robot_name}/urdf/{robot_name}.urdf"
    }


def create_simulation_scene(scene_name: str) -> Dict:
    """Create a basic simulation scene configuration"""
    return {
        "scene_name": scene_name,
        "gravity": [0, 0, -9.81],
        "time_step": 1.0/60.0,
        "objects": [],
        "lights": [],
        "cameras": []
    }


def validate_simulation_config(config: Dict) -> bool:
    """Validate simulation configuration"""
    required_keys = ["scene_name", "gravity", "time_step"]
    return all(key in config for key in required_keys)


def log_simulation_step(step: int, info: Dict = None):
    """Log simulation step information"""
    if info is None:
        info = {}
    print(f"Simulation Step: {step}, Info: {info}")


def calculate_distance(pos1: List[float], pos2: List[float]) -> float:
    """Calculate Euclidean distance between two 3D points"""
    return np.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))


def normalize_vector(vector: List[float]) -> List[float]:
    """Normalize a 3D vector"""
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return [v / norm for v in vector]


def quaternion_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    # Convert quaternion to Euler angles
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles to quaternion (w, x, y, z)"""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z