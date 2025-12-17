# Chapter 3: Sensor Simulation and Integration

## Learning Objectives
After completing this chapter, you will be able to:
- Implement LiDAR sensor simulation in both Gazebo and Unity environments
- Configure depth camera sensors for 3D perception in digital twins
- Simulate IMU sensors for orientation and motion tracking
- Process and visualize sensor data in real-time
- Validate that sensor simulation produces realistic data outputs

## Introduction

In the previous chapters, we established the physics foundation (Chapter 1) and visualization layer (Chapter 2) of our digital twin. Now, we'll complete the digital twin concept by adding sensor simulation - the perception system that allows the robot to understand its environment.

Sensors are critical components of any robotic system, providing the data needed for navigation, mapping, and interaction. In digital twin applications, accurate sensor simulation is essential for developing and testing perception algorithms without requiring physical hardware. This chapter covers the three most common sensor types in robotics:

1. **LiDAR sensors** - For 360-degree distance measurements and mapping
2. **Depth cameras** - For 3D scene reconstruction and object detection
3. **IMU sensors** - For orientation, acceleration, and motion tracking

## LiDAR Sensor Simulation

### Understanding LiDAR in Robotics

LiDAR (Light Detection and Ranging) sensors emit laser pulses and measure the time it takes for the light to return after reflecting off objects. This provides accurate distance measurements that can be used for:

- Environment mapping
- Obstacle detection
- Localization
- Path planning

### LiDAR in Gazebo

Gazebo provides realistic LiDAR simulation through the `libgazebo_ros_ray.so` plugin. Here's how to configure a LiDAR sensor in your URDF:

```xml
<!-- LiDAR sensor definition -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
      <ros>
        <namespace>/basic_humanoid</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Configuration Parameters

- `update_rate`: How frequently the sensor publishes data (Hz)
- `samples`: Number of rays in the horizontal scan
- `min_angle`/`max_angle`: Angular range of the sensor
- `range_min`/`range_max`: Distance range of the sensor

## Exercise 1: Implement LiDAR Sensor in URDF

### Step 1: Add LiDAR Link to Robot Model

Add a LiDAR sensor to your humanoid robot model by creating a new link and joint:

```xml
<!-- LiDAR sensor link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.03"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Joint connecting LiDAR to head -->
<joint name="lidar_joint" type="fixed">
  <parent link="head"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

### Step 2: Test LiDAR in Gazebo

Launch your robot in Gazebo and verify that the LiDAR sensor is publishing data:

```bash
# Launch the robot with LiDAR
ros2 launch ros2_integration_examples basic_physics_with_sensors.launch.py

# Monitor the LiDAR data
ros2 topic echo /basic_humanoid/scan
```

## Depth Camera Sensor Simulation

### Understanding Depth Cameras

Depth cameras provide both color images and depth information for each pixel. This enables:

- 3D scene reconstruction
- Object recognition and segmentation
- Augmented reality applications
- Environment mapping

### Depth Camera in Gazebo

Gazebo supports depth camera simulation through the `libgazebo_ros_openni_kinect.so` plugin:

```xml
<!-- Depth camera sensor -->
<gazebo reference="camera_link">
  <sensor name="camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/basic_humanoid</namespace>
        <remapping>image_raw:=camera/image_raw</remapping>
        <remapping>depth/image_raw:=camera/depth/image_raw</remapping>
        <remapping>points:=camera/depth/points</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
      <baseline>0.2</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera Configuration Parameters

- `horizontal_fov`: Field of view of the camera
- `image_width`/`image_height`: Resolution of the captured images
- `clip_near`/`clip_far`: Range of depth measurements
- `update_rate`: Frame rate of the camera

## Exercise 2: Implement Depth Camera Sensor

### Step 1: Add Depth Camera to Robot Model

Add a depth camera to your robot's head:

```xml
<!-- Camera link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<!-- Joint connecting camera to head -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

### Step 2: Test Depth Camera in Gazebo

Verify that the depth camera is publishing both color and depth images:

```bash
# Monitor color images
ros2 topic echo /basic_humanoid/camera/image_raw --field data | head -c 100

# Monitor depth images
ros2 topic echo /basic_humanoid/camera/depth/image_raw --field data | head -c 100
```

## IMU Sensor Simulation

### Understanding IMU Sensors

Inertial Measurement Units (IMUs) measure:
- Linear acceleration (3 axes)
- Angular velocity (3 axes)
- Sometimes magnetic field (3 axes)

IMUs are essential for:
- Robot localization
- Motion tracking
- Orientation estimation
- Balance control

### IMU in Gazebo

Gazebo provides IMU simulation through the `libgazebo_ros_imu_sensor.so` plugin:

```xml
<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/basic_humanoid</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Configuration Parameters

- `update_rate`: How frequently the IMU publishes data
- `noise`: Simulated sensor noise for realistic data
- `frame_name`: Coordinate frame for the IMU measurements

## Exercise 3: Implement IMU Sensor

### Step 1: Add IMU to Robot Model

Add an IMU sensor to your robot's torso:

```xml
<!-- IMU link -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<!-- Joint connecting IMU to torso -->
<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### Step 2: Test IMU in Gazebo

Verify that the IMU is publishing orientation and acceleration data:

```bash
# Monitor IMU data
ros2 topic echo /basic_humanoid/imu
```

## Sensor Data Processing and Visualization

### Processing Sensor Data with ROS 2

Once sensors are implemented, you'll need to process the data for use in your applications:

```python
#!/usr/bin/env python3
# sensor_data_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np

class SensorDataProcessor(Node):
    def __init__(self):
        super().__init__('sensor_data_processor')

        # Create subscribers for all sensor types
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/basic_humanoid/scan',
            self.lidar_callback,
            10)

        self.camera_subscription = self.create_subscription(
            Image,
            '/basic_humanoid/camera/image_raw',
            self.camera_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            '/basic_humanoid/imu',
            self.imu_callback,
            10)

        self.bridge = CvBridge()

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)
        # Filter out invalid ranges
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Log some statistics
        if len(valid_ranges) > 0:
            avg_distance = np.mean(valid_ranges)
            self.get_logger().info(f'Average distance to obstacles: {avg_distance:.2f}m')

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image (example: log image dimensions)
        height, width, channels = cv_image.shape
        self.get_logger().info(f'Camera image: {width}x{height}x{channels}')

    def imu_callback(self, msg):
        # Extract orientation and angular velocity
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity

        # Log orientation (example)
        self.get_logger().info(
            f'IMU orientation: x={orientation.x:.3f}, y={orientation.y:.3f}, z={orientation.z:.3f}, w={orientation.w:.3f}')

def main(args=None):
    rclpy.init(args=args)
    processor = SensorDataProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visualizing Sensor Data in Unity

To visualize sensor data in Unity, you can create specialized components:

```csharp
// SensorDataVisualizer.cs
using UnityEngine;
using System.Collections.Generic;

public class SensorDataVisualizer : MonoBehaviour
{
    [Header("Sensor Visualization")]
    public GameObject lidarPointPrefab;
    public GameObject depthPointCloudPrefab;
    public LineRenderer imuOrientationIndicator;

    [Header("Visualization Settings")]
    public float maxLidarRange = 30.0f;
    public int lidarResolution = 360;
    public float pointSize = 0.05f;

    private List<GameObject> lidarPoints = new List<GameObject>();
    private List<Vector3> lidarReadings = new List<Vector3>();

    void Start()
    {
        InitializeLidarVisualization();
    }

    void InitializeLidarVisualization()
    {
        // Create lidar points for visualization
        for (int i = 0; i < lidarResolution; i++)
        {
            GameObject point = Instantiate(lidarPointPrefab, transform);
            point.SetActive(false);
            lidarPoints.Add(point);
        }
    }

    public void UpdateLidarData(float[] ranges, float angleMin, float angleMax)
    {
        float angleIncrement = (angleMax - angleMin) / ranges.Length;

        for (int i = 0; i < Mathf.Min(ranges.Length, lidarPoints.Count); i++)
        {
            float range = ranges[i];
            float angle = angleMin + i * angleIncrement;

            if (range >= 0.1f && range <= maxLidarRange)
            {
                // Calculate position in Unity coordinates
                float x = range * Mathf.Cos(angle);
                float y = 0; // Assuming 2D LiDAR
                float z = range * Mathf.Sin(angle);

                Vector3 worldPos = transform.TransformPoint(new Vector3(x, y, z));

                lidarPoints[i].transform.position = worldPos;
                lidarPoints[i].SetActive(true);
            }
            else
            {
                lidarPoints[i].SetActive(false);
            }
        }
    }

    public void UpdateIMUData(Quaternion orientation)
    {
        if (imuOrientationIndicator != null)
        {
            // Update the orientation indicator based on IMU data
            imuOrientationIndicator.transform.rotation = orientation;
        }
    }

    public void ClearVisualization()
    {
        foreach (GameObject point in lidarPoints)
        {
            if (point != null)
                point.SetActive(false);
        }
    }
}
```

## Exercise 4: Sensor Integration Challenge

### Objective
Integrate all three sensor types (LiDAR, depth camera, IMU) into your humanoid robot model and create a simple perception system.

### Tasks
1. Add all three sensors to your URDF model
2. Configure appropriate update rates for each sensor
3. Create a ROS 2 node that subscribes to all sensor data
4. Implement basic processing for each sensor type
5. Test the integrated sensor system in simulation

### Validation
- All sensors publish data at their configured rates
- Sensor data is processed without significant delay
- Perception system maintains real-time performance
- Sensor data appears realistic and accurate

## Performance Considerations for Sensor Simulation

### Computational Requirements
Sensor simulation can be computationally intensive:
- LiDAR: High ray count increases accuracy but decreases performance
- Depth cameras: Higher resolution requires more processing power
- IMU: Generally lightweight but high update rates can impact performance

### Optimization Strategies
1. **Adjust update rates** based on application requirements
2. **Reduce sensor resolution** when maximum accuracy isn't needed
3. **Use Level of Detail (LOD)** for sensor visualization
4. **Implement sensor data filtering** to reduce processing load

## Troubleshooting Common Sensor Issues

### LiDAR Problems
- **No data published**: Check Gazebo plugin configuration and ROS remappings
- **Incorrect ranges**: Verify coordinate frame and sensor mounting position
- **Performance issues**: Reduce ray count or increase minimum range

### Depth Camera Problems
- **Black images**: Check camera parameters and lighting conditions
- **No depth data**: Verify that depth camera plugin is loaded correctly
- **Distorted images**: Check camera calibration parameters

### IMU Problems
- **Drifting values**: Check noise parameters and coordinate frame alignment
- **Incorrect orientation**: Verify sensor mounting and frame transformations
- **High latency**: Reduce update rate or optimize processing pipeline

## Summary

In this chapter, you learned how to implement and integrate sensor simulation into your digital twin:

- How to configure LiDAR sensors for environment mapping
- How to set up depth cameras for 3D perception
- How to implement IMU sensors for orientation tracking
- How to process and visualize sensor data
- How to validate sensor simulation quality

Sensor simulation completes the perception pipeline of your digital twin, enabling development and testing of complex robotics algorithms in a safe, controlled environment. In the next chapter, we'll explore how to integrate all components into a complete digital twin system.

## Next Steps

Continue to Chapter 4: Digital Twin Integration to learn how to combine physics simulation, visualization, and sensor systems into a complete digital twin architecture.