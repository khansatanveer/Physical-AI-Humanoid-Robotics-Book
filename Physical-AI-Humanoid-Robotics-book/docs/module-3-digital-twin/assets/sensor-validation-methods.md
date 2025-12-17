# Sensor Simulation Validation Methods

## Overview
This document outlines the methods and criteria for validating that the sensor simulation in the digital twin produces realistic and accurate data outputs. Proper validation ensures that the simulated sensors behave similarly to their real-world counterparts.

## LiDAR Sensor Validation

### Physical Accuracy Validation
1. **Range Validation**:
   - Verify that measured distances match actual distances in the environment
   - Test with objects at various distances (0.1m to 30m range)
   - Check for consistent accuracy across the entire range

2. **Angular Resolution Validation**:
   - Confirm that the sensor can distinguish between closely spaced objects
   - Test with objects at different angles relative to the sensor
   - Verify that the 1-degree resolution produces expected separation

3. **Noise Characterization**:
   - Measure the noise level in static conditions
   - Verify that noise follows expected statistical patterns
   - Compare noise levels to real LiDAR specifications

### Performance Validation
1. **Update Rate Consistency**:
   - Measure actual update rate vs. configured rate (10Hz)
   - Check for dropped messages or timing irregularities
   - Validate that processing doesn't affect simulation timing

2. **Data Throughput**:
   - Measure message size and transmission rate
   - Verify that the system can handle 360 range measurements efficiently
   - Test performance under various environmental complexity

### Environmental Validation
1. **Surface Material Effects**:
   - Test with different surface reflectivities
   - Verify that dark or transparent surfaces behave appropriately
   - Check that highly reflective surfaces don't cause artifacts

2. **Occlusion Handling**:
   - Verify that objects properly block laser beams
   - Test multi-path effects with complex geometries
   - Confirm that only visible surfaces are detected

## Depth Camera Validation

### Geometric Accuracy
1. **Depth Accuracy**:
   - Compare measured depths to actual distances
   - Test accuracy across the full depth range (0.1m to 10m)
   - Verify accuracy varies predictably with distance (more error at range)

2. **Field of View Validation**:
   - Confirm 60-degree horizontal FOV matches specifications
   - Test that objects appear at correct positions in image
   - Verify that the aspect ratio matches 640x480 resolution

3. **Resolution and Detail**:
   - Test ability to resolve small objects at various distances
   - Verify that fine details are preserved appropriately
   - Check for aliasing or artifacts in the depth data

### Performance Validation
1. **Frame Rate Consistency**:
   - Verify 30 FPS output rate is maintained
   - Test frame timing stability under different lighting
   - Check for dropped frames during complex scenes

2. **Memory and Processing**:
   - Measure memory usage for depth image buffers
   - Validate processing time for depth calculations
   - Test performance with multiple depth cameras

### Environmental Effects
1. **Lighting Conditions**:
   - Test performance under different lighting scenarios
   - Verify depth accuracy in bright/dark conditions
   - Check for artifacts caused by strong light sources

2. **Transparency and Reflection**:
   - Test behavior with transparent objects
   - Verify correct handling of reflective surfaces
   - Check for ghosting or multiple returns

## IMU Sensor Validation

### Static Validation
1. **Gravity Measurement**:
   - Verify that static Z-axis reads ~9.8 m/s² (gravity)
   - Confirm X and Y axes read ~0 m/s² when level
   - Test with the sensor at different orientations

2. **Orientation Accuracy**:
   - Compare measured orientation to known positions
   - Verify quaternion normalization (unit length)
   - Test conversion accuracy between representations

### Dynamic Validation
1. **Rotation Detection**:
   - Test angular velocity measurements during rotation
   - Verify integration of angular velocity matches orientation
   - Check for drift during extended rotation tests

2. **Acceleration Detection**:
   - Test linear acceleration measurements during movement
   - Verify that gravity compensation works correctly
   - Check cross-axis sensitivity and coupling

### Noise and Drift Validation
1. **Noise Characteristics**:
   - Measure noise levels in static conditions
   - Verify noise follows Gaussian distribution
   - Compare to specified noise parameters

2. **Drift Analysis**:
   - Monitor long-term stability of measurements
   - Check for temperature or time-dependent drift
   - Verify that drift is within acceptable bounds

## Multi-Sensor Validation

### Cross-Sensor Consistency
1. **Temporal Synchronization**:
   - Verify that sensors are properly timestamped
   - Check that data from different sensors can be correlated
   - Test time synchronization accuracy

2. **Spatial Consistency**:
   - Confirm that sensor positions match physical model
   - Verify coordinate system transformations
   - Test consistency between different sensor measurements of same features

### Sensor Fusion Validation
1. **Fused Output Quality**:
   - Compare fused results to individual sensor outputs
   - Verify that fusion improves accuracy/reliability
   - Test failure modes and graceful degradation

2. **Performance Metrics**:
   - Measure computational overhead of fusion
   - Verify real-time performance is maintained
   - Test resource usage under full sensor load

## Validation Tools and Scripts

### Automated Validation Script
```python
#!/usr/bin/env python3
# sensor_validation_test.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
from collections import deque

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Data collection
        self.lidar_samples = deque(maxlen=100)
        self.imu_samples = deque(maxlen=100)
        self.fusion_samples = deque(maxlen=100)

        # Validation results
        self.validation_results = {}

        # Create subscribers
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/basic_humanoid/scan', self.validate_lidar, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/basic_humanoid/imu', self.validate_imu, 10)
        self.fusion_subscription = self.create_subscription(
            Float32MultiArray, '/basic_humanoid/fused_sensor_state',
            self.validate_fusion, 10)

        # Timer for periodic validation
        self.timer = self.create_timer(5.0, self.run_periodic_validation)

        self.get_logger().info('Sensor Validator initialized')

    def validate_lidar(self, msg):
        """Validate LiDAR data quality"""
        # Check for valid ranges
        valid_ranges = [r for r in msg.ranges if 0.1 <= r <= 30.0]
        range_validity = len(valid_ranges) / len(msg.ranges) if msg.ranges else 0

        # Check for reasonable values
        if valid_ranges:
            avg_range = sum(valid_ranges) / len(valid_ranges)
            self.validation_results['lidar_avg_range'] = avg_range
            self.validation_results['lidar_range_validity'] = range_validity

        self.lidar_samples.append({
            'timestamp': time.time(),
            'validity_ratio': range_validity,
            'avg_range': np.mean(valid_ranges) if valid_ranges else 0
        })

    def validate_imu(self, msg):
        """Validate IMU data quality"""
        # Check quaternion normalization
        quat_norm = np.sqrt(
            msg.orientation.x**2 + msg.orientation.y**2 +
            msg.orientation.z**2 + msg.orientation.w**2
        )
        quat_normalized = abs(quat_norm - 1.0) < 0.01

        # Check gravity in Z-axis when static
        linear_acc_magnitude = np.sqrt(
            msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )
        gravity_reasonable = abs(linear_acc_magnitude - 9.8) < 1.0  # Allow for movement

        self.validation_results['imu_quat_normalized'] = quat_normalized
        self.validation_results['imu_gravity_reasonable'] = gravity_reasonable

        self.imu_samples.append({
            'timestamp': time.time(),
            'quat_normalized': quat_normalized,
            'gravity_reasonable': gravity_reasonable
        })

    def validate_fusion(self, msg):
        """Validate fused sensor state"""
        if len(msg.data) >= 10:  # Expected format
            # Check position reasonableness
            pos_magnitude = np.sqrt(msg.data[0]**2 + msg.data[1]**2 + msg.data[2]**2)
            pos_reasonable = pos_magnitude < 100.0  # Within 100m workspace

            # Check quaternion normalization
            quat_norm = np.sqrt(msg.data[3]**2 + msg.data[4]**2 +
                              msg.data[5]**2 + msg.data[6]**2)
            quat_normalized = abs(quat_norm - 1.0) < 0.01

            self.validation_results['fusion_pos_reasonable'] = pos_reasonable
            self.validation_results['fusion_quat_normalized'] = quat_normalized

            self.fusion_samples.append({
                'timestamp': time.time(),
                'pos_reasonable': pos_reasonable,
                'quat_normalized': quat_normalized
            })

    def run_periodic_validation(self):
        """Run periodic validation checks"""
        self.get_logger().info('Running periodic sensor validation...')

        # LiDAR validation summary
        if self.lidar_samples:
            avg_validity = np.mean([s['validity_ratio'] for s in self.lidar_samples])
            self.get_logger().info(f'LiDAR: {avg_validity*100:.1f}% valid ranges')

        # IMU validation summary
        if self.imu_samples:
            norm_ratio = np.mean([s['quat_normalized'] for s in self.imu_samples])
            self.get_logger().info(f'IMU: {norm_ratio*100:.1f}% normalized quaternions')

        # Fusion validation summary
        if self.fusion_samples:
            pos_ratio = np.mean([s['pos_reasonable'] for s in self.fusion_samples])
            quat_ratio = np.mean([s['quat_normalized'] for s in self.fusion_samples])
            self.get_logger().info(f'Fusion: {pos_ratio*100:.1f}% reasonable positions, {quat_ratio*100:.1f}% normalized quaternions')

        # Overall validation status
        overall_pass = all([
            avg_validity > 0.95 if self.lidar_samples else True,
            norm_ratio > 0.99 if self.imu_samples else True,
            pos_ratio > 0.99 if self.fusion_samples else True
        ])

        status = "PASSED" if overall_pass else "FAILED"
        self.get_logger().info(f'Overall Validation Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Validation stopped by user')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Validation Checklist

#### LiDAR Validation Checklist
- [ ] Range values are within 0.1m to 30m limits
- [ ] All 360 angles are represented in scan
- [ ] Update rate is approximately 10Hz
- [ ] Noise levels are realistic
- [ ] Distant objects show expected resolution loss
- [ ] Transparent/reflective surfaces behave correctly
- [ ] Sensor mounting position matches URDF

#### Depth Camera Validation Checklist
- [ ] Image resolution is 640x480
- [ ] Depth values are in meters
- [ ] Horizontal FOV is approximately 60 degrees
- [ ] Update rate is approximately 30Hz
- [ ] Depth accuracy degrades predictably with distance
- [ ] Color and depth images are properly synchronized
- [ ] Sensor mounting position matches URDF

#### IMU Validation Checklist
- [ ] Quaternion values are normalized (unit length)
- [ ] Static Z-axis reads ~9.8 m/s²
- [ ] Update rate is approximately 100Hz
- [ ] Noise levels match configured parameters
- [ ] Orientation changes correctly with rotation
- [ ] Sensor mounting position matches URDF
- [ ] Covariance values are realistic

#### Multi-Sensor Validation Checklist
- [ ] All sensors publish simultaneously without conflict
- [ ] Coordinate systems are properly transformed
- [ ] Timestamps are synchronized across sensors
- [ ] Fused output is more accurate than individual sensors
- [ ] System maintains real-time performance
- [ ] Failure of one sensor doesn't break others

## Acceptance Criteria

For the sensor simulation to be considered valid:

1. **Accuracy Requirements**:
   - LiDAR: Range accuracy within 1% of actual distance
   - Depth Camera: Depth accuracy within 5% at close range, 10% at far range
   - IMU: Orientation accuracy within 1 degree when static

2. **Performance Requirements**:
   - All sensors maintain configured update rates
   - System maintains real-time simulation (≥30 FPS)
   - Sensor processing uses `<20%` of available CPU

3. **Reliability Requirements**:
   - 99%+ of sensor messages are valid
   - No systematic biases in measurements
   - Proper handling of edge cases and invalid data

## Troubleshooting Common Issues

### LiDAR Issues
- **No data published**: Check Gazebo plugin configuration and ROS remappings
- **Invalid ranges**: Verify coordinate frames and sensor mounting
- **Performance problems**: Reduce ray count or increase minimum range

### Depth Camera Issues
- **Black images**: Check camera parameters and lighting conditions
- **No depth data**: Verify depth camera plugin is loaded correctly
- **Distorted images**: Check camera calibration parameters

### IMU Issues
- **Drifting values**: Check noise parameters and coordinate frame alignment
- **Incorrect gravity**: Verify sensor orientation and gravity compensation
- **High noise**: Adjust noise parameters in configuration

## Conclusion

Proper validation of sensor simulation is critical for the digital twin's effectiveness. The methods outlined in this document provide comprehensive validation approaches for each sensor type and their integration. Regular validation should be performed to ensure continued accuracy and reliability of the sensor simulation system.