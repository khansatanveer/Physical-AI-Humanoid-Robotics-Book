# Troubleshooting Guide for Digital Twin Integration

## Overview
This guide provides solutions for common issues encountered when setting up and running the complete digital twin system with Gazebo physics simulation, Unity visualization, and sensor integration.

## Common Issues and Solutions

### 1. Connection and Communication Issues

#### Issue: Gazebo and Unity not communicating
**Symptoms:**
- Unity scene shows static robot while Gazebo robot moves
- No sensor data appears in Unity
- Bridge node shows connection errors

**Solutions:**
1. **Check Network Configuration**
   ```bash
   # Verify IP addresses match between ROS 2 and Unity
   hostname -I  # Check current IP
   # Update launch file with correct IP if needed
   ```

2. **Verify Firewall Settings**
   - Open required ports (5555 for Unity bridge, ROS 2 ports)
   - Check if antivirus is blocking connections

3. **Check ROS 2 Environment**
   ```bash
   # Ensure ROS 2 environment is properly sourced
   echo $ROS_DOMAIN_ID  # Should match between all terminals
   ros2 topic list      # Verify topics are being published
   ```

#### Issue: ROS 2 nodes not connecting
**Symptoms:**
- Nodes show "no data" for topics
- Error messages about unreachable nodes
- High network latency

**Solutions:**
1. **Domain ID Issues**
   ```bash
   export ROS_DOMAIN_ID=0  # Use consistent domain ID
   ```

2. **DDS Configuration**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Try different RMW
   ```

### 2. Performance Issues

#### Issue: Low simulation rate in Gazebo
**Symptoms:**
- Real-time factor < 0.5
- Jittery or slow simulation
- High CPU usage

**Solutions:**
1. **Optimize Physics Settings**
   ```xml
   <!-- In URDF/SDF, reduce complexity -->
   <max_contacts>10</max_contacts>  <!-- Reduce contact points -->
   <ode>
     <solver>
       <type>quick</type>
       <iters>20</iters>  <!-- Reduce solver iterations -->
     </solver>
   </ode>
   ```

2. **Reduce Sensor Complexity**
   - Lower LiDAR resolution (reduce samples)
   - Reduce camera resolution temporarily
   - Decrease sensor update rates

#### Issue: Low Unity frame rate
**Symptoms:**
- Unity FPS < 30
- Laggy visualization
- Delayed response to changes

**Solutions:**
1. **Optimize Visualization Settings**
   - Reduce point cloud density for LiDAR
   - Use Level of Detail (LOD) for complex models
   - Disable unnecessary visual effects

2. **Reduce Data Rate**
   - Lower sensor update frequency
   - Implement data decimation in visualization

### 3. Sensor-Specific Issues

#### Issue: LiDAR not publishing data
**Symptoms:**
- `/scan` topic shows no messages
- Empty or all-zero ranges array
- Sensor appears in Gazebo but no data output

**Solutions:**
1. **Check Plugin Configuration**
   ```xml
   <!-- Verify plugin is properly loaded -->
   <plugin name="lidar_controller" filename="libgazebo_ros_ray.so">
     <ros>
       <namespace>/basic_humanoid</namespace>
       <remapping>~/out:=scan</remapping>
     </ros>
     <output_type>sensor_msgs/LaserScan</output_type>
     <frame_name>lidar_link</frame_name>
   </plugin>
   ```

2. **Verify Coordinate Frames**
   ```bash
   ros2 run tf2_tools view_frames  # Check if frames exist
   ros2 run rqt_tf_tree rqt_tf_tree  # Visualize frame tree
   ```

#### Issue: Depth camera showing black images
**Symptoms:**
- Camera images are completely black
- Depth data shows invalid values
- No point cloud generation

**Solutions:**
1. **Check Camera Configuration**
   ```xml
   <!-- Ensure proper camera settings -->
   <camera name="head_camera">
     <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
     <image>
       <width>640</width>
       <height>480</height>
       <format>R8G8B8</format>
     </image>
     <clip>
       <near>0.1</near>  <!-- Check near/far clipping -->
       <far>10</far>
     </clip>
   </camera>
   ```

2. **Verify Lighting Conditions**
   - Ensure Gazebo world has adequate lighting
   - Check if camera is pointed at empty space

#### Issue: IMU showing drifting values
**Symptoms:**
- Orientation quaternion values drift over time
- Acceleration values don't match expected gravity
- High noise levels in static conditions

**Solutions:**
1. **Check Noise Parameters**
   ```xml
   <!-- Verify realistic noise values -->
   <angular_velocity>
     <x>
       <noise type="gaussian">
         <stddev>2e-4</stddev>  <!-- Appropriate for IMU -->
       </noise>
     </x>
   </angular_velocity>
   ```

2. **Validate Coordinate Frames**
   - Ensure IMU is properly mounted
   - Check that gravity aligns with Z-axis

### 4. Integration Issues

#### Issue: Robot states not synchronizing
**Symptoms:**
- Robot pose differs between Gazebo and Unity
- Joint positions don't match between systems
- Timing synchronization problems

**Solutions:**
1. **Check Coordinate System Conversion**
   ```csharp
   // In Unity, verify coordinate conversion
   public Vector3 RosToUnity(Vector3 rosVector)
   {
       // ROS: X forward, Y left, Z up
       // Unity: X right, Y up, Z forward
       return new Vector3(rosVector.z, rosVector.x, rosVector.y);
   }
   ```

2. **Verify Time Synchronization**
   - Ensure both systems use the same time source
   - Check for clock differences between machines

#### Issue: Bridge node crashing or disconnecting
**Symptoms:**
- Bridge node stops unexpectedly
- Connection timeouts between systems
- Data loss during operation

**Solutions:**
1. **Implement Connection Recovery**
   ```python
   def connect_to_unity(self):
       while not self.unity_socket and not self.shutdown_flag:
           try:
               self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               self.unity_socket.connect((self.unity_ip, self.unity_port))
               self.unity_socket.settimeout(0.1)
               break
           except Exception as e:
               self.get_logger().error(f'Connection failed: {e}')
               time.sleep(1)  # Wait before retry
   ```

2. **Check Resource Limits**
   - Monitor memory usage
   - Check network buffer sizes
   - Verify file descriptor limits

### 5. System-Level Issues

#### Issue: High CPU or memory usage
**Symptoms:**
- System becomes unresponsive
- High resource utilization
- Slow performance across all components

**Solutions:**
1. **Optimize Update Rates**
   ```python
   # Reduce update rates where possible
   self.ros_publish_rate = 30  # Instead of higher rates
   ```

2. **Implement Data Throttling**
   - Use message filters to reduce data rate
   - Implement sampling for high-frequency data

#### Issue: Intermittent failures
**Symptoms:**
- System works sometimes, fails other times
- Random disconnections
- Unpredictable behavior

**Solutions:**
1. **Add Comprehensive Error Handling**
   ```python
   def send_to_unity(self):
       try:
           # Send data
           self.unity_socket.send(json_data.encode('utf-8') + b'\n')
       except socket.error as e:
           self.get_logger().warning(f'Socket error: {e}')
           self.reconnect_to_unity()
       except Exception as e:
           self.get_logger().error(f'General error: {e}')
   ```

2. **Check System Stability**
   - Monitor system logs for errors
   - Check for resource exhaustion
   - Verify hardware stability

## Diagnostic Tools

### ROS 2 Diagnostic Commands
```bash
# Check system status
ros2 topic list
ros2 node list
ros2 lifecycle list

# Monitor specific topics
ros2 topic echo /basic_humanoid/scan
ros2 topic hz /basic_humanoid/joint_states

# Check network connections
ros2 doctor
```

### Performance Monitoring
```bash
# System resources
htop
iotop
nethogs

# Gazebo-specific
gz stats
gz topic -e -t /statistics

# Unity performance (in Unity editor)
# Window -> Analysis -> Profiler
```

## Prevention Strategies

### 1. Pre-Operation Checks
- Verify all required services are running
- Check network connectivity
- Validate configuration files
- Monitor system resources

### 2. Continuous Monitoring
- Implement health checks in bridge nodes
- Monitor data quality metrics
- Track performance metrics over time
- Log all system events

### 3. Backup Plans
- Prepare fallback configurations
- Document manual recovery procedures
- Maintain configuration backups
- Plan for graceful degradation

## When to Seek Additional Help

Contact support or consult additional documentation when:
- Issues persist after following troubleshooting steps
- Errors occur during system startup
- Performance is significantly below requirements
- Security or stability concerns arise

## Quick Reference

### Common Commands
```bash
# Restart ROS 2 daemon
pkill -f ros
ros2 daemon stop && ros2 daemon start

# Check all topics
ros2 topic list -t -v

# Monitor specific node
ros2 run rqt_graph rqt_graph
```

### Emergency Procedures
1. **Immediate System Stop**
   - Ctrl+C in all terminals
   - Close Unity application
   - Kill any remaining processes

2. **Configuration Reset**
   - Restart ROS 2 daemon
   - Reload default configurations
   - Clear temporary files

This troubleshooting guide should help resolve most common issues with the digital twin system. For complex problems, consider the systematic approach: identify symptoms, isolate the component, test the connection, and implement the appropriate solution.