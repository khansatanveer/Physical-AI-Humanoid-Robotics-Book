# Chapter 2: Unity Environment Building and Visualization - Exercises

## Exercise 2.1: Basic Unity Scene Setup

### Objective
Set up a basic Unity scene for digital twin visualization that matches the physics simulation from Chapter 1.

### Tasks
1. Create a new Unity project named "DigitalTwinVisualization"
2. Configure project settings to match Gazebo physics update rates (50Hz)
3. Set up a basic scene with:
   - Main camera with appropriate positioning for digital twin viewing
   - Directional light matching Gazebo's lighting
   - Ground plane for reference
   - UI elements for displaying simulation status

### Implementation Steps
1. Open Unity Hub and create a new 3D (Built-in Render Pipeline) project
2. Configure Time settings: Maximum Allowed Timestep to 0.333, Fixed Timestep to 0.02
3. Create basic scene objects with appropriate transforms
4. Add the DigitalTwinCamera script to the main camera

### Validation Criteria
- Scene runs at minimum 30 FPS
- Camera follows a target object smoothly
- Lighting matches the coordinate system alignment requirements
- All objects are properly positioned in the scene

### Submission Requirements
- Screenshot of your Unity scene hierarchy
- Screenshot of the scene view showing the basic setup
- Confirmation that the camera script is properly attached and functioning

---

## Exercise 2.2: Environment Asset Creation

### Objective
Create 3D environment assets that match the Gazebo physics world from Chapter 1.

### Tasks
1. Create primitive shapes that match the obstacles in your Gazebo world:
   - A box obstacle
   - A cylindrical obstacle
   - A ramp
   - A platform
2. Apply appropriate materials to match the physics simulation
3. Position and scale objects to match the Gazebo world dimensions

### Implementation Steps
1. Create primitive GameObjects (Cube, Cylinder, Capsule, Cube)
2. Adjust position, rotation, and scale to match Gazebo world
3. Create and apply materials with appropriate colors and properties
4. Test that the visual environment matches the physics environment

### Validation Criteria
- All obstacles match the size and position of Gazebo world objects
- Materials provide clear visual distinction between different objects
- Scene hierarchy is organized and clean
- Performance remains above 30 FPS with all objects present

### Submission Requirements
- Screenshot of the Unity scene showing all environment assets
- Code snippet showing how you created and positioned the objects
- Description of the coordinate system conversion used

---

## Exercise 2.3: Robot Visualization Model

### Objective
Create a Unity visualization model that matches the URDF robot from Chapter 1.

### Tasks
1. Create a hierarchy of GameObjects that matches the URDF joint structure
2. Implement the RobotVisualization script to control joint positions
3. Apply appropriate materials to match the robot's appearance
4. Test that the visual model responds to joint position updates

### Implementation Steps
1. Create parent GameObject for the robot root
2. Create child GameObjects for each link (head, torso, arms, legs)
3. Set up the RobotVisualization component with joint information
4. Implement joint position updates based on input values
5. Test with predefined poses (standing, sitting, waving)

### Validation Criteria
- Robot hierarchy matches the URDF structure
- Joint positions update correctly when values are changed
- Robot maintains realistic proportions based on URDF dimensions
- Robot can be set to different poses programmatically

### Submission Requirements
- Screenshot of the robot GameObject hierarchy
- Code snippet showing the RobotVisualization implementation
- Demonstration of the robot in at least two different poses
- Explanation of how coordinate system conversion is handled

---

## Exercise 2.4: Real-time Synchronization Implementation

### Objective
Implement real-time synchronization between a simulated physics system and Unity visualization.

### Tasks
1. Create a mock ROS communication system to simulate joint state messages
2. Implement the DigitalTwinConnector script to receive joint positions
3. Update the robot visualization in real-time based on received data
4. Test synchronization performance and responsiveness

### Implementation Steps
1. Create a mock joint state publisher that generates realistic values
2. Implement the DigitalTwinConnector to receive and process joint data
3. Connect the connector to the RobotVisualization component
4. Add performance monitoring to track update frequency and response time

### Validation Criteria
- Joint positions update at 50Hz (or higher) to match physics simulation
- Visual updates occur with less than 100ms delay
- Performance remains stable with continuous updates
- Robot movements appear smooth and synchronized

### Submission Requirements
- Code for the mock joint state publisher
- Screenshot of the DigitalTwinConnector in the Unity inspector
- Performance metrics showing update frequency and response time
- Video or GIF showing the synchronization in action

---

## Exercise 2.5: Interactive Elements and Controls

### Objective
Add interactive elements to allow user control of the digital twin visualization.

### Tasks
1. Implement camera control system for user navigation
2. Create UI elements for controlling robot joint positions
3. Add pose buttons for predefined robot positions
4. Implement performance monitoring display

### Implementation Steps
1. Add CameraController script to the main camera
2. Create UI canvas with joint sliders and pose buttons
3. Connect UI elements to the RobotController component
4. Add performance monitoring display showing FPS and frame time

### Validation Criteria
- Camera can be controlled with mouse and keyboard
- Joint sliders update robot positions in real-time
- Pose buttons set the robot to predefined positions smoothly
- Performance display updates correctly and shows FPS > 30

### Submission Requirements
- Screenshot of the UI with controls visible
- Code snippet showing how UI elements connect to robot control
- Description of the interaction methods implemented
- Performance metrics showing the impact of UI on frame rate

---

## Exercise 2.6: Performance Optimization

### Objective
Optimize the Unity scene to meet performance targets (`<100ms` response time).

### Tasks
1. Implement Level of Detail (LOD) for complex models
2. Set up occlusion culling for static objects
3. Optimize draw calls through batching
4. Validate that performance targets are met

### Implementation Steps
1. Create LOD groups for complex robot models
2. Mark static objects for batching and lighting
3. Implement performance monitoring with frame time tracking
4. Test performance under various conditions

### Validation Criteria
- Scene maintains >30 FPS under normal conditions
- Frame time consistently stays below 100ms
- LOD system switches appropriately based on distance
- Draw calls are minimized through batching

### Submission Requirements
- Performance report showing frame times before and after optimization
- Screenshot of LOD settings in the Unity editor
- Code for performance monitoring implementation
- Description of optimization techniques applied and their impact

---

## Assessment Rubric

### Technical Implementation (60%)
- Correct implementation of all required components
- Proper use of Unity APIs and best practices
- Accurate matching between Gazebo physics and Unity visualization

### Performance (25%)
- Achievement of performance targets (`<100ms` response time)
- Efficient use of resources and optimization techniques
- Stable frame rate maintenance

### Documentation and Submission (15%)
- Clear, comprehensive code documentation
- Proper submission of all required artifacts
- Accurate validation of implementation against criteria

### Additional Challenge (Bonus 10%)
Extend the digital twin with additional features such as:
- Real-time sensor visualization (e.g., LiDAR point clouds)
- Animation blending for smoother joint movements
- Network synchronization with actual ROS/Gazebo simulation