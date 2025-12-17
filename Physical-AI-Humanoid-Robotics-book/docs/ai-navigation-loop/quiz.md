# Quiz: Complete AI-Driven Navigation Loop

## Question 1: What are the three main components of the AI-driven navigation loop?

A) Perception, Planning, Control
B) Sensing, Mapping, Localization
C) Detection, Tracking, Prediction
D) Vision, Navigation, Execution

**Answer: A) Perception, Planning, Control**

The AI-driven navigation loop consists of three main components: Perception (understanding the environment through sensors), Planning (determining how to reach goals while avoiding obstacles), and Control (executing the planned actions on the physical robot).

## Question 2: Which of the following is NOT a key challenge specific to humanoid robot navigation compared to wheeled robots?

A) Bipedal locomotion requiring dynamic balance
B) Non-holonomic constraints limiting sideways movement
C) Turning in place using differential drive
D) Step planning for individual foot placement

**Answer: C) Turning in place using differential drive**

Differential drive turning is a capability of wheeled robots, not humanoid robots. Humanoid robots face unique challenges like maintaining balance during bipedal locomotion, planning individual steps, and managing center of mass, which are not concerns for wheeled robots.

## Question 3: What is the primary purpose of the perception component in the navigation loop?

A) To plan optimal paths to goals
B) To execute planned movements on the robot
C) To understand the environment through sensors
D) To manage robot balance and stability

**Answer: C) To understand the environment through sensors**

The perception component's primary purpose is to process sensor data (cameras, LIDAR, IMU, etc.) to understand the environment, detect obstacles, identify objects, and create environmental models that can be used by the planning component.

## Question 4: In the context of Nav2 for humanoid robots, what does the "step_size" parameter typically control?

A) The height of each step the robot takes
B) The maximum distance between consecutive footsteps
C) The duration of each step
D) The width of the robot's feet

**Answer: B) The maximum distance between consecutive footsteps**

The "step_size" parameter in humanoid navigation controls the maximum distance between consecutive footsteps, which is critical for maintaining balance and ensuring stable locomotion for bipedal robots.

## Question 5: Which ROS 2 message type is commonly used for sending navigation goals to Nav2?

A) geometry_msgs/Pose
B) nav_msgs/Path
C) nav2_msgs/action/NavigateToPose
D) sensor_msgs/LaserScan

**Answer: C) nav2_msgs/action/NavigateToPose**

The NavigateToPose action is the primary interface for sending navigation goals to Nav2. It allows clients to send a target pose goal and receive feedback and results about the navigation execution.

## Question 6: What is the main purpose of the local costmap in Nav2?

A) To store the global map of the environment
B) To track the robot's current position
C) To maintain a map of obstacles in the immediate vicinity for local planning
D) To store the planned path to the goal

**Answer: C) To maintain a map of obstacles in the immediate vicinity for local planning**

The local costmap maintains a map of obstacles in the robot's immediate vicinity and is used by the local planner to avoid obstacles and navigate safely in real-time.

## Question 7: Which of the following is a humanoid-specific navigation challenge?

A) Wheel slippage on smooth surfaces
B) Maintaining dynamic balance during movement
C) Differential drive kinematics
D) Three-wheeled omni-directional movement

**Answer: B) Maintaining dynamic balance during movement**

Maintaining dynamic balance during movement is a unique challenge for humanoid robots due to their bipedal locomotion, which requires continuous balance management unlike wheeled robots that have stable contact with the ground.

## Question 8: What is the role of the controller server in Nav2?

A) To plan global paths from start to goal
B) To follow the planned path by generating velocity commands
C) To detect obstacles in the environment
D) To maintain the global map

**Answer: B) To follow the planned path by generating velocity commands**

The controller server in Nav2 is responsible for following the path provided by the global planner by generating appropriate velocity commands to control the robot's movement.

## Question 9: In sensor fusion for navigation, what is the primary benefit of combining multiple sensor types?

A) Reduced computational requirements
B) Increased reliability and robustness in various conditions
C) Lower hardware costs
D) Simpler software implementation

**Answer: B) Increased reliability and robustness in various conditions**

Combining multiple sensor types through sensor fusion provides increased reliability and robustness because if one sensor fails or performs poorly in certain conditions, other sensors can compensate, leading to more reliable navigation.

## Question 10: What is the typical frequency relationship between global planning and local planning in Nav2?

A) Global planning runs more frequently than local planning
B) Local planning runs more frequently than global planning
C) Both run at the same frequency
D) Frequency is not important in navigation systems

**Answer: B) Local planning runs more frequently than global planning**

Local planning typically runs at a higher frequency (e.g., 10-20 Hz) than global planning (e.g., 1-5 Hz) because local planning needs to react quickly to immediate obstacles and changes in the environment, while global planning updates the overall route less frequently.