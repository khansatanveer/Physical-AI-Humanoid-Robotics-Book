# Chapter 1: Physics Fundamentals Exercises

## Exercise 1: Gravity Validation
**Objective**: Verify that gravity is working correctly in your simulation environment.

### Steps:
1. Launch the basic physics world: `gz sim -r basic_physics.sdf`
2. Spawn a sphere at position (0, 0, 3) with the command:
   ```bash
   gz model -f https://fuel.gazebosim.org/1.0/openrobotics/models/Sphere -m sphere_1 -x 0 -y 0 -z 3
   ```
3. Observe the sphere's motion as it falls
4. Use the Gazebo GUI to measure the time it takes to hit the ground

### Expected Outcome:
- The sphere should accelerate as it falls
- The sphere should hit the ground after approximately 0.78 seconds (for 3m height with g=9.81)
- The sphere should bounce and eventually come to rest

### Validation:
- Record the time to ground impact
- Compare with theoretical calculation: t = √(2h/g) = √(2*3/9.81)
- Document any differences and potential causes

## Exercise 2: Collision Behavior Analysis
**Objective**: Understand how different objects interact when colliding.

### Steps:
1. Launch the physics world with multiple objects
2. Spawn objects with different masses at the same height:
   - Light object (1kg): `gz model -f https://fuel.gazebosim.org/1.0/openrobotics/models/Sphere -m light_sphere -x 1 -y 0 -z 2`
   - Heavy object (10kg): `gz model -f https://fuel.gazebosim.org/1.0/openrobotics/models/Cube -m heavy_cube -x 1.5 -y 0.5 -z 2`
3. Observe and compare their fall behavior and impact

### Expected Outcome:
- Both objects should fall at the same rate (independent of mass)
- Both objects should hit the ground at approximately the same time
- Impact behavior might differ due to different shapes and masses

### Validation:
- Measure the time to impact for both objects
- Compare the bounce behavior after impact
- Explain any differences observed

## Exercise 3: Humanoid Robot Physics Check
**Objective**: Validate that the humanoid robot model behaves correctly under gravity.

### Steps:
1. Launch the basic physics simulation with the humanoid:
   ```bash
   ros2 launch digital_twin_examples basic_physics.launch.py
   ```
2. Observe the robot's initial behavior when spawned at 1m height
3. Check that all joints maintain proper constraints
4. Verify that the robot's center of mass is stable

### Expected Outcome:
- The robot should fall as a coherent unit
- Joints should not exceed their defined limits
- The robot should land in a physically plausible pose
- No parts should detach or behave erratically

### Validation:
- Check that joint angles remain within limits (use `ros2 topic echo /basic_humanoid/joint_states`)
- Verify that the robot maintains its structural integrity
- Document any joint limit violations or instability

## Exercise 4: Physics Parameter Tuning
**Objective**: Understand how physics parameters affect simulation behavior.

### Steps:
1. Create a modified world file with different gravity (e.g., 0 0 -4.9 for half Earth gravity)
2. Run the same experiments as in Exercise 1
3. Compare the results with normal gravity conditions
4. Try with higher gravity (e.g., 0 0 -19.6 for double Earth gravity)

### Expected Outcome:
- Lower gravity: Objects fall more slowly, longer time to impact
- Higher gravity: Objects fall faster, shorter time to impact
- The relationship should be proportional to √(2/g)

### Validation:
- Measure and record fall times for different gravity values
- Calculate the theoretical times and compare with simulation
- Plot fall time vs. gravity to verify the relationship

## Exercise 5: Complex Physics Interaction
**Objective**: Test physics with multiple simultaneous interactions.

### Steps:
1. Create a scenario with multiple objects at different heights
2. Add a ramp or incline to the environment
3. Launch several objects simultaneously to create collision cascades
4. Observe how the physics engine handles multiple simultaneous collisions

### Expected Outcome:
- Objects should interact realistically with each other
- Collisions should conserve momentum approximately
- The simulation should remain stable even with many interactions

### Validation:
- Check for any objects passing through each other
- Verify that the simulation doesn't become unstable
- Document any unexpected behaviors

## Submission Requirements:
For each exercise, provide:
1. A brief description of what you did
2. The results you observed
3. Any measurements taken
4. Analysis of whether the results matched expectations
5. Screenshots or recorded videos of key behaviors (optional but recommended)

## Grading Criteria:
- **Excellent (A)**: All exercises completed with thorough analysis and additional insights
- **Good (B)**: All exercises completed with good analysis and understanding
- **Satisfactory (C)**: All exercises completed with basic understanding
- **Needs Improvement (F)**: Exercises incomplete or with significant misunderstandings