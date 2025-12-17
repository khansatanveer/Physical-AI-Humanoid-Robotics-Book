# Data Model: Digital Twin (Gazebo & Unity)

## Entities

### Digital Twin Environment
- **Name**: String (unique identifier for the environment)
- **Description**: Text (overview of the environment purpose)
- **Gazebo Scene**: File path (URDF/SDF files for physics simulation)
- **Unity Scene**: File path (Unity scene file for visualization)
- **Physics Parameters**: Object (gravity, friction, etc.)
- **Created Date**: Timestamp
- **Validation**: Boolean (whether the environment is validated to work in both simulators)

### Humanoid Robot Model
- **Name**: String (robot name, e.g., "Atlas", "Pepper", custom)
- **URDF Path**: File path (Unified Robot Description Format file)
- **Visual Meshes**: Array of file paths (3D model files for visualization)
- **Collision Meshes**: Array of file paths (physics collision models)
- **Joints**: Array of joint configurations (position, limits, types)
- **Links**: Array of link configurations (physical properties)
- **Sensors**: Array of sensor definitions attached to the robot

### Sensor Simulation Data
- **Type**: Enum (LiDAR, Depth Camera, IMU, Camera, etc.)
- **Data Format**: String (format specification for the sensor data)
- **Frequency**: Number (updates per second)
- **Accuracy**: Object (error characteristics, noise parameters)
- **Range**: Object (min/max values for the sensor)
- **Field of View**: Object (angular coverage for visual sensors)
- **Sample Data**: File path (example data output for documentation)

### Learning Exercise
- **Title**: String (exercise name)
- **Description**: Text (what the learner will accomplish)
- **Prerequisites**: Array of string (what knowledge/skills needed)
- **Steps**: Array of step objects (ordered instructions)
- **Expected Outcome**: Text (what the learner should see/achieve)
- **Difficulty**: Enum (beginner, intermediate)
- **Estimated Time**: Number (minutes to complete)

## Relationships

- Digital Twin Environment **contains** multiple Humanoid Robot Models
- Digital Twin Environment **generates** Sensor Simulation Data
- Humanoid Robot Model **has** multiple Sensor Simulation Data streams
- Learning Exercise **uses** Digital Twin Environment
- Learning Exercise **involves** Humanoid Robot Model

## State Transitions

### Simulation State
- `CREATED` → `CONFIGURED` (when environment is set up)
- `CONFIGURED` → `RUNNING` (when simulation is started)
- `RUNNING` → `PAUSED` (when simulation is paused)
- `RUNNING` → `STOPPED` (when simulation ends normally)
- `RUNNING` → `ERROR` (when simulation fails)
- `PAUSED` → `RUNNING` (when resumed)
- `STOPPED` → `CONFIGURED` (when reset)

### Exercise State
- `NOT_STARTED` → `IN_PROGRESS` (when learner begins)
- `IN_PROGRESS` → `COMPLETED` (when learner finishes successfully)
- `IN_PROGRESS` → `FAILED` (when learner doesn't complete correctly)
- `COMPLETED` → `RESET` (when learner wants to repeat)

## Validation Rules

1. Digital Twin Environment must have both Gazebo and Unity scene files defined
2. Humanoid Robot Model URDF must be syntactically valid
3. Sensor Simulation Data frequency must be within realistic ranges
4. Learning Exercise steps must be in logical sequence
5. All file paths must exist within the project structure
6. Physics parameters must be within realistic ranges for humanoid robots
7. All dependencies between entities must be resolvable