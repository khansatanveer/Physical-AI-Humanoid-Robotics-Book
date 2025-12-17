# Gravity Demonstration Example

## Overview
This example demonstrates how gravity affects objects in the Gazebo simulation environment. Understanding gravity is fundamental to creating realistic physics simulations for digital twins.

## Physics Parameters
- Gravity: 0 0 -9.81 m/s² (Earth's gravitational acceleration)
- Simulation step size: 0.001 seconds
- Real-time factor: 1.0 (simulation runs at real-time speed)

## Expected Behavior
When an object is placed in the simulation:
1. It accelerates downward at 9.81 m/s²
2. The velocity increases linearly with time
3. The position follows a quadratic relationship with time

## Mathematical Model
The motion of an object under gravity follows:
- v = v₀ + gt (velocity over time)
- s = s₀ + v₀t + ½gt² (position over time)

Where:
- v = velocity
- s = position
- g = gravitational acceleration (-9.81 m/s²)
- t = time
- Subscript ₀ indicates initial values

## Validation Criteria
To verify that gravity is working correctly:
1. Objects should fall when not supported
2. Falling objects should accelerate (not move at constant speed)
3. Objects should reach the ground faster than in zero-gravity simulation
4. Impact velocity should be consistent with gravitational acceleration

## Troubleshooting
- If objects don't fall: Check that gravity is set correctly in the world file
- If objects fall too slowly: Verify that the simulation step size is appropriate
- If objects fall through the ground: Check collision properties of both objects