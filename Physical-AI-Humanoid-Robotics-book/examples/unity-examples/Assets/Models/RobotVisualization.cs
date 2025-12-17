// Robot Visualization Model for Digital Twin
// Synchronizes with Gazebo physics model via ROS communication

using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class JointInfo
{
    public string jointName;
    public Transform jointTransform;
    public JointType jointType;
    public float position;
    public float velocity;
    public float effort;
    public float minLimit = -1.57f;
    public float maxLimit = 1.57f;
    public Vector3 rotationAxis = Vector3.right; // Default rotation axis
}

public enum JointType
{
    Revolute,
    Continuous,
    Prismatic,
    Fixed
}

public class RobotVisualization : MonoBehaviour
{
    [Header("Robot Configuration")]
    public string robotName = "basic_humanoid";
    public List<JointInfo> joints = new List<JointInfo>();

    [Header("Coordinate Conversion")]
    public bool useRosCoordinateConversion = true;

    [Header("Performance Settings")]
    public float maxUpdateRate = 50.0f; // Hz - matches Gazebo physics update rate
    private float updateInterval;
    private float lastUpdateTime;

    void Start()
    {
        updateInterval = 1.0f / maxUpdateRate;
        lastUpdateTime = 0;

        // Initialize joint information based on the basic humanoid model
        InitializeJoints();
    }

    void Update()
    {
        // Limit update frequency to match physics simulation rate
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateRobotVisualization();
            lastUpdateTime = Time.time;
        }
    }

    void InitializeJoints()
    {
        // Initialize joint information based on the basic humanoid URDF
        // This matches the joint structure from basic_humanoid.urdf

        // Head joint
        AddJoint("head_joint", "Head", JointType.Revolute,
                 minLimit: -0.5f, maxLimit: 0.5f, rotationAxis: Vector3.right);

        // Left arm joints
        AddJoint("left_shoulder_joint", "LeftUpperArm", JointType.Revolute,
                 minLimit: -1.57f, maxLimit: 1.57f, rotationAxis: Vector3.forward);
        AddJoint("left_elbow_joint", "LeftLowerArm", JointType.Revolute,
                 minLimit: 0, maxLimit: 2.0f, rotationAxis: Vector3.forward);

        // Right arm joints
        AddJoint("right_shoulder_joint", "RightUpperArm", JointType.Revolute,
                 minLimit: -1.57f, maxLimit: 1.57f, rotationAxis: Vector3.forward);
        AddJoint("right_elbow_joint", "RightLowerArm", JointType.Revolute,
                 minLimit: 0, maxLimit: 2.0f, rotationAxis: Vector3.forward);

        // Left leg joints
        AddJoint("left_hip_joint", "LeftUpperLeg", JointType.Revolute,
                 minLimit: -1.57f, maxLimit: 1.57f, rotationAxis: Vector3.forward);
        AddJoint("left_knee_joint", "LeftLowerLeg", JointType.Revolute,
                 minLimit: 0, maxLimit: 1.57f, rotationAxis: Vector3.forward);

        // Right leg joints
        AddJoint("right_hip_joint", "RightUpperLeg", JointType.Revolute,
                 minLimit: -1.57f, maxLimit: 1.57f, rotationAxis: Vector3.forward);
        AddJoint("right_knee_joint", "RightLowerLeg", JointType.Revolute,
                 minLimit: 0, maxLimit: 1.57f, rotationAxis: Vector3.forward);
    }

    void AddJoint(string jointName, string transformName, JointType type,
                 float minLimit = -1.57f, float maxLimit = 1.57f, Vector3 rotationAxis = default)
    {
        Transform jointTransform = FindTransform(transformName);
        if (jointTransform != null)
        {
            JointInfo joint = new JointInfo
            {
                jointName = jointName,
                jointTransform = jointTransform,
                jointType = type,
                minLimit = minLimit,
                maxLimit = maxLimit
            };

            if (rotationAxis != default(Vector3))
            {
                joint.rotationAxis = rotationAxis;
            }

            joints.Add(joint);
        }
        else
        {
            Debug.LogWarning($"Could not find transform '{transformName}' for joint '{jointName}'");
        }
    }

    Transform FindTransform(string name)
    {
        Transform[] allTransforms = GetComponentsInChildren<Transform>();
        foreach (Transform t in allTransforms)
        {
            if (t.name == name || t.name.ToLower().Contains(name.ToLower()))
            {
                return t;
            }
        }
        return null;
    }

    public void UpdateRobotJoints(float[] jointPositions)
    {
        // Update joint positions based on ROS joint state messages
        for (int i = 0; i < Mathf.Min(joints.Count, jointPositions.Length); i++)
        {
            JointInfo joint = joints[i];

            // Clamp position to limits
            float clampedPosition = Mathf.Clamp(jointPositions[i], joint.minLimit, joint.maxLimit);
            joint.position = clampedPosition;

            // Apply rotation based on joint type and axis
            if (joint.jointTransform != null)
            {
                // Convert radians to degrees for Unity
                float rotationDegrees = clampedPosition * Mathf.Rad2Deg;

                // Apply rotation around the specified axis
                Quaternion targetRotation = Quaternion.AngleAxis(rotationDegrees, joint.rotationAxis);

                // Apply the rotation relative to the joint's initial rotation
                joint.jointTransform.localRotation = targetRotation;
            }
        }
    }

    void UpdateRobotVisualization()
    {
        // This method would be called periodically to update the visualization
        // based on the latest joint positions received from ROS/Gazebo
    }

    // Method to convert ROS coordinates to Unity coordinates
    public Vector3 RosToUnity(Vector3 rosVector)
    {
        if (!useRosCoordinateConversion)
        {
            return rosVector;
        }

        // ROS: X forward, Y left, Z up
        // Unity: X right, Y up, Z forward
        return new Vector3(rosVector.z, rosVector.x, rosVector.y);
    }

    // Method to convert Unity coordinates to ROS coordinates
    public Vector3 UnityToRos(Vector3 unityVector)
    {
        if (!useRosCoordinateConversion)
        {
            return unityVector;
        }

        // Unity: X right, Y up, Z forward
        // ROS: X forward, Y left, Z up
        return new Vector3(unityVector.y, unityVector.z, unityVector.x);
    }

    // Method to convert ROS quaternion to Unity quaternion
    public Quaternion RosToUnity(Quaternion rosQuaternion)
    {
        if (!useRosCoordinateConversion)
        {
            return rosQuaternion;
        }

        // ROS quaternion to Unity quaternion conversion
        return new Quaternion(rosQuaternion.z, rosQuaternion.x, rosQuaternion.y, rosQuaternion.w);
    }

    // Method to get current joint positions (for debugging or publishing back to ROS)
    public float[] GetCurrentJointPositions()
    {
        float[] positions = new float[joints.Count];
        for (int i = 0; i < joints.Count; i++)
        {
            positions[i] = joints[i].position;
        }
        return positions;
    }

    // Method to reset robot to initial position
    public void ResetToInitialPosition()
    {
        foreach (JointInfo joint in joints)
        {
            if (joint.jointTransform != null)
            {
                joint.jointTransform.localRotation = Quaternion.identity;
                joint.position = 0;
            }
        }
    }

#if UNITY_EDITOR
    // Draw gizmos to visualize joint axes in the editor
    void OnDrawGizmosSelected()
    {
        foreach (JointInfo joint in joints)
        {
            if (joint.jointTransform != null)
            {
                Gizmos.color = Color.red;
                Vector3 axisWorld = joint.jointTransform.TransformDirection(joint.rotationAxis);
                Gizmos.DrawLine(joint.jointTransform.position,
                              joint.jointTransform.position + axisWorld * 0.2f);
            }
        }
    }
#endif
}