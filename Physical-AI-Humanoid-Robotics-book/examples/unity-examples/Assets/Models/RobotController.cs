// Robot Controller for Digital Twin Visualization
// Provides interactive robot control for user engagement

using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Reference")]
    public RobotVisualization robotVisualization;

    [Header("Control Settings")]
    public bool allowManualControl = true;
    public float jointControlSpeed = 1.0f;
    public AnimationCurve jointSmoothCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

    [Header("Predefined Poses")]
    public bool enablePredefinedPoses = true;
    public float poseTransitionTime = 1.0f;

    [Header("Input Keys")]
    public KeyCode poseStandKey = KeyCode.Alpha1;
    public KeyCode poseSitKey = KeyCode.Alpha2;
    public KeyCode poseWaveKey = KeyCode.Alpha3;
    public KeyCode resetPoseKey = KeyCode.Alpha0;

    private float[] currentJointTargets;
    private float[] initialJointPositions;
    private bool isTransitioning = false;
    private float transitionStartTime;
    private float[] startJointPositions;

    void Start()
    {
        if (robotVisualization == null)
        {
            robotVisualization = GetComponent<RobotVisualization>();
        }

        if (robotVisualization != null)
        {
            // Initialize joint arrays
            int jointCount = robotVisualization.joints.Count;
            currentJointTargets = new float[jointCount];
            initialJointPositions = new float[jointCount];

            // Store initial positions
            for (int i = 0; i < jointCount; i++)
            {
                initialJointPositions[i] = 0; // Starting position
                currentJointTargets[i] = 0;
            }
        }
    }

    void Update()
    {
        if (robotVisualization != null && allowManualControl)
        {
            HandleManualControl();
            HandlePredefinedPoses();
        }

        if (isTransitioning)
        {
            UpdatePoseTransition();
        }
    }

    void HandleManualControl()
    {
        // Manual joint control via keyboard (for demonstration)
        // In a real implementation, this would come from UI sliders or other input methods

        // Example: Control head joint with arrow keys
        if (Input.GetKey(KeyCode.UpArrow))
        {
            SetJointTarget("head_joint", 0.3f);
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            SetJointTarget("head_joint", -0.3f);
        }
        else
        {
            SetJointTarget("head_joint", 0);
        }

        // Example: Control left arm
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            SetJointTarget("left_shoulder_joint", 1.0f);
        }
        else if (Input.GetKey(KeyCode.RightArrow))
        {
            SetJointTarget("left_shoulder_joint", -1.0f);
        }

        // Apply current targets to robot
        robotVisualization.UpdateRobotJoints(currentJointTargets);
    }

    void HandlePredefinedPoses()
    {
        if (!enablePredefinedPoses) return;

        if (Input.GetKeyDown(poseStandKey))
        {
            SetPoseToStand();
        }
        else if (Input.GetKeyDown(poseSitKey))
        {
            SetPoseToSit();
        }
        else if (Input.GetKeyDown(poseWaveKey))
        {
            SetPoseToWave();
        }
        else if (Input.GetKeyDown(resetPoseKey))
        {
            ResetToInitialPose();
        }
    }

    // Set target for a specific joint
    void SetJointTarget(string jointName, float targetValue)
    {
        if (robotVisualization == null) return;

        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            if (robotVisualization.joints[i].jointName == jointName)
            {
                currentJointTargets[i] = Mathf.Lerp(currentJointTargets[i], targetValue, jointControlSpeed * Time.deltaTime);
                break;
            }
        }
    }

    // Set robot to standing pose
    public void SetPoseToStand()
    {
        if (robotVisualization == null) return;

        // Store current positions for transition
        startJointPositions = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            startJointPositions[i] = currentJointTargets[i];
        }

        // Define standing pose (all joints at neutral position)
        float[] standPose = new float[robotVisualization.joints.Count];
        for (int i = 0; i < standPose.Length; i++)
        {
            standPose[i] = 0; // Neutral position for all joints
        }

        StartPoseTransition(standPose);
    }

    // Set robot to sitting pose
    public void SetPoseToSit()
    {
        if (robotVisualization == null) return;

        // Store current positions for transition
        startJointPositions = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            startJointPositions[i] = currentJointTargets[i];
        }

        // Define sitting pose (knees bent, hips angled)
        float[] sitPose = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            JointInfo joint = robotVisualization.joints[i];

            switch (joint.jointName)
            {
                case "left_hip_joint":
                case "right_hip_joint":
                    sitPose[i] = 0.8f; // Hip flexion for sitting
                    break;
                case "left_knee_joint":
                case "right_knee_joint":
                    sitPose[i] = 1.2f; // Knee flexion for sitting
                    break;
                default:
                    sitPose[i] = 0; // Neutral for other joints
                    break;
            }
        }

        StartPoseTransition(sitPose);
    }

    // Set robot to waving pose
    public void SetPoseToWave()
    {
        if (robotVisualization == null) return;

        // Store current positions for transition
        startJointPositions = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            startJointPositions[i] = currentJointTargets[i];
        }

        // Define waving pose (right arm up and moving)
        float[] wavePose = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            JointInfo joint = robotVisualization.joints[i];

            switch (joint.jointName)
            {
                case "right_shoulder_joint":
                    wavePose[i] = 1.2f; // Raise right arm
                    break;
                case "right_elbow_joint":
                    wavePose[i] = 0.8f; // Bend elbow
                    break;
                default:
                    wavePose[i] = 0; // Neutral for other joints
                    break;
            }
        }

        StartPoseTransition(wavePose);
    }

    // Reset to initial pose
    public void ResetToInitialPose()
    {
        if (robotVisualization == null) return;

        // Store current positions for transition
        startJointPositions = new float[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            startJointPositions[i] = currentJointTargets[i];
        }

        StartPoseTransition(initialJointPositions);
    }

    // Start a smooth transition to a new pose
    void StartPoseTransition(float[] targetPose)
    {
        isTransitioning = true;
        transitionStartTime = Time.time;

        // Update current targets to start the transition
        for (int i = 0; i < currentJointTargets.Length && i < targetPose.Length; i++)
        {
            currentJointTargets[i] = targetPose[i];
        }
    }

    // Update the pose transition based on time
    void UpdatePoseTransition()
    {
        float elapsed = Time.time - transitionStartTime;
        float progress = Mathf.Clamp01(elapsed / poseTransitionTime);

        if (progress >= 1.0f)
        {
            isTransitioning = false;
        }

        // Apply smooth transition using the animation curve
        float smoothProgress = jointSmoothCurve.Evaluate(progress);

        if (robotVisualization != null)
        {
            for (int i = 0; i < robotVisualization.joints.Count && i < startJointPositions.Length; i++)
            {
                float startPosition = startJointPositions[i];
                float endPosition = currentJointTargets[i];

                float newPosition = Mathf.Lerp(startPosition, endPosition, smoothProgress);
                currentJointTargets[i] = newPosition;
            }

            // Update the robot visualization
            robotVisualization.UpdateRobotJoints(currentJointTargets);
        }
    }

    // Method to set joint positions directly (for use from other scripts)
    public void SetJointPositions(float[] positions)
    {
        if (robotVisualization == null || positions == null) return;

        for (int i = 0; i < Mathf.Min(currentJointTargets.Length, positions.Length); i++)
        {
            currentJointTargets[i] = positions[i];
        }

        robotVisualization.UpdateRobotJoints(currentJointTargets);
    }

    // Method to get current joint positions
    public float[] GetJointPositions()
    {
        if (currentJointTargets == null) return new float[0];
        float[] positions = new float[currentJointTargets.Length];
        currentJointTargets.CopyTo(positions, 0);
        return positions;
    }

    // Method to get joint names
    public string[] GetJointNames()
    {
        if (robotVisualization == null) return new string[0];

        string[] names = new string[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            names[i] = robotVisualization.joints[i].jointName;
        }
        return names;
    }

    // Method to get joint limits
    public (float min, float max)[] GetJointLimits()
    {
        if (robotVisualization == null) return new (float min, float max)[0];

        var limits = new (float min, float max)[robotVisualization.joints.Count];
        for (int i = 0; i < robotVisualization.joints.Count; i++)
        {
            limits[i] = (robotVisualization.joints[i].minLimit, robotVisualization.joints[i].maxLimit);
        }
        return limits;
    }
}