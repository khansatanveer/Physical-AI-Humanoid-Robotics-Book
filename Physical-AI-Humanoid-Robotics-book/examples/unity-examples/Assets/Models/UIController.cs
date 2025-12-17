// UI Controller for Digital Twin Visualization
// Provides interactive UI elements for user engagement

using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class UIController : MonoBehaviour
{
    [Header("UI References")]
    public GameObject uiCanvas;
    public Text statusText;
    public Text fpsText;
    public Text robotStateText;
    public Slider[] jointSliders;
    public Button[] poseButtons;

    [Header("UI Panels")]
    public GameObject controlPanel;
    public GameObject infoPanel;
    public GameObject simulationControls;

    [Header("Robot Reference")]
    public RobotController robotController;
    public RobotVisualization robotVisualization;

    [Header("UI Settings")]
    public bool showUIByDefault = true;
    public KeyCode toggleUIKey = KeyCode.Tab;

    [Header("Performance Monitoring")]
    public PerformanceMonitor performanceMonitor;

    private bool isUIVisible = true;
    private float[] lastJointPositions;

    void Start()
    {
        InitializeUI();
        UpdateUIVisibility();
    }

    void Update()
    {
        UpdateUI();
        HandleUIInput();
    }

    void InitializeUI()
    {
        // Initialize robot controller if not set
        if (robotController == null)
        {
            robotController = FindObjectOfType<RobotController>();
        }

        if (robotVisualization == null)
        {
            robotVisualization = FindObjectOfType<RobotVisualization>();
        }

        if (performanceMonitor == null)
        {
            performanceMonitor = FindObjectOfType<PerformanceMonitor>();
        }

        // Create joint sliders if they don't exist
        if ((jointSliders == null || jointSliders.Length == 0) && robotController != null)
        {
            CreateJointSliders();
        }

        // Create pose buttons if they don't exist
        if ((poseButtons == null || poseButtons.Length == 0) && robotController != null)
        {
            CreatePoseButtons();
        }

        // Initialize with current robot state
        UpdateRobotStateDisplay();
    }

    void CreateJointSliders()
    {
        if (robotController == null) return;

        string[] jointNames = robotController.GetJointNames();
        (float min, float max)[] jointLimits = robotController.GetJointLimits();

        // Create a container for sliders
        GameObject sliderContainer = new GameObject("JointSliders");
        sliderContainer.transform.SetParent(uiCanvas.transform);
        RectTransform sliderContainerRect = sliderContainer.AddComponent<RectTransform>();
        sliderContainerRect.anchorMin = new Vector2(0.02f, 0.6f);
        sliderContainerRect.anchorMax = new Vector2(0.3f, 0.95f);
        sliderContainerRect.anchoredPosition = Vector2.zero;
        sliderContainerRect.sizeDelta = Vector2.zero;

        // Create vertical layout group
        VerticalLayoutGroup layout = sliderContainer.AddComponent<VerticalLayoutGroup>();
        layout.padding = new RectOffset(10, 10, 10, 10);
        layout.spacing = 5;
        layout.childAlignment = TextAnchor.UpperLeft;

        // Create sliders for each joint
        jointSliders = new Slider[jointNames.Length];
        for (int i = 0; i < jointNames.Length; i++)
        {
            // Create slider container
            GameObject sliderObj = new GameObject($"Slider_{jointNames[i]}");
            sliderObj.transform.SetParent(sliderContainer.transform);
            RectTransform sliderRect = sliderObj.AddComponent<RectTransform>();
            sliderRect.sizeDelta = new Vector2(0, 30);

            // Create label
            GameObject labelObj = new GameObject("Label");
            labelObj.transform.SetParent(sliderObj.transform);
            Text label = labelObj.AddComponent<Text>();
            label.text = jointNames[i];
            label.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            label.fontSize = 14;
            label.color = Color.white;
            RectTransform labelRect = labelObj.GetComponent<RectTransform>();
            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = new Vector2(0, 0);
            labelRect.offsetMax = new Vector2(-30, 0);

            // Create slider
            GameObject sliderComponent = new GameObject("Slider");
            sliderComponent.transform.SetParent(sliderObj.transform);
            Slider slider = sliderComponent.AddComponent<Slider>();
            slider.minValue = jointLimits[i].min;
            slider.maxValue = jointLimits[i].max;
            slider.value = 0; // Default position
            jointSliders[i] = slider;

            RectTransform sliderComponentRect = sliderComponent.GetComponent<RectTransform>();
            sliderComponentRect.anchorMin = new Vector2(1, 0);
            sliderComponentRect.anchorMax = new Vector2(1, 1);
            sliderComponentRect.pivot = new Vector2(1, 0.5f);
            sliderComponentRect.sizeDelta = new Vector2(150, 0);
            sliderComponentRect.anchoredPosition = new Vector2(-5, 0);

            // Add listener to update robot when slider changes
            int jointIndex = i; // Capture for closure
            slider.onValueChanged.AddListener((float value) =>
            {
                UpdateRobotJoint(jointIndex, value);
            });
        }
    }

    void CreatePoseButtons()
    {
        if (robotController == null) return;

        // Create a container for pose buttons
        GameObject buttonContainer = new GameObject("PoseButtons");
        buttonContainer.transform.SetParent(uiCanvas.transform);
        RectTransform buttonContainerRect = buttonContainer.AddComponent<RectTransform>();
        buttonContainerRect.anchorMin = new Vector2(0.7f, 0.8f);
        buttonContainerRect.anchorMax = new Vector2(0.95f, 0.95f);
        buttonContainerRect.anchoredPosition = Vector2.zero;
        buttonContainerRect.sizeDelta = Vector2.zero;

        // Create grid layout group
        GridLayoutGroup layout = buttonContainer.AddComponent<GridLayoutGroup>();
        layout.cellSize = new Vector2(100, 30);
        layout.spacing = new Vector2(10, 10);
        layout.startCorner = GridLayoutGroup.Corner.UpperLeft;
        layout.startAxis = GridLayoutGroup.Axis.Horizontal;
        layout.childAlignment = TextAnchor.MiddleCenter;

        // Create pose buttons
        poseButtons = new Button[4];

        // Stand button
        poseButtons[0] = CreateButton(buttonContainer.transform, "Stand", () => robotController.SetPoseToStand());
        // Sit button
        poseButtons[1] = CreateButton(buttonContainer.transform, "Sit", () => robotController.SetPoseToSit());
        // Wave button
        poseButtons[2] = CreateButton(buttonContainer.transform, "Wave", () => robotController.SetPoseToWave());
        // Reset button
        poseButtons[3] = CreateButton(buttonContainer.transform, "Reset", () => robotController.ResetToInitialPose());
    }

    Button CreateButton(Transform parent, string text, UnityEngine.Events.UnityAction onClick)
    {
        GameObject buttonObj = new GameObject($"Button_{text}");
        buttonObj.transform.SetParent(parent);
        Button button = buttonObj.AddComponent<Button>();

        // Add image component for button background
        Image buttonImage = buttonObj.AddComponent<Image>();
        buttonImage.color = new Color(0.2f, 0.4f, 0.8f, 0.8f);

        // Add text component
        GameObject textObj = new GameObject("Text");
        textObj.transform.SetParent(buttonObj.transform);
        Text buttonText = textObj.AddComponent<Text>();
        buttonText.text = text;
        buttonText.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        buttonText.fontSize = 14;
        buttonText.color = Color.white;
        buttonText.alignment = TextAnchor.MiddleCenter;

        // Set up button layout
        RectTransform textRect = textObj.GetComponent<RectTransform>();
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = new Vector2(5, 5);
        textRect.offsetMax = new Vector2(-5, -5);

        // Add listener
        button.onClick.AddListener(onClick);

        return button;
    }

    void UpdateUI()
    {
        // Update FPS display
        if (fpsText != null && performanceMonitor != null)
        {
            fpsText.text = $"FPS: {performanceMonitor.GetFps():F1}\nFrame Time: {performanceMonitor.GetAverageFrameTime() * 1000:F1}ms";
        }

        // Update robot state display
        UpdateRobotStateDisplay();

        // Update joint sliders if robot positions have changed
        if (robotController != null && jointSliders != null)
        {
            float[] currentPositions = robotController.GetJointPositions();

            if (lastJointPositions == null || lastJointPositions.Length != currentPositions.Length)
            {
                lastJointPositions = new float[currentPositions.Length];
            }

            bool positionsChanged = false;
            for (int i = 0; i < currentPositions.Length; i++)
            {
                if (Mathf.Abs(lastJointPositions[i] - currentPositions[i]) > 0.001f)
                {
                    positionsChanged = true;
                    break;
                }
            }

            if (positionsChanged)
            {
                for (int i = 0; i < Mathf.Min(jointSliders.Length, currentPositions.Length); i++)
                {
                    jointSliders[i].value = currentPositions[i];
                }
                System.Array.Copy(currentPositions, lastJointPositions, currentPositions.Length);
            }
        }
    }

    void UpdateRobotStateDisplay()
    {
        if (robotStateText != null && robotVisualization != null)
        {
            string jointInfo = "Robot Joints:\n";
            for (int i = 0; i < Mathf.Min(5, robotVisualization.joints.Count); i++) // Show first 5 joints
            {
                JointInfo joint = robotVisualization.joints[i];
                jointInfo += $"{joint.jointName}: {joint.position:F3}\n";
            }

            if (robotVisualization.joints.Count > 5)
            {
                jointInfo += "... and " + (robotVisualization.joints.Count - 5) + " more";
            }

            robotStateText.text = jointInfo;
        }
    }

    void HandleUIInput()
    {
        // Toggle UI visibility
        if (Input.GetKeyDown(toggleUIKey))
        {
            isUIVisible = !isUIVisible;
            UpdateUIVisibility();
        }
    }

    void UpdateUIVisibility()
    {
        if (uiCanvas != null)
        {
            uiCanvas.SetActive(isUIVisible);
        }

        if (controlPanel != null)
        {
            controlPanel.SetActive(isUIVisible);
        }

        if (infoPanel != null)
        {
            infoPanel.SetActive(isUIVisible);
        }

        if (simulationControls != null)
        {
            simulationControls.SetActive(isUIVisible);
        }
    }

    // Method to update a specific robot joint from UI
    void UpdateRobotJoint(int jointIndex, float value)
    {
        if (robotController != null)
        {
            float[] positions = robotController.GetJointPositions();
            if (jointIndex < positions.Length)
            {
                positions[jointIndex] = value;
                robotController.SetJointPositions(positions);
            }
        }
    }

    // Method to show/hide UI
    public void SetUIVisibility(bool visible)
    {
        isUIVisible = visible;
        UpdateUIVisibility();
    }

    // Method to toggle UI visibility
    public void ToggleUIVisibility()
    {
        isUIVisible = !isUIVisible;
        UpdateUIVisibility();
    }

    // Method to get current UI visibility state
    public bool GetUIVisibility()
    {
        return isUIVisible;
    }

    // Method to update status text
    public void UpdateStatusText(string status)
    {
        if (statusText != null)
        {
            statusText.text = status;
        }
    }
}