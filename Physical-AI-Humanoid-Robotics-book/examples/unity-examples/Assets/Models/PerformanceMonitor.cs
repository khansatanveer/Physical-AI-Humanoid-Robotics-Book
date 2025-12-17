// Performance Monitor for Digital Twin Visualization
// Tracks rendering performance to ensure <100ms response time

using UnityEngine;

public class PerformanceMonitor : MonoBehaviour
{
    [Header("Performance Settings")]
    public int frameHistorySize = 60; // Number of frames to track for average
    public int historyBufferSize = 1000; // Size of buffer for percentile calculations
    public float performanceWarningThreshold = 0.1f; // 100ms warning threshold
    public bool showPerformanceInEditor = true;

    [Header("Performance Data")]
    [ReadOnly] public float currentFrameTime = 0f;
    [ReadOnly] public float averageFrameTime = 0f;
    [ReadOnly] public float maxFrameTime = 0f;
    [ReadOnly] public int currentFPS = 0;
    [ReadOnly] public float p95FrameTime = 0f;

    private float[] frameTimes;
    private float[] frameTimeHistory;
    private int frameIndex = 0;
    private int historyIndex = 0;

    void Start()
    {
        InitializeBuffers();
    }

    void Update()
    {
        RecordFrameTime();
        UpdatePerformanceMetrics();
        CheckPerformanceThresholds();
    }

    void InitializeBuffers()
    {
        frameTimes = new float[frameHistorySize];
        frameTimeHistory = new float[historyBufferSize];
    }

    void RecordFrameTime()
    {
        // Record current frame time
        currentFrameTime = Time.unscaledDeltaTime;

        // Store in rolling frame history
        frameTimes[frameIndex] = currentFrameTime;
        frameIndex = (frameIndex + 1) % frameTimes.Length;

        // Store in larger history for percentile calculations
        frameTimeHistory[historyIndex] = currentFrameTime;
        historyIndex = (historyIndex + 1) % frameTimeHistory.Length;
    }

    void UpdatePerformanceMetrics()
    {
        // Calculate average frame time
        float sum = 0;
        for (int i = 0; i < frameTimes.Length; i++)
        {
            sum += frameTimes[i];
        }
        averageFrameTime = sum / frameTimes.Length;

        // Calculate max frame time in recent history
        maxFrameTime = 0;
        for (int i = 0; i < frameTimes.Length; i++)
        {
            if (frameTimes[i] > maxFrameTime)
                maxFrameTime = frameTimes[i];
        }

        // Calculate current FPS
        if (averageFrameTime > 0)
        {
            currentFPS = Mathf.RoundToInt(1.0f / averageFrameTime);
        }
        else
        {
            currentFPS = 0;
        }

        // Calculate 95th percentile frame time
        p95FrameTime = GetPercentileFrameTime(0.95f);
    }

    void CheckPerformanceThresholds()
    {
        // Check if we're meeting the <100ms response requirement
        if (currentFrameTime > performanceWarningThreshold)
        {
            Debug.LogWarning($"Frame time exceeded {performanceWarningThreshold * 1000:F0}ms: {currentFrameTime * 1000:F1}ms");
        }

        // Additional performance checks can be added here
    }

    public float GetAverageFrameTime()
    {
        return averageFrameTime;
    }

    public float GetMaxFrameTime()
    {
        return maxFrameTime;
    }

    public float GetPercentileFrameTime(float percentile)
    {
        if (frameTimeHistory.Length == 0) return 0;

        // Create a copy of the history and sort it
        float[] sortedHistory = new float[frameTimeHistory.Length];
        System.Array.Copy(frameTimeHistory, sortedHistory, frameTimeHistory.Length);
        System.Array.Sort(sortedHistory);

        // Calculate the index for the requested percentile
        int index = (int)(percentile * sortedHistory.Length);
        return sortedHistory[Mathf.Clamp(index, 0, sortedHistory.Length - 1)];
    }

    public int GetFps()
    {
        return currentFPS;
    }

    public bool IsMeetingPerformanceTarget()
    {
        // Check if 95th percentile frame time is under 100ms
        return p95FrameTime < 0.1f; // 100ms
    }

    public float GetP95FrameTime()
    {
        return p95FrameTime;
    }

    // Method to reset performance statistics
    public void ResetStatistics()
    {
        for (int i = 0; i < frameTimes.Length; i++)
        {
            frameTimes[i] = 0;
        }

        for (int i = 0; i < frameTimeHistory.Length; i++)
        {
            frameTimeHistory[i] = 0;
        }

        frameIndex = 0;
        historyIndex = 0;
    }

    // Method to get performance summary as a string
    public string GetPerformanceSummary()
    {
        return $"FPS: {currentFPS}\n" +
               $"Avg Frame Time: {averageFrameTime * 1000:F1}ms\n" +
               $"Max Frame Time: {maxFrameTime * 1000:F1}ms\n" +
               $"P95 Frame Time: {p95FrameTime * 1000:F1}ms\n" +
               $"Target < 100ms: {(p95FrameTime < 0.1f ? "PASS" : "FAIL")}";
    }

#if UNITY_EDITOR
    void OnValidate()
    {
        if (frameHistorySize <= 0) frameHistorySize = 1;
        if (historyBufferSize <= 0) historyBufferSize = 1;
        if (performanceWarningThreshold <= 0) performanceWarningThreshold = 0.1f;
    }

    void OnGUI()
    {
        if (showPerformanceInEditor)
        {
            GUILayout.BeginArea(new Rect(10, 10, 250, 150));
            GUILayout.Label("Performance Monitor");
            GUILayout.Label($"FPS: {currentFPS}");
            GUILayout.Label($"Avg Frame: {averageFrameTime * 1000:F1}ms");
            GUILayout.Label($"P95 Frame: {p95FrameTime * 1000:F1}ms");
            GUILayout.Label($"Target: {(p95FrameTime < 0.1f ? "PASS" : "FAIL")}");
            GUILayout.EndArea();
        }
    }
#endif
}

// Custom attribute to make fields read-only in inspector
public class ReadOnlyAttribute : PropertyAttribute {}