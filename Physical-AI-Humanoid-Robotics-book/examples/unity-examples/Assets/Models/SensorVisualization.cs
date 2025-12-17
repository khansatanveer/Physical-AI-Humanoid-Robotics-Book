// Sensor Visualization for Digital Twin
// Visualizes sensor data received from ROS/Gazebo in Unity

using UnityEngine;
using System.Collections.Generic;

public class SensorVisualization : MonoBehaviour
{
    [Header("Sensor Visualization Settings")]
    public GameObject lidarPointPrefab;
    public GameObject depthPointPrefab;
    public LineRenderer imuOrientationIndicator;
    public GameObject obstacleMarkerPrefab;

    [Header("LiDAR Visualization")]
    public float maxLidarRange = 30.0f;
    public int lidarResolution = 360;
    public float lidarPointSize = 0.05f;
    public Color lidarColor = Color.red;

    [Header("Depth Camera Visualization")]
    public float depthPointSize = 0.02f;
    public Color depthColor = Color.blue;

    [Header("IMU Visualization")]
    public Color imuColor = Color.green;

    [Header("Performance Settings")]
    public int maxVisualizationPoints = 1000;
    public bool enableLidarVisualization = true;
    public bool enableDepthVisualization = true;
    public bool enableImuVisualization = true;

    private List<GameObject> lidarPoints = new List<GameObject>();
    private List<GameObject> depthPoints = new List<GameObject>();
    private List<GameObject> obstacleMarkers = new List<GameObject>();

    private bool initialized = false;

    void Start()
    {
        InitializeSensorVisualization();
    }

    void InitializeSensorVisualization()
    {
        if (initialized) return;

        // Create lidar points for visualization
        if (lidarPointPrefab == null)
        {
            // Create a default sphere prefab if none is provided
            GameObject defaultLidarPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            defaultLidarPoint.name = "LidarPoint";
            defaultLidarPoint.GetComponent<Renderer>().material.color = lidarColor;
            defaultLidarPoint.transform.localScale = Vector3.one * lidarPointSize;
            defaultLidarPoint.SetActive(false);
            lidarPointPrefab = defaultLidarPoint;
        }

        // Create depth points for visualization
        if (depthPointPrefab == null)
        {
            // Create a default sphere prefab if none is provided
            GameObject defaultDepthPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            defaultDepthPoint.name = "DepthPoint";
            defaultDepthPoint.GetComponent<Renderer>().material.color = depthColor;
            defaultDepthPoint.transform.localScale = Vector3.one * depthPointSize;
            defaultDepthPoint.SetActive(false);
            depthPointPrefab = defaultDepthPoint;
        }

        // Create obstacle markers
        if (obstacleMarkerPrefab == null)
        {
            // Create a default cylinder prefab if none is provided
            GameObject defaultObstacleMarker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            defaultObstacleMarker.name = "ObstacleMarker";
            defaultObstacleMarker.GetComponent<Renderer>().material.color = Color.yellow;
            defaultObstacleMarker.transform.localScale = new Vector3(0.1f, 0.5f, 0.1f);
            defaultObstacleMarker.SetActive(false);
            obstacleMarkerPrefab = defaultObstacleMarker;
        }

        // Initialize lidar points
        for (int i = 0; i < lidarResolution; i++)
        {
            GameObject point = Instantiate(lidarPointPrefab, transform);
            point.SetActive(false);
            lidarPoints.Add(point);
        }

        initialized = true;
    }

    /// <summary>
    /// Update LiDAR visualization with new scan data
    /// </summary>
    /// <param name="ranges">Array of distance measurements</param>
    /// <param name="angleMin">Minimum angle of the scan</param>
    /// <param name="angleMax">Maximum angle of the scan</param>
    public void UpdateLidarVisualization(float[] ranges, float angleMin, float angleMax)
    {
        if (!enableLidarVisualization || !initialized) return;

        InitializeSensorVisualization();

        float angleIncrement = (angleMax - angleMin) / ranges.Length;

        for (int i = 0; i < Mathf.Min(ranges.Length, lidarPoints.Count); i++)
        {
            float range = ranges[i];
            float angle = angleMin + i * angleIncrement;

            if (range >= 0.1f && range <= maxLidarRange)
            {
                // Calculate position in local coordinates (relative to sensor)
                float x = range * Mathf.Cos(angle);
                float y = 0; // Assuming 2D LiDAR
                float z = range * Mathf.Sin(angle);

                Vector3 localPos = new Vector3(x, y, z);
                Vector3 worldPos = transform.TransformPoint(localPos);

                lidarPoints[i].transform.position = worldPos;
                lidarPoints[i].SetActive(true);
            }
            else
            {
                lidarPoints[i].SetActive(false);
            }
        }

        // Hide remaining points if ranges array is smaller than lidarPoints
        for (int i = ranges.Length; i < lidarPoints.Count; i++)
        {
            lidarPoints[i].SetActive(false);
        }
    }

    /// <summary>
    /// Update depth camera visualization with point cloud data
    /// </summary>
    /// <param name="depthPointsData">Array of 3D points from depth camera</param>
    public void UpdateDepthVisualization(Vector3[] depthPointsData)
    {
        if (!enableDepthVisualization || !initialized) return;

        // Clear existing depth points
        ClearDepthVisualization();

        // Create new depth points up to the maximum limit
        int pointsToCreate = Mathf.Min(depthPointsData.Length, maxVisualizationPoints);

        for (int i = 0; i < pointsToCreate; i++)
        {
            GameObject point = Instantiate(depthPointPrefab, transform);
            point.transform.position = transform.TransformPoint(depthPointsData[i]);
            point.SetActive(true);
            depthPoints.Add(point);
        }
    }

    /// <summary>
    /// Update IMU visualization with orientation data
    /// </summary>
    /// <param name="orientation">Quaternion representing orientation</param>
    public void UpdateImuVisualization(Quaternion orientation)
    {
        if (!enableImuVisualization || !initialized) return;

        if (imuOrientationIndicator != null)
        {
            // Set the color of the line renderer
            imuOrientationIndicator.startColor = imuColor;
            imuOrientationIndicator.endColor = imuColor;

            // Calculate start and end points for the orientation indicator
            Vector3 start = transform.position;
            Vector3 end = transform.position + transform.TransformDirection(orientation * Vector3.forward) * 0.5f;

            // Set the positions
            imuOrientationIndicator.SetPosition(0, start);
            imuOrientationIndicator.SetPosition(1, end);
        }
    }

    /// <summary>
    /// Update obstacle visualization based on sensor data
    /// </summary>
    /// <param name="obstaclePositions">List of obstacle positions in world coordinates</param>
    public void UpdateObstacleVisualization(List<Vector3> obstaclePositions)
    {
        // Clear existing obstacle markers
        ClearObstacleVisualization();

        // Create new obstacle markers
        foreach (Vector3 obstaclePos in obstaclePositions)
        {
            GameObject marker = Instantiate(obstacleMarkerPrefab, obstaclePos, Quaternion.identity);
            marker.SetActive(true);
            obstacleMarkers.Add(marker);
        }
    }

    /// <summary>
    /// Clear all LiDAR visualization points
    /// </summary>
    public void ClearLidarVisualization()
    {
        foreach (GameObject point in lidarPoints)
        {
            if (point != null)
                point.SetActive(false);
        }
    }

    /// <summary>
    /// Clear all depth visualization points
    /// </summary>
    public void ClearDepthVisualization()
    {
        foreach (GameObject point in depthPoints)
        {
            if (point != null)
                DestroyImmediate(point);
        }
        depthPoints.Clear();
    }

    /// <summary>
    /// Clear all obstacle visualization markers
    /// </summary>
    public void ClearObstacleVisualization()
    {
        foreach (GameObject marker in obstacleMarkers)
        {
            if (marker != null)
                DestroyImmediate(marker);
        }
        obstacleMarkers.Clear();
    }

    /// <summary>
    /// Clear all sensor visualizations
    /// </summary>
    public void ClearAllVisualizations()
    {
        ClearLidarVisualization();
        ClearDepthVisualization();
        ClearObstacleVisualization();
    }

    /// <summary>
    /// Process fused sensor data from ROS
    /// </summary>
    /// <param name="fusedData">Array containing [x, y, z, qx, qy, qz, qw, vx, vy, vz]</param>
    public void ProcessFusedSensorData(float[] fusedData)
    {
        if (fusedData.Length >= 7) // At least position and orientation
        {
            // Update position
            Vector3 position = new Vector3(fusedData[0], fusedData[1], fusedData[2]);
            transform.position = position;

            // Update orientation
            Quaternion orientation = new Quaternion(fusedData[3], fusedData[4], fusedData[5], fusedData[6]);
            transform.rotation = orientation;

            // Optionally use velocity data (indices 7, 8, 9) for additional visualization
        }
    }

    // Helper method to convert ROS coordinate system to Unity
    public Vector3 RosToUnity(Vector3 rosVector)
    {
        // ROS: X forward, Y left, Z up
        // Unity: X right, Y up, Z forward
        return new Vector3(rosVector.z, rosVector.x, rosVector.y);
    }

    // Helper method to convert ROS quaternion to Unity quaternion
    public Quaternion RosToUnity(Quaternion rosQuaternion)
    {
        // ROS quaternion to Unity quaternion conversion
        return new Quaternion(rosQuaternion.z, rosQuaternion.x, rosQuaternion.y, rosQuaternion.w);
    }

    void OnDestroy()
    {
        // Clean up visualization objects
        ClearAllVisualizations();
    }
}