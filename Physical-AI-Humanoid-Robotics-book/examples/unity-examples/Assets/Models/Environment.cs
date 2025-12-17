// Basic Environment Models for Unity Digital Twin
// These models correspond to the physics_world.sdf from Gazebo

using UnityEngine;

public class BasicEnvironment
{
    public GameObject groundPlane;
    public GameObject obstacle1;
    public GameObject obstacle2;
    public GameObject ramp;
    public GameObject platform;

    // Constructor to build the environment
    public BasicEnvironment()
    {
        CreateGroundPlane();
        CreateObstacles();
        CreateRamp();
        CreatePlatform();
    }

    // Create ground plane
    private void CreateGroundPlane()
    {
        groundPlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
        groundPlane.name = "GroundPlane";
        groundPlane.transform.position = Vector3.zero;
        groundPlane.transform.localScale = new Vector3(5, 1, 5); // Scale the plane to match Gazebo world

        // Apply ground material
        Material groundMaterial = CreateGroundMaterial();
        groundPlane.GetComponent<Renderer>().material = groundMaterial;
    }

    // Create obstacles
    private void CreateObstacles()
    {
        // Obstacle 1 (box)
        obstacle1 = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle1.name = "Obstacle1";
        obstacle1.transform.position = new Vector3(2, 0.5f, 2);
        obstacle1.transform.localScale = new Vector3(1, 1, 1);

        Material obstacleMaterial = CreateObstacleMaterial();
        obstacle1.GetComponent<Renderer>().material = obstacleMaterial;

        // Obstacle 2 (cylinder)
        obstacle2 = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        obstacle2.name = "Obstacle2";
        obstacle2.transform.position = new Vector3(-2, 0.75f, -2);
        obstacle2.transform.localScale = new Vector3(0.8f, 0.75f, 0.8f); // Unity cylinder is 2 units tall by default
        obstacle2.transform.rotation = Quaternion.Euler(90, 0, 0); // Rotate to lay flat

        obstacle2.GetComponent<Renderer>().material = obstacleMaterial;
    }

    // Create ramp
    private void CreateRamp()
    {
        ramp = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        ramp.name = "Ramp";
        ramp.transform.position = new Vector3(3, 0.5f, -1);
        ramp.transform.localScale = new Vector3(0.5f, 2, 0.5f); // Stretch in X to make it a ramp
        ramp.transform.rotation = Quaternion.Euler(0, 0, 30); // Angle the ramp

        Material rampMaterial = CreateRampMaterial();
        ramp.GetComponent<Renderer>().material = rampMaterial;
    }

    // Create platform
    private void CreatePlatform()
    {
        platform = GameObject.CreatePrimitive(PrimitiveType.Cube);
        platform.name = "Platform";
        platform.transform.position = new Vector3(-3, 1, 1);
        platform.transform.localScale = new Vector3(2, 0.2f, 2);

        Material platformMaterial = CreatePlatformMaterial();
        platform.GetComponent<Renderer>().material = platformMaterial;
    }

    // Helper method to create ground material
    private Material CreateGroundMaterial()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = new Color(0.3f, 0.5f, 0.3f); // Green ground to match Gazebo
        material.SetFloat("_Metallic", 0.0f);
        material.SetFloat("_Smoothness", 0.2f);
        return material;
    }

    // Helper method to create obstacle material
    private Material CreateObstacleMaterial()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = new Color(0.6f, 0.4f, 0.2f); // Brown obstacle
        material.SetFloat("_Metallic", 0.1f);
        material.SetFloat("_Smoothness", 0.3f);
        return material;
    }

    // Helper method to create ramp material
    private Material CreateRampMaterial()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = new Color(0.8f, 0.6f, 0.2f); // Yellow ramp
        material.SetFloat("_Metallic", 0.1f);
        material.SetFloat("_Smoothness", 0.4f);
        return material;
    }

    // Helper method to create platform material
    private Material CreatePlatformMaterial()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = new Color(0.4f, 0.4f, 0.8f); // Blue platform
        material.SetFloat("_Metallic", 0.2f);
        material.SetFloat("_Smoothness", 0.5f);
        return material;
    }
}