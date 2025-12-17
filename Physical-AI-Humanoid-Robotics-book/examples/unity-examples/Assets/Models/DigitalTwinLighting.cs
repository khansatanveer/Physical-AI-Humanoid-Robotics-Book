// Lighting System for Digital Twin Visualization
// Implements realistic lighting to match Gazebo environment

using UnityEngine;

public class DigitalTwinLighting : MonoBehaviour
{
    [Header("Light Configuration")]
    public Light mainLight; // Directional light for primary illumination
    public Light fillLight; // Secondary light for fill illumination
    public Light rimLight;  // Back light for rim highlights

    [Header("Light Properties")]
    [Range(0, 2)] public float mainLightIntensity = 1.0f;
    [Range(0, 1)] public float fillLightIntensity = 0.3f;
    [Range(0, 1)] public float rimLightIntensity = 0.2f;

    [Header("Color Temperature")]
    public Color mainLightColor = Color.white;
    public Color fillLightColor = new Color(0.9f, 0.95f, 1.0f); // Slightly blue
    public Color rimLightColor = new Color(0.8f, 0.85f, 1.0f);  // Cool white

    void Start()
    {
        SetupLighting();
    }

    void SetupLighting()
    {
        // Find or create main directional light
        if (mainLight == null)
        {
            GameObject mainLightObj = GameObject.Find("Main Directional Light");
            if (mainLightObj == null)
            {
                mainLightObj = new GameObject("Main Directional Light");
                mainLightObj.AddComponent<Light>();
                mainLightObj.transform.position = new Vector3(0, 10, 0);
                mainLightObj.transform.rotation = Quaternion.Euler(50, -30, 0);
            }
            mainLight = mainLightObj.GetComponent<Light>();
        }

        // Configure main light
        mainLight.type = LightType.Directional;
        mainLight.color = mainLightColor;
        mainLight.intensity = mainLightIntensity;
        mainLight.shadows = LightShadows.Soft;
        mainLight.shadowStrength = 0.8f;
        mainLight.shadowResolution = ShadowResolution.Medium;

        // Find or create fill light
        if (fillLight == null)
        {
            GameObject fillLightObj = GameObject.Find("Fill Light");
            if (fillLightObj == null)
            {
                fillLightObj = new GameObject("Fill Light");
                fillLightObj.AddComponent<Light>();
                fillLightObj.transform.position = new Vector3(5, 5, -5);
                fillLightObj.transform.LookAt(Vector3.zero);
            }
            fillLight = fillLightObj.GetComponent<Light>();
        }

        // Configure fill light
        fillLight.type = LightType.Directional;
        fillLight.color = fillLightColor;
        fillLight.intensity = fillLightIntensity;
        fillLight.shadows = LightShadows.None;

        // Find or create rim light
        if (rimLight == null)
        {
            GameObject rimLightObj = GameObject.Find("Rim Light");
            if (rimLightObj == null)
            {
                rimLightObj = new GameObject("Rim Light");
                rimLightObj.AddComponent<Light>();
                rimLightObj.transform.position = new Vector3(-5, 5, 5);
                rimLightObj.transform.LookAt(Vector3.zero);
            }
            rimLight = rimLightObj.GetComponent<Light>();
        }

        // Configure rim light
        rimLight.type = LightType.Directional;
        rimLight.color = rimLightColor;
        rimLight.intensity = rimLightIntensity;
        rimLight.shadows = LightShadows.None;
    }

    // Method to adjust lighting based on time of day or simulation state
    public void SetLightingPreset(LightingPreset preset)
    {
        switch (preset)
        {
            case LightingPreset.Default:
                mainLightIntensity = 1.0f;
                fillLightIntensity = 0.3f;
                rimLightIntensity = 0.2f;
                break;
            case LightingPreset.Bright:
                mainLightIntensity = 1.5f;
                fillLightIntensity = 0.5f;
                rimLightIntensity = 0.3f;
                break;
            case LightingPreset.Dim:
                mainLightIntensity = 0.7f;
                fillLightIntensity = 0.2f;
                rimLightIntensity = 0.1f;
                break;
        }
        SetupLighting();
    }
}

public enum LightingPreset
{
    Default,
    Bright,
    Dim
}