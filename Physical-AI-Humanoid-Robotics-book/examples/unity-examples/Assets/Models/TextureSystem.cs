// Texture System for Digital Twin Visualization
// Handles material properties and texturing for realistic appearance

using UnityEngine;

public class TextureSystem : MonoBehaviour
{
    [Header("Material Properties")]
    public Material[] robotMaterials;
    public Material[] environmentMaterials;
    public Material[] sensorMaterials;

    [Header("Texture Settings")]
    public bool useNormalMaps = true;
    public bool useSpecularMaps = true;
    public bool useEmissionMaps = false;
    [Range(0.1f, 2.0f)] public float textureScale = 1.0f;

    [Header("Performance Settings")]
    public bool enableAnisotropicFiltering = true;
    public bool enableMipMapBias = true;
    [Range(0, 16)] public int anisotropicLevel = 4;

    void Start()
    {
        ApplyTextureSettings();
    }

    void ApplyTextureSettings()
    {
        // Apply settings to robot materials
        if (robotMaterials != null)
        {
            foreach (Material mat in robotMaterials)
            {
                if (mat != null)
                {
                    ConfigureMaterial(mat);
                }
            }
        }

        // Apply settings to environment materials
        if (environmentMaterials != null)
        {
            foreach (Material mat in environmentMaterials)
            {
                if (mat != null)
                {
                    ConfigureMaterial(mat);
                }
            }
        }

        // Apply settings to sensor materials
        if (sensorMaterials != null)
        {
            foreach (Material mat in sensorMaterials)
            {
                if (mat != null)
                {
                    ConfigureMaterial(mat);
                }
            }
        }
    }

    void ConfigureMaterial(Material material)
    {
        // Set texture scale
        material.mainTextureScale = new Vector3(textureScale, textureScale, textureScale);

        // Configure texture properties
        if (material.mainTexture != null)
        {
            Texture2D texture = material.mainTexture as Texture2D;
            if (texture != null)
            {
                // Apply anisotropic filtering
                if (enableAnisotropicFiltering)
                {
                    texture.anisoLevel = anisotropicLevel;
                }

                // Apply mipmap bias if enabled
                if (enableMipMapBias)
                {
                    texture.mipMapBias = 0.1f; // Small bias for better quality
                }
            }
        }

        // Configure material properties for performance
        material.enableInstancing = true;
    }

    // Method to switch to low quality textures for performance
    public void SetLowQualityTextures()
    {
        textureScale = 0.5f;
        anisotropicLevel = 1; // Disable anisotropic filtering
        enableAnisotropicFiltering = false;

        ApplyTextureSettings();
    }

    // Method to switch to high quality textures
    public void SetHighQualityTextures()
    {
        textureScale = 1.0f;
        anisotropicLevel = 4;
        enableAnisotropicFiltering = true;

        ApplyTextureSettings();
    }

    // Method to update materials based on performance requirements
    public void OptimizeForPerformance()
    {
        // Reduce texture quality to maintain performance targets
        SetLowQualityTextures();
    }

    // Method to restore quality when performance allows
    public void RestoreQuality()
    {
        SetHighQualityTextures();
    }
}