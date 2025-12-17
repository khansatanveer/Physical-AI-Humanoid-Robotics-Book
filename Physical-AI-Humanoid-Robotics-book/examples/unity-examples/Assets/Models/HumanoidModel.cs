// Basic Humanoid Robot Model for Unity
// This model corresponds to the basic_humanoid.urdf from Gazebo

// Robot hierarchy structure
public class BasicHumanoidModel
{
    // Root of the robot
    public GameObject root;

    // Body parts
    public GameObject head;
    public GameObject torso;
    public GameObject leftUpperArm;
    public GameObject leftLowerArm;
    public GameObject rightUpperArm;
    public GameObject rightLowerArm;
    public GameObject leftUpperLeg;
    public GameObject leftLowerLeg;
    public GameObject rightUpperLeg;
    public GameObject rightLowerLeg;

    // Joint positions (relative to parent)
    public Vector3 headPosition = new Vector3(0, 0.8f, 0);
    public Vector3 leftUpperArmPosition = new Vector3(-0.3f, 0.6f, 0);
    public Vector3 rightUpperArmPosition = new Vector3(0.3f, 0.6f, 0);
    public Vector3 leftUpperLegPosition = new Vector3(-0.15f, -0.5f, 0);
    public Vector3 rightUpperLegPosition = new Vector3(0.15f, -0.5f, 0);

    // Constructor to build the model
    public BasicHumanoidModel()
    {
        // Create root
        root = CreateBodyPart("HumanoidRoot", Vector3.zero, Vector3.one);

        // Create torso (main body)
        torso = CreateBodyPart("Torso", Vector3.zero, new Vector3(0.4f, 0.8f, 0.3f));
        torso.transform.SetParent(root.transform);

        // Create head
        head = CreateBodyPart("Head", headPosition, new Vector3(0.25f, 0.25f, 0.25f));
        head.transform.SetParent(torso.transform);

        // Create left arm
        leftUpperArm = CreateBodyPart("LeftUpperArm", leftUpperArmPosition, new Vector3(0.15f, 0.4f, 0.15f));
        leftUpperArm.transform.SetParent(torso.transform);

        leftLowerArm = CreateBodyPart("LeftLowerArm", new Vector3(0, -0.4f, 0), new Vector3(0.12f, 0.35f, 0.12f));
        leftLowerArm.transform.SetParent(leftUpperArm.transform);

        // Create right arm
        rightUpperArm = CreateBodyPart("RightUpperArm", rightUpperArmPosition, new Vector3(0.15f, 0.4f, 0.15f));
        rightUpperArm.transform.SetParent(torso.transform);

        rightLowerArm = CreateBodyPart("RightLowerArm", new Vector3(0, -0.4f, 0), new Vector3(0.12f, 0.35f, 0.12f));
        rightLowerArm.transform.SetParent(rightUpperArm.transform);

        // Create left leg
        leftUpperLeg = CreateBodyPart("LeftUpperLeg", leftUpperLegPosition, new Vector3(0.18f, 0.5f, 0.18f));
        leftUpperLeg.transform.SetParent(torso.transform);

        leftLowerLeg = CreateBodyPart("LeftLowerLeg", new Vector3(0, -0.5f, 0), new Vector3(0.15f, 0.45f, 0.15f));
        leftLowerLeg.transform.SetParent(leftUpperLeg.transform);

        // Create right leg
        rightUpperLeg = CreateBodyPart("RightUpperLeg", rightUpperLegPosition, new Vector3(0.18f, 0.5f, 0.18f));
        rightUpperLeg.transform.SetParent(torso.transform);

        rightLowerLeg = CreateBodyPart("RightLowerLeg", new Vector3(0, -0.5f, 0), new Vector3(0.15f, 0.45f, 0.15f));
        rightLowerLeg.transform.SetParent(rightUpperLeg.transform);
    }

    // Helper method to create a body part
    private GameObject CreateBodyPart(string name, Vector3 position, Vector3 scale)
    {
        GameObject part = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        part.name = name;
        part.transform.position = position;
        part.transform.localScale = scale;

        // Apply material for humanoid appearance
        Material material = CreateRobotMaterial();
        part.GetComponent<Renderer>().material = material;

        return part;
    }

    // Helper method to create a material for the robot
    private Material CreateRobotMaterial()
    {
        Material material = new Material(Shader.Find("Standard"));
        material.color = Color.gray; // Metallic gray for robot appearance
        material.SetFloat("_Metallic", 0.7f);
        material.SetFloat("_Smoothness", 0.5f);
        return material;
    }
}