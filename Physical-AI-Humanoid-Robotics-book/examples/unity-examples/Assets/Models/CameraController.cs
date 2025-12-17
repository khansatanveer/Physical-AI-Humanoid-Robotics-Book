// Camera Controller for Digital Twin Visualization
// Provides interactive camera controls for user navigation

using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5.0f;
    public float fastMoveMultiplier = 3.0f;
    public float lookSensitivity = 2.0f;

    [Header("Rotation Settings")]
    public float minVerticalAngle = -90.0f;
    public float maxVerticalAngle = 90.0f;

    [Header("Zoom Settings")]
    public float zoomSpeed = 10.0f;
    public float minZoomDistance = 2.0f;
    public float maxZoomDistance = 50.0f;

    [Header("Target Following")]
    public Transform targetToFollow;
    public Vector3 offset = new Vector3(-5, 3, -5);
    public float smoothSpeed = 0.125f;

    [Header("Input Keys")]
    public KeyCode fastMoveKey = KeyCode.LeftShift;
    public KeyCode resetCameraKey = KeyCode.R;

    private float rotationX = 0.0f;
    private float rotationY = 0.0f;
    private Vector3 desiredPosition;

    private bool isMouseLocked = false;
    private bool isOrbiting = false;
    private Vector3 lastMousePosition;

    void Start()
    {
        desiredPosition = transform.position;
        Cursor.visible = !isMouseLocked;
    }

    void Update()
    {
        HandleMouseInput();
        HandleKeyboardInput();
        HandleTargetFollowing();

        UpdateCameraPosition();
    }

    void HandleMouseInput()
    {
        // Toggle mouse lock with middle mouse button
        if (Input.GetMouseButtonDown(2)) // Middle mouse button
        {
            isMouseLocked = !isMouseLocked;
            Cursor.visible = !isMouseLocked;
            lastMousePosition = Input.mousePosition;
        }

        if (isMouseLocked)
        {
            // Rotation with right mouse button
            if (Input.GetMouseButton(1)) // Right mouse button
            {
                rotationX += Input.GetAxis("Mouse X") * lookSensitivity;
                rotationY -= Input.GetAxis("Mouse Y") * lookSensitivity;
                rotationY = Mathf.Clamp(rotationY, minVerticalAngle, maxVerticalAngle);

                transform.localRotation = Quaternion.AngleAxis(rotationX, Vector3.up);
                transform.localRotation *= Quaternion.AngleAxis(rotationY, Vector3.left);
            }
            // Panning with left mouse button
            else if (Input.GetMouseButton(0)) // Left mouse button
            {
                Vector3 mouseDelta = (Input.mousePosition - lastMousePosition) * 0.01f;
                Vector3 right = transform.right * -mouseDelta.x * moveSpeed * 0.1f;
                Vector3 up = transform.up * mouseDelta.y * moveSpeed * 0.1f;

                transform.position += right + up;
            }

            lastMousePosition = Input.mousePosition;
        }
    }

    void HandleKeyboardInput()
    {
        // Movement with WASD keys
        float currentMoveSpeed = moveSpeed;
        if (Input.GetKey(fastMoveKey))
        {
            currentMoveSpeed *= fastMoveMultiplier;
        }

        float moveX = Input.GetAxis("Horizontal") * currentMoveSpeed * Time.deltaTime;
        float moveZ = Input.GetAxis("Vertical") * currentMoveSpeed * Time.deltaTime;

        // Move in the direction the camera is facing (ignore Y)
        Vector3 forward = transform.TransformDirection(Vector3.forward);
        forward.y = 0;
        forward.Normalize();
        Vector3 right = transform.TransformDirection(Vector3.right);

        transform.position += (forward * moveZ + right * moveX);

        // Vertical movement with Q and E
        if (Input.GetKey(KeyCode.Q))
        {
            transform.position += Vector3.down * currentMoveSpeed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.E))
        {
            transform.position += Vector3.up * currentMoveSpeed * Time.deltaTime;
        }

        // Reset camera position
        if (Input.GetKeyDown(resetCameraKey))
        {
            ResetCamera();
        }

        // Zoom with mouse wheel
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0)
        {
            Vector3 zoomDirection = (targetToFollow != null) ?
                (targetToFollow.position - transform.position).normalized :
                transform.forward;

            Vector3 newPosition = transform.position + zoomDirection * scroll * zoomSpeed * 100.0f;

            if (targetToFollow != null)
            {
                float distance = Vector3.Distance(newPosition, targetToFollow.position);
                if (distance >= minZoomDistance && distance <= maxZoomDistance)
                {
                    transform.position = newPosition;
                }
            }
            else
            {
                transform.position = newPosition;
            }
        }
    }

    void HandleTargetFollowing()
    {
        if (targetToFollow != null)
        {
            // Calculate desired position based on target and offset
            desiredPosition = targetToFollow.position + offset;

            // Smoothly move towards the desired position
            transform.position = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);
            transform.LookAt(targetToFollow);
        }
    }

    void UpdateCameraPosition()
    {
        // Apply any additional camera position adjustments here
    }

    void ResetCamera()
    {
        if (targetToFollow != null)
        {
            transform.position = targetToFollow.position + offset;
            transform.LookAt(targetToFollow);
            rotationX = 0;
            rotationY = 0;
        }
        else
        {
            // Default reset position
            transform.position = new Vector3(0, 5, -10);
            transform.rotation = Quaternion.Euler(15, 0, 0);
        }
    }

    // Method to set a new target to follow
    public void SetTargetToFollow(Transform newTarget)
    {
        targetToFollow = newTarget;
    }

    // Method to get current camera state for saving/loading
    public CameraState GetCameraState()
    {
        return new CameraState
        {
            position = transform.position,
            rotation = transform.rotation,
            target = targetToFollow
        };
    }

    // Method to set camera state from saved data
    public void SetCameraState(CameraState state)
    {
        transform.position = state.position;
        transform.rotation = state.rotation;
        targetToFollow = state.target;
    }
}

[System.Serializable]
public class CameraState
{
    public Vector3 position;
    public Quaternion rotation;
    public Transform target;
}