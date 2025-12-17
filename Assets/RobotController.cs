using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    [SerializeField] private WheelCollider FLC;
    [SerializeField] private WheelCollider FRC;
    [SerializeField] private WheelCollider RLC;
    [SerializeField] private WheelCollider RRC;

    [SerializeField] private Transform FLT;
    [SerializeField] private Transform FRT;
    [SerializeField] private Transform RLT;
    [SerializeField] private Transform RRT;

    [SerializeField] private Transform FRS;
    [SerializeField] private Transform L1S;
    [SerializeField] private Transform L2S;
    [SerializeField] private Transform L3S;
    [SerializeField] private Transform R1S;
    [SerializeField] private Transform R2S;
    [SerializeField] private Transform R3S;
    [SerializeField] private Transform ORS;

    // --- TUNING PARAMETERS (Adjust these in code if needed) ---
    // Max torque (power) applied to wheels
    private float maxMotorTorque = 250f;  // Reduced max speed
    // Max steering angle in degrees
    private float maxSteeringAngle = 45f; 
    // Brake torque applied when stopping for obstacles
    private float brakeTorque = 1000f;   
    
    // PID Control Variables
    // P: How hard to turn based on current error
    private float Kp = 15.0f; 
    // I: Accumulates error over time (helps with angled slopes)
    private float Ki = 0.1f; 
    // D: Dampens the movement (stops wobbling)
    private float Kd = 8.0f;  

    private float lastError = 0f;
    private float integral = 0f;

    // Raycast settings
    private float rayDistance = 4.0f;  // Increased for earlier edge detection
    private float obstacleDetectionDist = 3.0f;

    private void Start()
    {
        // 1. Configure Sensors
        // The instructions allow us to rotate sensors in code. 
        // We rotate them 90 degrees on X so 'transform.forward' points down at the track.
        Quaternion facingDown = Quaternion.Euler(-90, 0, 0);
        
        if(L1S) L1S.localRotation = facingDown;
        if(L2S) L2S.localRotation = facingDown;
        if(L3S) L3S.localRotation = facingDown;
        if(R1S) R1S.localRotation = facingDown;
        if(R2S) R2S.localRotation = facingDown;
        if(R3S) R3S.localRotation = facingDown;

        // Front sensor should point forward (default), or slight down angle
        if(FRS) FRS.localRotation = Quaternion.Euler(0, 0, 0); 

        // 2. Configure Wheel Physics (Optional safety)
        // Ensure the center of mass is low to prevent flipping on tight turns
        GetComponent<Rigidbody>().centerOfMass = new Vector3(0, -0.5f, 0);
    }

    private void FixedUpdate()
    {
        HandleSensingAndSteering();
        HandleObstacles();
        UpdateVisuals();
    }

    private void HandleSensingAndSteering()
    {
        // --- 1. Calculate Error for ROAD/PATH FOLLOWING ---
        // For a road with edges on both sides, the robot should stay in the middle
        // where sensors DON'T detect the edges.
        // If left sensors hit edge → steer RIGHT (away from edge)
        // If right sensors hit edge → steer LEFT (away from edge)
        
        int leftHits = 0;
        int rightHits = 0;

        // Count edge detections on each side
        leftHits += CheckSensor(L3S);
        leftHits += CheckSensor(L2S);
        leftHits += CheckSensor(L1S);
        
        rightHits += CheckSensor(R3S);
        rightHits += CheckSensor(R2S);
        rightHits += CheckSensor(R1S);

        // Calculate error based on which side detects the edge
        float currentError = 0f;
        
        if (leftHits > 0 && rightHits == 0)
        {
            // Left sensors see edge → too far left → steer RIGHT (positive error)
            currentError = leftHits * 4.0f;  // Increased reaction
        }
        else if (rightHits > 0 && leftHits == 0)
        {
            // Right sensors see edge → too far right → steer LEFT (negative error)
            currentError = -rightHits * 4.0f;  // Increased reaction
        }
        else if (leftHits > 0 && rightHits > 0)
        {
            // Both sides see edges → calculate balance to stay centered
            currentError = (leftHits - rightHits) * 3.0f;  // Increased reaction
        }
        else
        {
            // No edges detected → continue straight or use last error
            currentError = lastError * 0.5f;
        }

        // --- 2. PID Calculations ---
        
        // Proportional
        float P = currentError * Kp;
        
        // Integral (with clamping to prevent "Windup")
        integral += currentError * Time.fixedDeltaTime;
        integral = Mathf.Clamp(integral, -10f, 10f);
        float I = integral * Ki;

        // Derivative
        float D = (currentError - lastError) / Time.fixedDeltaTime * Kd;
        
        float pidOutput = P + I + D;
        
        lastError = currentError;

        // --- 3. Apply Steering ---
        float steerAngle = Mathf.Clamp(pidOutput, -maxSteeringAngle, maxSteeringAngle);
        
        FLC.steerAngle = steerAngle;
        FRC.steerAngle = steerAngle;

        // --- 4. Apply Throttle ---
        // Smart Speed: Slow down AGGRESSIVELY if the steering angle is sharp (Cornering)
        // If error is low (straight), go full speed. If error is high (turn), go very slow.
        
        // More aggressive speed reduction based on steering
        float speedFactor = 1.0f - (Mathf.Abs(steerAngle / maxSteeringAngle) * 2.0f);  // Even more aggressive
        // Also reduce speed based on error magnitude (anticipate turns)
        float errorFactor = 1.0f - (Mathf.Abs(currentError) / 8.0f);  // React to smaller errors
        
        // Combine both factors (use the lower one)
        speedFactor = Mathf.Min(speedFactor, errorFactor);
        
        // Clamp minimum speed - allow very slow speeds in sharp turns
        speedFactor = Mathf.Clamp(speedFactor, 0.1f, 1.0f);  // Can go as slow as 10%

        float currentTorque = maxMotorTorque * speedFactor;

        // Apply drive to all wheels (AWD is better for traction on slopes)
        FLC.motorTorque = currentTorque;
        FRC.motorTorque = currentTorque;
        RLC.motorTorque = currentTorque;
        RRC.motorTorque = currentTorque;
    }

    private void HandleObstacles()
    {
        // If FRS is not assigned, skip
        if (FRS == null) return;

        RaycastHit hit;
        // Cast ray forward from the front sensor
        if (Physics.Raycast(FRS.position, FRS.forward, out hit, obstacleDetectionDist))
        {
            // We hit something!
            // Check if it is NOT the ground (assuming obstacles stick up)
            // Or check by tag if obstacles are tagged "Obstacle"
            // For now, simple proximity check:
            
            Debug.DrawLine(FRS.position, hit.point, Color.red);

            // Emergency Brake Logic
            FLC.brakeTorque = brakeTorque;
            FRC.brakeTorque = brakeTorque;
            RLC.brakeTorque = brakeTorque;
            RRC.brakeTorque = brakeTorque;
            
            // Cut power
            FLC.motorTorque = 0;
            FRC.motorTorque = 0;
            RLC.motorTorque = 0;
            RRC.motorTorque = 0;
        }
        else
        {
            // Release brakes
            FLC.brakeTorque = 0;
            FRC.brakeTorque = 0;
            RLC.brakeTorque = 0;
            RRC.brakeTorque = 0;
            
            Debug.DrawRay(FRS.position, FRS.forward * obstacleDetectionDist, Color.green);
        }
    }

    private int CheckSensor(Transform sensor)
    {
        if (sensor == null) return 0;

        RaycastHit hit;
        // Raycast downwards (sensor.forward because we rotated it in Start)
        if (Physics.Raycast(sensor.position, sensor.forward, out hit, rayDistance))
        {
            // Visualize the ray
            Debug.DrawLine(sensor.position, hit.point, Color.cyan);

            // LOGIC: How do we know it's the line/edge?
            // Option A: Tag check (if the track has a tag)
            // Option B: Color/Texture check (advanced)
            // Option C: The track is raised or specific layer
            
            // Assuming standard coursework simulation where "Hit" means "Track/Edge detected" 
            // OR the floor is on a different layer. 
            // If the floor is detected everywhere, we need to check the object name or tag.
            
            // ADAPT THIS: If the ray hits "Track" or "Line", return 1.
            // If the generic floor is tagged "Ground" and track is "Track":
            // if (hit.collider.CompareTag("Track")) return 1;
            
            // For general robust detection (assuming track is slightly raised or distinct):
            return 1; 
        }
        return 0;
    }

    private void UpdateVisuals()
    {
        ApplyWheelPose(FLC, FLT);
        ApplyWheelPose(FRC, FRT);
        ApplyWheelPose(RLC, RLT);
        ApplyWheelPose(RRC, RRT);
    }

    // Helper to sync mesh with collider
    private void ApplyWheelPose(WheelCollider collider, Transform visualWheel)
    {
        if (collider == null || visualWheel == null) return;

        Vector3 pos;
        Quaternion rot;
        collider.GetWorldPose(out pos, out rot);

        visualWheel.position = pos;
        visualWheel.rotation = rot;
    }
}