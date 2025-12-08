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
    private float maxMotorTorque = 300f; 
    // Max steering angle in degrees
    private float maxSteeringAngle = 35f; 
    // Brake torque applied when stopping for obstacles
    private float brakeTorque = 1000f;   
    
    // PID Control Variables
    // P: How hard to turn based on current error
    private float Kp = 10.0f; 
    // I: Accumulates error over time (helps with angled slopes)
    private float Ki = 0.05f; 
    // D: Dampens the movement (stops wobbling)
    private float Kd = 5.0f;  

    private float lastError = 0f;
    private float integral = 0f;

    // Raycast settings
    private float rayDistance = 2.0f;
    private float obstacleDetectionDist = 3.0f;

    private void Start()
    {
        // 1. Configure Sensors
        // The instructions allow us to rotate sensors in code. 
        // We rotate them 90 degrees on X so 'transform.forward' points down at the track.
        Quaternion facingDown = Quaternion.Euler(90, 0, 0);
        
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
        // --- 1. Calculate Error (Weighted Average) ---
        // We assign values to sensors:
        // L3: -3, L2: -2, L1: -1, R1: 1, R2: 2, R3: 3
        
        float weightedSum = 0f;
        float sumOfActiveSensors = 0f;

        // Helper function to check line (returns 1 if hit line, 0 if not)
        // We assume the track is DIFFERENT from the floor. 
        // If track is black (low intensity) or a specific tag, adjust 'CheckLine' logic.
        // Here we assume standard Raycast: Hit = Line found.
        
        weightedSum += CheckSensor(L3S) * -3.0f;
        weightedSum += CheckSensor(L2S) * -2.0f;
        weightedSum += CheckSensor(L1S) * -1.0f;
        weightedSum += CheckSensor(R1S) * 1.0f;
        weightedSum += CheckSensor(R2S) * 2.0f;
        weightedSum += CheckSensor(R3S) * 3.0f;

        sumOfActiveSensors += CheckSensor(L3S);
        sumOfActiveSensors += CheckSensor(L2S);
        sumOfActiveSensors += CheckSensor(L1S);
        sumOfActiveSensors += CheckSensor(R1S);
        sumOfActiveSensors += CheckSensor(R2S);
        sumOfActiveSensors += CheckSensor(R3S);

        float currentError = 0f;
        
        if (sumOfActiveSensors > 0)
        {
            currentError = weightedSum / sumOfActiveSensors;
        }
        else
        {
            // If no line is seen, use the last known error to "search" in that direction
            // This handles gaps or sharp turns where we lose the line briefly
            currentError = lastError > 0 ? 3.0f : -3.0f;
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
        // Smart Speed: Slow down if the steering angle is sharp (Cornering)
        // If error is low (straight), go full speed. If error is high (turn), go slow.
        float speedFactor = 1.0f - Mathf.Abs(steerAngle / maxSteeringAngle);
        // Clamp minimum speed so we don't stop completely in turns
        speedFactor = Mathf.Clamp(speedFactor, 0.4f, 1.0f); 

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

            // LOGIC: How do we know it's the line?
            // Option A: Tag check (if the track has a tag)
            // Option B: Color/Texture check (advanced)
            // Option C: The track is raised or specific layer
            
            // Assuming standard coursework simulation where "Hit" means "Track detected" 
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