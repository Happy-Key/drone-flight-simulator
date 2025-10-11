using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    [SerializeField] private GameObject[] rotors;
    [SerializeField] private float mass;
    [SerializeField] private float SMoI = 1f;
    [SerializeField] private float maxRotorSpeed;

    [SerializeField] private Vector3 extForces;
    [SerializeField] private float linearDrag;
    [SerializeField] private float angularDrag;

    [Header("Controller Parameters")]
    [SerializeField] private float ascentSpeed;
    [SerializeField] private float tilt;

    [Header("PID Parameters")]
    [SerializeField] private float vkP;
    [SerializeField] private float vkI;
    [SerializeField] private float vkD;
    [SerializeField] private float rkP;
    [SerializeField] private float rkI;
    [SerializeField] private float rkD;
    [SerializeField] private float akP;
    [SerializeField] private float akI;
    [SerializeField] private float akD;

    private float[] rotorSpeeds;
    private Vector3 velocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;

    private Vector3 accumulatedVelocityError = Vector3.zero;
    private Vector3 previousVelocityError = Vector3.zero;
    private Vector3 accumulatedRotationError = Vector3.zero;
    private Vector3 previousRotationError = Vector3.zero;
    private Vector3 accumulatedAVError = Vector3.zero;
    private Vector3 previousAVError = Vector3.zero;

    private bool hovering = true;

    const float visualRotorSpeed = 5000f;

    private void Start()
    {
        rotorSpeeds = new float[rotors.Length];
        for (int i = 0; i < rotorSpeeds.Length; i++) rotorSpeeds[i] = 0f;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.H)) hovering = !hovering;

        for (int i = 0; i < rotors.Length; i++)
        {
            rotors[i].transform.GetChild(0).Rotate(0, rotorSpeeds[i] * Time.deltaTime * visualRotorSpeed * (i % 2 == 0 ? 1 : -1), 0);
        }

        for (int i = 0; i < rotors.Length; i++)
        {
            rotorSpeeds[i] = 0f;
            if (Input.GetKey(KeyCode.Alpha1 + i))
            {
                rotorSpeeds[i] = maxRotorSpeed;
            }
        }
    }

    private void FixedUpdate()
    {
        if (hovering)
        {
            DirectedHover(Input.GetAxis("Elevation") * Vector3.up * ascentSpeed, Vector3.up + Input.GetAxis("Vertical") * transform.forward * tilt + Input.GetAxis("Horizontal") * transform.right * tilt);
        }

        PhysicsUpdate();
    }

    private void PhysicsUpdate()
    {
        Vector3 deltaAngularVelocity = Vector3.zero;
        for (int i = 0; i < rotors.Length; i++)
        {
            deltaAngularVelocity += RotorTorquePerSpeed(i) * rotorSpeeds[i];
        }
        deltaAngularVelocity *= Time.fixedDeltaTime;
        deltaAngularVelocity /= mass * SMoI;
        transform.Rotate(angularVelocity + deltaAngularVelocity / 2f, (angularVelocity + deltaAngularVelocity / 2f).magnitude * Time.fixedDeltaTime);
        angularVelocity += deltaAngularVelocity;

        Vector3 deltaVelocity = Vector3.zero;
        for (int i = 0; i < rotors.Length; i++)
        {
            deltaVelocity += rotors[i].transform.up * rotorSpeeds[i];
        }
        deltaVelocity += extForces;
        deltaVelocity *= Time.fixedDeltaTime;
        deltaVelocity /= mass;
        transform.position += (velocity + deltaVelocity / 2f) * Time.fixedDeltaTime;
        velocity += deltaVelocity;

        angularVelocity *= Mathf.Exp(-angularDrag * Time.fixedDeltaTime);
        velocity *= Mathf.Exp(-linearDrag * Time.fixedDeltaTime);
    }

    private void DirectedHover(Vector3 targetVerticalVelocity, Vector3 targetUpVector)
    {

        Vector3 velocityError = targetVerticalVelocity - velocity;
        accumulatedVelocityError += velocityError * Time.fixedDeltaTime;
        Vector3 targetForce = mass * PID(vkP, vkI, vkD, velocityError, accumulatedVelocityError, previousVelocityError);
        previousVelocityError = velocityError;

        Vector3 rotationError = Vector3.Angle(targetUpVector, transform.up) * Vector3.Cross(transform.up, targetUpVector).normalized;
        accumulatedRotationError += rotationError * Time.fixedDeltaTime;
        Vector3 targetAngularVelocity = PID(rkP, rkI, rkD, rotationError, accumulatedRotationError, previousRotationError);
        previousRotationError = rotationError;

        Vector3 AVError = targetAngularVelocity - angularVelocity;
        accumulatedAVError += AVError * Time.fixedDeltaTime;
        Vector3 targetTorque = mass * SMoI * PID(akP, akI, akD, AVError, accumulatedAVError, previousAVError);
        previousAVError = AVError;


        float[,] A = new float[4, rotors.Length];
        float[] B = { targetTorque.x, targetTorque.y, targetTorque.z, targetForce.y };
        for (int i = 0; i < rotors.Length; i++)
        {
            A[0, i] = RotorTorquePerSpeed(i).x;
            A[1, i] = RotorTorquePerSpeed(i).y;
            A[2, i] = RotorTorquePerSpeed(i).z;
            A[3, i] = 1;
        }
        GaussianElimination.Solve(A, B);
        for(int i = 0; i < rotorSpeeds.Length; i++)
        {
            rotorSpeeds[i] = Mathf.Clamp(B[i], -maxRotorSpeed, maxRotorSpeed);
        }
    }

    private Vector3 PID(float kP, float kI, float kD, Vector3 currentError, Vector3 accumulatedError, Vector3 previousError)
    {
        return kP * currentError + kI * accumulatedError + kD * (currentError - previousError);
    }

    private Vector3 RotorTorquePerSpeed(int index)
    {
        return Vector3.Cross(rotors[index].transform.position - transform.position, rotors[index].transform.up) + rotors[index].transform.up * (index % 2 == 1 ? 1 : -1);
    }
}
