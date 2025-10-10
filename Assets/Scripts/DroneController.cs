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


    private float[] rotorSpeeds;
    private Vector3 velocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;

    const float visualRotorSpeed = 24000f;

    private void Start()
    {
        rotorSpeeds = new float[rotors.Length];
        for (int i = 0; i < rotorSpeeds.Length; i++) rotorSpeeds[i] = 0f;
    }

    void Update()
    {
        for (int i = 0; i < rotors.Length; i++)
        {
            rotorSpeeds[i] = 0f;
            if (Input.GetKey(KeyCode.Alpha1 + i))
            {
                rotorSpeeds[i] = maxRotorSpeed;
            }
        }

        for (int i = 0; i < rotors.Length; i++)
        {
            rotors[i].transform.GetChild(0).Rotate(0, rotorSpeeds[i] * Time.deltaTime * visualRotorSpeed, 0);
        }
    }

    private void FixedUpdate()
    {
        PhysicsUpdate();

    }

    private void PhysicsUpdate()
    {
        Vector3 deltaAngularVelocity = Vector3.zero;
        for (int i = 0; i < rotors.Length; i++)
        {
            deltaAngularVelocity += RotorTorque(i);
        }
        deltaAngularVelocity *= Time.fixedDeltaTime;
        deltaAngularVelocity /= mass * SMoI;
        transform.Rotate(angularVelocity + deltaAngularVelocity / 2f, (angularVelocity + deltaAngularVelocity / 2f).magnitude);
        angularVelocity += deltaAngularVelocity;

        Vector3 deltaVelocity = Vector3.zero;
        for (int i = 0; i < rotors.Length; i++)
        {
            deltaVelocity += rotors[i].transform.up * rotorSpeeds[i];
        }
        deltaVelocity += extForces;
        deltaVelocity *= Time.fixedDeltaTime;
        deltaVelocity /= mass;
        transform.Translate(velocity + deltaVelocity / 2f);
        velocity += deltaVelocity;

        angularVelocity *= Mathf.Exp(-angularDrag * Time.fixedDeltaTime);
        velocity *= Mathf.Exp(-linearDrag * Time.fixedDeltaTime);
    }

    private Vector3 RotorTorque(int index)
    {
        return Vector3.Cross(rotors[index].transform.localPosition, rotors[index].transform.up) * rotorSpeeds[index];
    }
}
