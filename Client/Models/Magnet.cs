using System.Numerics;
using Animations.Client.Enums;
using Animations.Client.Extensions;
using BepuPhysics;

namespace Animations.Client.Models;

public class Magnet
{
    private const int NumElementsInDiscreteApproximation = 10;

    public bool IsFixed { get; set; }
    public float Mass { get; set; }
    public float Radius { get; set; }
    public float Length { get; set; }
    public float MomentOfInertia => 0.5f * Mass * Radius * Radius;
    public Vector3 Position { get; set; }
    public Vector3 Magnetization { get; set; }
    public Vector3 Velocity { get; set; }
    public Vector3 AngularVelocity { get; set; }
    public Quaternion Orientation { get; set; }
    public MagnetType Type { get; set; }
    public BodyHandle PhysicsEngineBodyHandle { get; set; }
    public List<Voxel> Voxels { get; set; } = new();

    public Magnet(
        Vector3 position,
        Vector3 magnetization,
        float mass,
        float radius,
        float length,
        bool isFixed,
        MagnetType type)
    {
        Position = position;
        Magnetization = magnetization;
        Mass = mass;
        Radius = radius;
        Length = length;
        Velocity = Vector3.Zero;
        AngularVelocity = Vector3.Zero;
        Orientation = Quaternion.Identity;
        IsFixed = isFixed;
        Type = type;
    }

    public void UpdatePositionAndAngularPosition(float deltaTime)
    {
        Position += Velocity * deltaTime;
        UpdateOrientation(deltaTime);
    }

    public void UpdateAngularVelocity(Vector3 torque, float deltaTime)
    {
        var angularAcceleration = torque / MomentOfInertia;
        AngularVelocity += angularAcceleration * deltaTime;
    }

    public void UpdateOrientation(float deltaTime)
    {
        var w = new Quaternion(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z, 0);

        var qDot = Quaternion.Multiply(w, Orientation);

        var scaledQDot = new Quaternion(
            qDot.X * deltaTime * 0.5f, qDot.Y * deltaTime * 0.5f,
            qDot.Z * deltaTime * 0.5f, qDot.W * deltaTime * 0.5f);

        Orientation = new Quaternion(
            Orientation.X + scaledQDot.X,
            Orientation.Y + scaledQDot.Y,
            Orientation.Z + scaledQDot.Z,
            Orientation.W + scaledQDot.W
        );

        Orientation = Quaternion.Normalize(Orientation);
    }

    public Vector3 ComputeMultipleDipoleFieldAtPoint(Vector3 point)
    {
        var totalField = Vector3.Zero;
        var dL = Length / NumElementsInDiscreteApproximation;

        var segmentMagnetization = Magnetization / NumElementsInDiscreteApproximation;

        for (int i = 0; i < NumElementsInDiscreteApproximation; i++)
        {
            var segmentCenter = Position + Vector3.Transform(new Vector3(0, 0, i * dL - Length / 2 + dL / 2), Orientation.ToMatrix());

            var fieldContribution = CalculateSingleDipoleFieldAtPoint(segmentCenter, segmentMagnetization, point);

            totalField += fieldContribution;
        }

        return totalField;
    }

    public Vector3 CalculateMagneticFieldAtPoint(Vector3 point)
    {
        Vector3 r = point - Position;
        float rMagnitude = r.Length();
        Vector3 rHat = r / rMagnitude;

        float mu0 = 4 * (float)Math.PI * 1e-7f;  // Use the correct value for mu0
        Vector3 B = (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                    (3 * Vector3.Dot(Magnetization, rHat) * rHat - Magnetization);

        return B;
    }

    public Vector3 CalculateTorque(Vector3 externalField)
    {
        return Vector3.Cross(Magnetization, externalField);
    }

    private static Vector3 CalculateSingleDipoleFieldAtPoint(Vector3 dipolePosition, Vector3 dipoleMagnetization, Vector3 point)
    {
        Vector3 r = point - dipolePosition;
        var mu0 = 4 * (float)Math.PI * 1e-7f;
        var rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        return (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
               (3 * Vector3.Dot(dipoleMagnetization, r) * r - dipoleMagnetization * rMagnitude * rMagnitude);
    }

    public Vector3 ComputeForceOnMagnet(Magnet other) => other.Magnetization * ComputeMultipleDipoleFieldAtPoint(other.Position);

    public Vector3 ComputeTorque(Magnet other)
    {
        Vector3 torque = Vector3.Zero;
        float dL = Length / NumElementsInDiscreteApproximation;

        for (int i = 0; i < NumElementsInDiscreteApproximation; i++)
        {
            Vector3 elementPos = Position + Vector3.Transform(new Vector3(0, 0, i * dL - Length / 2 + dL / 2), Orientation.ToMatrix());
            Vector3 localField = other.ComputeMultipleDipoleFieldAtPoint(elementPos);
            Vector3 dT = Vector3.Cross(Magnetization * dL, localField);
            torque += dT;
        }

        return torque;
    }
}