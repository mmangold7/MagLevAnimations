using Animations.Shared.Enums;
using Animations.Shared.Extensions;
using BepuPhysics;
using System.Numerics;

namespace Animations.Shared.Models;

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
    public List<Voxel> VoxelsForApproximation { get; set; } = new();

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

    public Vector3 ComputeFieldAtPoint(Vector3 point)
    {
        float mu0 = 4 * (float)Math.PI * 1e-1f;
        Vector3 field = Vector3.Zero;
        float dL = Length / NumElementsInDiscreteApproximation;

        for (int i = 0; i < NumElementsInDiscreteApproximation; i++)
        {
            Vector3 elementPos = Position + Vector3.Transform(new Vector3(0, 0, i * dL - Length / 2 + dL / 2), Orientation.ToMatrix());
            Vector3 r = point - elementPos;
            float rMagnitude = r.Length();
            float preFactor = 3 * mu0 / (4 * (float)Math.PI * (float)Math.Pow(rMagnitude, 4));
            float distance = r.Length();

            if (distance < 1e-6) continue;

            Vector3 dB = (Magnetization * dL * 3.0f * Vector3.Dot(Vector3.Transform(Vector3.UnitZ, Orientation.ToMatrix()), r) * r / (distance * distance * distance) - Magnetization * Vector3.Transform(Vector3.UnitZ, Orientation.ToMatrix())) / (distance * distance);
            dB *= preFactor;
            field += dB;
        }

        return field;
    }

    public Vector3 ComputeForceOnMagnet(Magnet other)
    {
        Vector3 fieldAtOtherMagnet = ComputeFieldAtPoint(other.Position);
        return other.Magnetization * fieldAtOtherMagnet;
    }

    public Vector3 ComputeTorque(Magnet other)
    {
        Vector3 torque = Vector3.Zero;
        float dL = Length / NumElementsInDiscreteApproximation;

        for (int i = 0; i < NumElementsInDiscreteApproximation; i++)
        {
            Vector3 elementPos = Position + Vector3.Transform(new Vector3(0, 0, i * dL - Length / 2 + dL / 2), Orientation.ToMatrix());
            Vector3 localField = other.ComputeFieldAtPoint(elementPos);
            Vector3 dT = Vector3.Cross(Magnetization * dL, localField);
            torque += dT;
        }

        return torque;
    }
}