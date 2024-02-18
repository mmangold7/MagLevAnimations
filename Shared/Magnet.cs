using System.Numerics;

namespace Animations.Shared;

public class Magnet
{
    public Vector3 Position { get; set; } // Center of the magnet
    public Vector3 Magnetization { get; set; } // Direction and magnitude of magnetization
    public float Radius { get; set; }
    public float Length { get; set; }
    public float Current { get; set; } // Equivalent current for magnetization
    public float Mass { get; set; } // Mass of the magnet

    public Magnet(Vector3 position, Vector3 magnetization, float radius, float length, float current, float mass)
    {
        Position = position;
        Magnetization = magnetization;
        Radius = radius;
        Length = length;
        Current = current;
        Mass = mass; // Initialize mass
    }

    // Calculate the magnetic field at a point due to this cylindrical magnet
    public Vector3 CalculateFieldAtPoint(Vector3 point)
    {
        int slices = 10; // Number of slices along the length of the cylinder
        Vector3 totalField = Vector3.Zero;

        for (int i = 0; i < slices; i++)
        {
            // Calculate the center position of the current slice
            float sliceCenterZ = Position.Z - Length / 2 + Length * i / slices + Length / (2 * slices);
            Vector3 sliceCenter = new Vector3(Position.X, Position.Y, sliceCenterZ);

            // Add the field contribution from this slice
            totalField += CalculateFieldFromLoop(sliceCenter, Radius, point, Current);
        }

        return totalField;
    }

    private Vector3 CalculateFieldFromLoop(Vector3 loopCenter, float radius, Vector3 point, float current)
    {
        int segments = 100; // Number of segments to divide the loop into
        Vector3 totalField = Vector3.Zero;
        float mu0 = 4 * (float)Math.PI * 1e-7f; // Vacuum permeability
        float segmentLength = 2 * (float)Math.PI * radius / segments; // Length of each segment

        for (int i = 0; i < segments; i++)
        {
            // Angle of the current segment in radians
            float angle = 2 * (float)Math.PI * i / segments;

            // Position of the current segment
            Vector3 segmentPosition = loopCenter + new Vector3(radius * (float)Math.Cos(angle), radius * (float)Math.Sin(angle), 0);

            // dl is the vector representing the current segment
            Vector3 dl = new Vector3(-(float)Math.Sin(angle), (float)Math.Cos(angle), 0) * segmentLength;

            // r is the vector from the segment to the point
            Vector3 r = point - segmentPosition;
            float rMagnitude = r.Length();

            // Biot-Savart Law
            Vector3 dB = (mu0 * current / (4 * (float)Math.PI)) * (Vector3.Cross(dl, r) / (rMagnitude * rMagnitude * rMagnitude));

            totalField += dB;
        }

        return totalField;
    }
}