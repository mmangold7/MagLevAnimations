using System.Numerics;

namespace Animations.Shared;

public class FieldVector
{
    public Vector3 Position { get; set; }
    public Vector3 Direction { get; set; }
    public float Magnitude { get; set; }
    public int Index { get; set; }

    public FieldVector(Vector3 position, Vector3 direction, float magnitude, int index)
    {
        Position = position;
        Direction = direction;
        Magnitude = magnitude;
        Index = index;
    }
}