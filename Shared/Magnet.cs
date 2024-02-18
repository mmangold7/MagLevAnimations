using System.Numerics;

namespace Animations.Shared;

public class Magnet
{
    public MagnetType Type { get; set; }
    public bool IsFixed { get; set; }
    public Vector3 Position { get; set; }
    public Vector3 Magnetization { get; set; }
    public float Radius { get; set; }
    public float Length { get; set; }
    public float Current { get; set; }
    public float Mass { get; set; }

    public Magnet(
        Vector3 position,
        Vector3 magnetization,
        float radius,
        float length,
        float current,
        float mass,
        MagnetType type,
        bool isFixed)
    {
        Type = type;
        IsFixed = isFixed;
        Position = position;
        Magnetization = magnetization;
        Radius = radius;
        Length = length;
        Current = current;
        Mass = mass;
    }
}