using System.Numerics;

namespace Animations.Shared;

public class Voxel
{
    public Vector3 Position { get; set; }
    public Vector3 Magnetization { get; set; }
    public float SideLength { get; set; }

    public Voxel(Vector3 position, Vector3 magnetization, float sideLength)
    {
        Position = position;
        Magnetization = magnetization;
        SideLength = sideLength;
    }

    public float Volume => SideLength * SideLength * SideLength;
}