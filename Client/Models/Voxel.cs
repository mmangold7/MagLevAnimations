using System.Numerics;

namespace Animations.Client.Models;

public class Voxel
{
    public Vector3 Position { get; set; }
    public Vector3 Magnetization { get; set; }
    public float SideLength { get; set; }
    public float Volume => SideLength * SideLength * SideLength;

    public Voxel(Vector3 position, Vector3 magnetization, float sideLength)
    {
        Position = position;
        Magnetization = magnetization;
        SideLength = sideLength;
    }
}