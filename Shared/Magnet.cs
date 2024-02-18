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

    public int VoxelResolution { get; set; } // Number of voxels along one dimension
    public List<Voxel> Voxels { get; set; } = new(); // List of voxels composing the magnet


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

    public void GenerateVoxels(int resolution)
    {
        Voxels.Clear();

        float voxelSize = Length / resolution; // Assuming cubic voxels for simplicity
        Vector3 voxelMagneticMoment = Magnetization / (resolution * resolution * resolution);

        for (int x = 0; x < resolution; x++)
        {
            for (int y = 0; y < resolution; y++)
            {
                for (int z = 0; z < resolution; z++)
                {
                    Vector3 voxelPosition = new Vector3(
                        Position.X + (x + 0.5f) * voxelSize - Length / 2,
                        Position.Y + (y + 0.5f) * voxelSize - Length / 2, // Adjust based on actual magnet orientation
                        Position.Z + (z + 0.5f) * voxelSize - Length / 2);

                    Voxels.Add(new Voxel(voxelPosition, voxelMagneticMoment, resolution));
                }
            }
        }
    }
}