using System.Numerics;
using Animations.Shared.Enums;

namespace Animations.Shared.Models.Parameters;

public class SimulationParameters
{
    public float TimeStep { get; set; }
    public Vector3 Gravity { get; set; }
    public Vector3 SimulationExtents { get; set; }
    public SimulationMode SimulationMode { get; set; }
    public int Divisions { get; set; }
    public bool ShowGravityField { get; set; }
    public bool ShowMagneticField { get; set; }
    public int VoxelsPerDivision { get; set; }
}