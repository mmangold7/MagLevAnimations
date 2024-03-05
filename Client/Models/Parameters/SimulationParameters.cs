using System.Numerics;
using Animations.Client.Enums;

namespace Animations.Client.Models.Parameters;

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