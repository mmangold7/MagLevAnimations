namespace Animations.Client.Models;

public class SimulationStateForVisualization
{
    public float TimeSinceStart { get; set; }
    public object[]? Magnets { get; set; }
    public object[]? GravityFieldData { get; set; }
    public object[]? MagneticFieldData { get; set; }
    public object[]? MiddleSliceOfFieldVectors { get; set; }
}