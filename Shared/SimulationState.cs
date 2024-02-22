namespace Animations.Shared;

public class SimulationState
{
    public object[]? GravityFieldData { get; set; }
    public object[]? MagneticFieldData { get; set; }
    public object[]? Magnets { get; set; }
    public float TimeSinceStart { get; set; }
}