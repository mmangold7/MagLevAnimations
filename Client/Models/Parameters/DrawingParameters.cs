namespace Animations.Client.Models.Parameters;

public class DrawingParameters
{
    public bool ShowCoils { get; set; }
    public string? FieldDrawingStyle { get; set; }
    public bool ShowGravityField { get; set; }
    public bool ShowMagneticField { get; set; }
    public float AmbientLightLevel { get; set; }
    public float CeilingLightLevel { get; set; }
}