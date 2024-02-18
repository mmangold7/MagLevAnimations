using System.Numerics;

namespace Animations.Shared;

public class SimulationManager
{
    public Magnet? TargetMagnet { get; private set; }
    public Magnet? FixedMagnet { get; private set; }
    private readonly float _timeStep = 0.01f;
    private Vector3 _gravity = new(0, -9.81f, 0);
    public Vector3 SimulationExtents { get; private set; }

    public SimulationManager()
    {
        SimulationExtents = new Vector3(1.0f, 1.0f, 1.0f);
        InitializeMagnets();
    }

    private void InitializeMagnets()
    {
        float baseHeight = -(SimulationExtents.Y / 4.0f );
        float gap = 0.1f;

        float targetMagnetHeight = 0.2f;
        float fixedMagnetHeight = 0.2f;

        float targetMagnetMass = 1f;
        float fixedMagnetMass = 1f;

        Vector3 targetPosition = new Vector3(0, baseHeight + targetMagnetHeight / 2 + gap, 0);
        TargetMagnet = new Magnet(targetPosition, new Vector3(0, 1, 0), 0.05f, targetMagnetHeight, 1.0f, targetMagnetMass);
        Vector3 fixedPosition = new Vector3(0, targetPosition.Y + targetMagnetHeight / 2 + fixedMagnetHeight / 2 + gap, 0);
        FixedMagnet = new Magnet(fixedPosition, new Vector3(0, -1, 0), 0.05f, fixedMagnetHeight, 1.0f, fixedMagnetMass);
    }

    public void UpdateSimulation()
    {
        if (TargetMagnet == null || FixedMagnet == null) return;

        // Calculate magnetic field and force as before...
        Vector3 magneticField = FixedMagnet.CalculateFieldAtPoint(TargetMagnet.Position);
        Vector3 magneticForce = TargetMagnet.Magnetization * magneticField.Length(); // Simplified force calculation

        // Apply gravity to the target magnet
        Vector3 gravityForce = _gravity * TargetMagnet.Mass; // Assuming Mass is a property of your Magnet class

        // Combine forces (simplified model, might need more accurate physics for complex simulations)
        Vector3 totalForce = magneticForce + gravityForce;

        // Update the target magnet's position
        TargetMagnet.Position += totalForce * _timeStep;
    }
}