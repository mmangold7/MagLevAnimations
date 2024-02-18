using System.Numerics;

namespace Animations.Shared;

public class SimulationManager
{
    public List<Magnet> Magnets { get; set; } = new();
    public List<FieldVector> GravityFieldVectors { get; set; } = new();
    public List<FieldVector> MagneticFieldVectors { get; set; } = new();
    private readonly float _timeStep = 0.01f;
    private Vector3 _gravity = new(0, -9.81f, 0);
    public Vector3 SimulationExtents { get; set; } = new(1.0f, 1.0f, 1.0f);

    public void AddMagnet(Magnet magnet)
    {
        Magnets.Add(magnet);
    }

    public void UpdateSimulation()
    {
        if (!Magnets.Any()) return;

        var levitationTargets = Magnets.Where(m => !m.IsFixed);

        foreach (var target in levitationTargets)
        {
            Vector3 totalMagneticForce = Vector3.Zero;

            foreach (var magnet in Magnets.Where(m => m != target))
            {
                Vector3 magneticFieldAtTarget = CalculateFieldAtPoint(magnet, target.Position);
                Vector3 magneticForce = magneticFieldAtTarget * target.Magnetization.Length();
                totalMagneticForce += magneticForce;
            }

            Vector3 gravityForce = _gravity * target.Mass;
            Vector3 totalForce = totalMagneticForce + gravityForce;
            target.Position += totalForce * _timeStep;
        }
    }

    public Vector3 CalculateFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        int slices = 10;
        Vector3 totalField = Vector3.Zero;

        for (int i = 0; i < slices; i++)
        {
            float sliceCenterZ = sourceMagnet.Position.Z - sourceMagnet.Length / 2 + sourceMagnet.Length * i / slices + sourceMagnet.Length / (2 * slices);
            Vector3 sliceCenter = new Vector3(sourceMagnet.Position.X, sourceMagnet.Position.Y, sliceCenterZ);
            totalField += CalculateFieldFromLoop(sourceMagnet, sliceCenter, point);
        }

        return totalField;
    }

    private Vector3 CalculateFieldFromLoop(Magnet sourceMagnet, Vector3 loopCenter, Vector3 point)
    {
        int segments = 100;
        Vector3 totalField = Vector3.Zero;
        float mu0 = 4 * (float)Math.PI * 1e-7f;
        float segmentLength = 2 * (float)Math.PI * sourceMagnet.Radius / segments;

        for (int i = 0; i < segments; i++)
        {
            float angle = 2 * (float)Math.PI * i / segments;
            Vector3 segmentPosition = loopCenter + new Vector3(sourceMagnet.Radius * (float)Math.Cos(angle), sourceMagnet.Radius * (float)Math.Sin(angle), 0);
            Vector3 dl = new Vector3(-(float)Math.Sin(angle), (float)Math.Cos(angle), 0) * segmentLength;
            Vector3 r = point - segmentPosition;
            float rMagnitude = r.Length();
            Vector3 dB = (mu0 * sourceMagnet.Current / (4 * (float)Math.PI)) * (Vector3.Cross(dl, r) / (rMagnitude * rMagnitude * rMagnitude));
            totalField += dB;
        }

        return totalField;
    }

    public void CalculateGravityField(int divisions)
    {
        GravityFieldVectors.Clear();
        Vector3 gravityDirection = Vector3.Normalize(_gravity);
        float divisionLength = SimulationExtents.X / divisions;
        float gravityMagnitude = Math.Min(9.81f, divisionLength);

        for (int x = 0; x < divisions; x++)
        {
            for (int y = 0; y < divisions; y++)
            {
                for (int z = 0; z < divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, divisions);
                    GravityFieldVectors.Add(new FieldVector(position, Vector3.Normalize(gravityDirection), gravityMagnitude));
                }
            }
        }
    }

    public void CalculateMagneticField(int divisions)
    {
        MagneticFieldVectors.Clear();
        float divisionLength = SimulationExtents.X / divisions;
        float maxMagnitude = 0f;
        float minMagnitude = float.MaxValue;

        List<FieldVector> tempVectors = new List<FieldVector>();

        for (int x = 0; x < divisions; x++)
        {
            for (int y = 0; y < divisions; y++)
            {
                for (int z = 0; z < divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, divisions);
                    Vector3 totalMagneticField = Vector3.Zero;

                    foreach (var magnet in Magnets)
                    {
                        Vector3 magneticField = CalculateFieldAtPoint(magnet, position);
                        totalMagneticField += magneticField;
                    }

                    float magnitude = totalMagneticField.Length();
                    maxMagnitude = Math.Max(maxMagnitude, magnitude);
                    minMagnitude = Math.Min(minMagnitude, magnitude);

                    tempVectors.Add(new FieldVector(position, totalMagneticField, magnitude));
                }
            }
        }

        foreach (var vector in tempVectors)
        {
            float scaledMagnitude = ScaleMagnitude(vector.Magnitude, minMagnitude, maxMagnitude, divisionLength * 0.1f, divisionLength);
            Vector3 normalizedDirection = Vector3.Normalize(vector.Direction) * scaledMagnitude;

            MagneticFieldVectors.Add(new FieldVector(vector.Position, normalizedDirection, scaledMagnitude));
        }
    }

    private float ScaleMagnitude(float value, float min, float max, float newMin, float newMax)
    {
        if (max - min == 0) return newMin;
        return (value - min) / (max - min) * (newMax - newMin) + newMin;
    }

    private Vector3 CalculatePositionInSpace(int x, int y, int z, int divisions)
    {
        float stepSize = SimulationExtents.X / divisions;
        float offset = stepSize / 2.0f;

        return new Vector3(
            (x * stepSize + offset) - (SimulationExtents.X / 2.0f),
            (y * stepSize + offset) - (SimulationExtents.Y / 2.0f),
            (z * stepSize + offset) - (SimulationExtents.Z / 2.0f)
        );
    }

    public void InitializeTwoMagnets()
    {
        float baseHeight = -(SimulationExtents.Y / 4.0f);
        float gap = 0.1f;

        float targetMagnetHeight = 0.1f;
        float fixedMagnetHeight = 0.1f;

        float targetMagnetMass = 1f;
        float fixedMagnetMass = 1f;

        Vector3 targetPosition = new Vector3(0, baseHeight + targetMagnetHeight / 2 + gap, 0);
        var permanentMagnet = new Magnet(targetPosition, new Vector3(0, 1, 0), 0.05f, targetMagnetHeight, 1.0f,
            targetMagnetMass, MagnetType.Permanent, false);

        Vector3 fixedPosition = new Vector3(0, targetPosition.Y + targetMagnetHeight / 2 + fixedMagnetHeight / 2 + gap, 0);
        var targetMagnet = new Magnet(fixedPosition, new Vector3(0, -1, 0), 0.05f, fixedMagnetHeight, 1.0f, fixedMagnetMass,
            MagnetType.Permanent, true);

        AddMagnet(permanentMagnet);
        AddMagnet(targetMagnet);
    }
}