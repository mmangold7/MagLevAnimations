using System.Numerics;

namespace Animations.Shared;

public class SimulationManager
{
    private readonly float _timeStep;
    private readonly Vector3 _gravity;
    private readonly Vector3 _simulationExtents;

    public SimulationManager(float timeStep, Vector3 gravity, Vector3 simulationExtents)
    {
        _timeStep = timeStep;
        _gravity = gravity;
        _simulationExtents = simulationExtents;
    }

    public SimulationMode CurrentMode { get; set; } = SimulationMode.DipoleApproximation;
    public List<Magnet> Magnets { get; set; } = new();
    public List<FieldVector> GravityFieldVectors { get; set; } = new();
    public List<FieldVector> MagneticFieldVectors { get; set; } = new();
    public bool ShowGravityField { get; set; }
    public int Divisions { get; set; }
    public bool ShowMagneticField { get; set; }

    public void SwitchApproximationTo(SimulationMode newMode, int divisions = 10, int voxelsPerDivision = 10)
    {
        if (CurrentMode != newMode)
        {
            CurrentMode = newMode;
            switch (newMode)
            {
                case SimulationMode.VoxelBased:
                {
                    var divisionLength = _simulationExtents.X / divisions;
                    var voxelLength = divisionLength / voxelsPerDivision;

                    foreach (var magnet in Magnets)
                        GenerateVoxelsForMagnet(magnet, voxelLength);
                    break;
                }
                case SimulationMode.DipoleApproximation:
                {
                    foreach (var magnet in Magnets)
                        magnet.Voxels.Clear();
                    break;
                }
            }
        }
    }

    private void GenerateVoxelsForMagnet(Magnet magnet, float voxelLength)
    {
        var voxelsPerDimension = (int)Math.Round(magnet.Length / voxelLength);
        var voxelMagneticMoment = magnet.Magnetization / (voxelsPerDimension * voxelsPerDimension * voxelsPerDimension);

        for (int x = 0; x < voxelsPerDimension; x++)
        {
            for (int y = 0; y < voxelsPerDimension; y++)
            {
                for (int z = 0; z < voxelsPerDimension; z++)
                {
                    var voxelPosition = new Vector3(
                        magnet.Position.X + (x + 0.5f) * voxelLength - magnet.Length / 2,
                        magnet.Position.Y + (y + 0.5f) * voxelLength - magnet.Length / 2,
                        magnet.Position.Z + (z + 0.5f) * voxelLength - magnet.Length / 2);

                    magnet.Voxels.Add(new Voxel(voxelPosition, voxelMagneticMoment, voxelLength));
                }
            }
        }
    }

    public void AddMagnet(Magnet magnet)
    {
        Magnets.Add(magnet);
    }

    public SimulationState UpdateSimulation()
    {
        switch (CurrentMode)
        {
            case SimulationMode.DipoleApproximation:
            {
                Profiling.RunWithClockingLog(UpdateMagnetPositionsUsingDipoleApproximation);
                break;
            }
            case SimulationMode.VoxelBased:
            {
                Profiling.RunWithClockingLog(UpdateMagnetsUsingVoxelApproximation);
                break;
            }
        }
        
        if (ShowGravityField) Profiling.RunWithClockingLog(CalculateGravityField);
        if (ShowMagneticField) Profiling.RunWithClockingLog(CalculateMagneticField);

        return GetSimulationState();
    }

    public SimulationState GetSimulationState()
    {
        return new SimulationState
        {
            Magnets = Magnets.Select(ConvertMagnetToJsObject).ToArray(),
            GravityFieldData = GravityFieldVectors.Select(ConvertFieldToJsObject).ToArray(),
            MagneticFieldData = MagneticFieldVectors.Select(ConvertFieldToJsObject).ToArray()
        };
    }

    private object ConvertMagnetToJsObject(Magnet magnet) => new
    {
        position = new { magnet.Position.X, magnet.Position.Y, magnet.Position.Z },
        radius = magnet.Radius,
        length = magnet.Length,
        magnetization = new { magnet.Magnetization.X, magnet.Magnetization.Y, magnet.Magnetization.Z, },
    };

    private object ConvertFieldToJsObject(FieldVector vector) => new
    {
        position = new { vector.Position.X, vector.Position.Y, vector.Position.Z },
        direction = new { vector.Direction.X, vector.Direction.Y, vector.Direction.Z },
        magnitude = vector.Magnitude,
        index = vector.Index
    };

    private void UpdateMagnetsUsingVoxelApproximation()
    {
        foreach (var targetMagnet in Magnets.Where(m => !m.IsFixed))
        {
            Vector3 totalForce = Vector3.Zero;

            foreach (var sourceMagnet in Magnets.Where(m => m != targetMagnet))
            {
                foreach (var targetVoxel in targetMagnet.Voxels)
                {
                    Vector3 voxelForce = Vector3.Zero;

                    foreach (var sourceVoxel in sourceMagnet.Voxels)
                    {
                        voxelForce += CalculateDipoleDipoleForce(targetVoxel, sourceVoxel);
                    }

                    totalForce += voxelForce;
                }
            }

            Vector3 gravityForce = _gravity * targetMagnet.Mass;
            totalForce += gravityForce;
            targetMagnet.Position += totalForce * _timeStep / targetMagnet.Mass;
        }
    }

    private void UpdateMagnetPositionsUsingDipoleApproximation()
    {
        foreach (var target in Magnets.Where(m => !m.IsFixed))
        {
            Vector3 totalForce = Vector3.Zero;

            foreach (var source in Magnets.Where(m => m != target))
            {
                Vector3 forceFromSource = CalculateDipoleDipoleForce(target, source);
                totalForce += forceFromSource;
            }

            Vector3 gravityForce = _gravity * target.Mass;
            totalForce += gravityForce;
            target.Position += totalForce / target.Mass * _timeStep;
        }
    }

    private Vector3 CalculateDipoleDipoleForce(Magnet target, Magnet source)
    {
        Vector3 r = target.Position - source.Position;
        float rMagnitude = r.Length();
        Vector3 rHat = r / rMagnitude;

        float mu0 = 4 * (float)Math.PI * 1e-7f;
        float prefactor = (3 * mu0) / (4 * (float)Math.PI * (float)Math.Pow(rMagnitude, 4));

        float m1DotR = Vector3.Dot(source.Magnetization, rHat);
        float m2DotR = Vector3.Dot(target.Magnetization, rHat);
        float m1DotM2 = Vector3.Dot(source.Magnetization, target.Magnetization);

        Vector3 term1 = (m2DotR * source.Magnetization);
        Vector3 term2 = (m1DotR * target.Magnetization);
        Vector3 term3 = m1DotM2 * rHat;
        Vector3 term4 = -5 * (m1DotR * m2DotR / (float)Math.Pow(rMagnitude, 2)) * rHat;

        Vector3 force = prefactor * (term1 + term2 + term3 + term4);

        return force;
    }

    private Vector3 CalculateDipoleDipoleForce(Voxel targetVoxel, Voxel sourceVoxel)
    {
        Vector3 r = targetVoxel.Position - sourceVoxel.Position;
        float mu0 = 4 * (float)Math.PI * 1e-7f;
        float m1 = sourceVoxel.Magnetization.Length() * sourceVoxel.Volume;
        float m2 = targetVoxel.Magnetization.Length() * targetVoxel.Volume;
        float rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        Vector3 force = (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                        (3 * (Vector3.Dot(sourceVoxel.Magnetization, r) * targetVoxel.Magnetization) +
                         3 * (Vector3.Dot(targetVoxel.Magnetization, r) * sourceVoxel.Magnetization) -
                         5 * Vector3.Dot(sourceVoxel.Magnetization, targetVoxel.Magnetization) * r / rMagnitude) -
                        (mu0 / (3 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) * m1 * m2 * r;

        return force;
    }

    public Vector3 CalculateFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        Vector3 totalField = Vector3.Zero;

        switch (CurrentMode)
        {
            case SimulationMode.DipoleApproximation:
            {
                Vector3 r = point - sourceMagnet.Position;
                float mu0 = 4 * (float)Math.PI * 1e-7f;
                float rMagnitude = r.Length();

                if (rMagnitude == 0) return Vector3.Zero;

                totalField = (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                             (3 * Vector3.Dot(sourceMagnet.Magnetization, r) * r - sourceMagnet.Magnetization * rMagnitude * rMagnitude);
                break;
            }
            case SimulationMode.VoxelBased:
                totalField = sourceMagnet.Voxels.Aggregate(totalField, (current, voxel) =>
                    current + CalculateFieldFromVoxel(voxel, point));
                break;
        }

        return totalField;
    }

    private Vector3 CalculateFieldFromVoxel(Voxel voxel, Vector3 point)
    {
        Vector3 r = point - voxel.Position;
        float mu0 = 4 * (float)Math.PI * 1e-7f;
        float rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        Vector3 field = (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                        (3 * Vector3.Dot(voxel.Magnetization, r) * r - voxel.Magnetization * rMagnitude * rMagnitude);

        return field;
    }

    public void CalculateGravityField()
    {
        GravityFieldVectors.Clear();
        Vector3 gravityDirection = Vector3.Normalize(_gravity);
        float divisionLength = _simulationExtents.X / Divisions;
        float gravityMagnitude = Math.Min(9.81f, divisionLength);

        for (int x = 0; x < Divisions; x++)
        {
            for (int y = 0; y < Divisions; y++)
            {
                for (int z = 0; z < Divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, Divisions);
                    var flatIndex = x * _simulationExtents.Y * _simulationExtents.Z + y * _simulationExtents.Z + z;
                    GravityFieldVectors.Add(new FieldVector(position, Vector3.Normalize(gravityDirection), gravityMagnitude, (int)flatIndex));
                }
            }
        }
    }

    public void CalculateMagneticField()
    {
        MagneticFieldVectors.Clear();
        float divisionLength = _simulationExtents.X / Divisions;
        float maxMagnitude = 0f;
        float minMagnitude = float.MaxValue;

        List<FieldVector> tempVectors = new List<FieldVector>();

        for (int x = 0; x < Divisions; x++)
        {
            for (int y = 0; y < Divisions; y++)
            {
                for (int z = 0; z < Divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, Divisions);
                    Vector3 totalMagneticField = Vector3.Zero;

                    foreach (var magnet in Magnets)
                    {
                        Vector3 magneticField = CalculateFieldAtPoint(magnet, position);
                        totalMagneticField += magneticField;
                    }

                    float magnitude = totalMagneticField.Length();
                    maxMagnitude = Math.Max(maxMagnitude, magnitude);
                    minMagnitude = Math.Min(minMagnitude, magnitude);

                    var flatIndex = x * _simulationExtents.Y * _simulationExtents.Z + y * _simulationExtents.Z + z;
                    tempVectors.Add(new FieldVector(position, totalMagneticField, magnitude, (int)flatIndex));
                }
            }
        }

        foreach (var vector in tempVectors)
        {
            float scaledMagnitude = ScaleMagnitude(vector.Magnitude, minMagnitude, maxMagnitude, divisionLength * 0.1f, divisionLength);
            Vector3 normalizedDirection = Vector3.Normalize(vector.Direction) * scaledMagnitude;

            MagneticFieldVectors.Add(new FieldVector(vector.Position, normalizedDirection, scaledMagnitude, vector.Index));
        }
    }

    private float ScaleMagnitude(float value, float min, float max, float newMin, float newMax)
    {
        if (max - min == 0) return newMin;
        return (value - min) / (max - min) * (newMax - newMin) + newMin;
    }

    private Vector3 CalculatePositionInSpace(int x, int y, int z, int divisions)
    {
        float stepSize = _simulationExtents.X / divisions;
        float offset = stepSize / 2.0f;

        return new Vector3(
            (x * stepSize + offset) - (_simulationExtents.X / 2.0f),
            (y * stepSize + offset) - (_simulationExtents.Y / 2.0f),
            (z * stepSize + offset) - (_simulationExtents.Z / 2.0f)
        );
    }

    public void InitializeTwoMagnets()
    {
        float baseHeight = -(_simulationExtents.Y / 4.0f);
        float gap = 1f;

        float stabilizingMagnetHeight = 1f;
        float fixedMagnetHeight = 1f;

        float stabilizingMagnetMass = .01f;
        float fixedMagnetMass = .01f;

        Vector3 targetPosition = new Vector3(0, baseHeight + stabilizingMagnetHeight / 2 + gap, 0);
        var levitatingMagnet = new Magnet(targetPosition, new Vector3(0, 1, 0), stabilizingMagnetHeight / 2, stabilizingMagnetHeight, 1.0f,
            stabilizingMagnetMass, MagnetType.Permanent, false);

        Vector3 fixedPosition = new Vector3(0, targetPosition.Y + stabilizingMagnetHeight / 2 + fixedMagnetHeight / 2 + gap, 0);
        var stabilizingMagnet = new Magnet(fixedPosition, new Vector3(0, 1, 0), fixedMagnetHeight / 2, fixedMagnetHeight, 1.0f, fixedMagnetMass,
            MagnetType.Permanent, true);

        AddMagnet(levitatingMagnet);
        AddMagnet(stabilizingMagnet);
    }
}