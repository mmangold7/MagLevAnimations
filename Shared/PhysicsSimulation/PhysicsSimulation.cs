using Animations.Shared.Contracts;
using Animations.Shared.Enums;
using Animations.Shared.Extensions;
using Animations.Shared.Models;
using Animations.Shared.Models.Parameters;
using Animations.Shared.Simulation;
using System.Numerics;

namespace Animations.Shared.PhysicsSimulation;

public class PhysicsSimulation
{
    private const float MagnetHeight = 1f;
    private const float MagnetRadius = 0.5f;
    private const float MagnetMass = 0.01f;
    private const float GapBetweenMagnets = 0.1f;

    private float _timeStep;
    private Vector3 _gravity;
    private SimulationMode _simulationMode;
    public readonly List<Magnet> Magnets = new();

    public Vector3 Extents { get; set; }

    public PhysicsSimulation(Vector3 extents)
    {
        Extents = extents;
        var twoMagnets = InitializeTwoMagnets(extents);
        Magnets.AddRange(twoMagnets);
    }

    public void UpdateParameters(SimulationParameters parameters)
    {
        _timeStep = parameters.TimeStep;
        _gravity = parameters.Gravity;
        _simulationMode = parameters.SimulationMode;

        SetupVoxels(parameters);
    }

    private void SetupVoxels(SimulationParameters parameters)
    {
        if (parameters.SimulationMode == SimulationMode.VoxelBased)
            GenerateVoxels(parameters.SimulationExtents, parameters.Divisions, parameters.VoxelsPerDivision);
        else
            ClearAllVoxels();
    }

    public void GenerateVoxels(Vector3 simulationExtents, int divisions, int voxelsPerDivision)
    {
        var divisionLength = simulationExtents.X / divisions;
        var voxelLength = divisionLength / voxelsPerDivision;

        foreach (var magnet in Magnets)
            GenerateVoxelsForMagnet(magnet, voxelLength);
    }

    public void ClearAllVoxels()
    {
        foreach (var magnet in Magnets) magnet.Voxels.Clear();
    }

    private static void GenerateVoxelsForMagnet(Magnet magnet, float voxelLength)
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

    public void UpdateMagnetsPositionsBasedOnMode()
    {
        ISimulationStrategy currentStrategy = _simulationMode switch
        {
            SimulationMode.Bepu => new BepuSimulationStrategy(_gravity),
            SimulationMode.DipoleApproximation => new SingleDipoleSimulationStrategy(),
            SimulationMode.MultipleDipoles => new MultiDipoleSimulationStrategy(),
            SimulationMode.VoxelBased => new VoxelSimulationStrategy(),
            _ => throw new ArgumentOutOfRangeException()
        };

        Profiling.RunWithClockingLog(() => currentStrategy.UpdateMagnetsPositions(Magnets, _gravity, _timeStep));
    }

    private bool IsColliding(Magnet magnet1, Magnet magnet2)
    {
        var line1Start = magnet1.Position;
        var line1End = magnet1.Position + Vector3.Transform(new Vector3(0, 0, magnet1.Length), magnet1.Orientation.ToMatrix());
        var line2Start = magnet2.Position;
        var line2End = magnet2.Position + Vector3.Transform(new Vector3(0, 0, magnet2.Length), magnet2.Orientation.ToMatrix());

        var distance = Fields.ShortestDistanceBetweenLines(line1Start, line1End, line2Start, line2End);

        return distance < magnet1.Radius + magnet2.Radius;
    }

    private void ResolveCollision(Magnet magnet1, Magnet magnet2)
    {
        var collisionNormal = Vector3.Normalize(magnet2.Position - magnet1.Position);

        var overlap = magnet1.Radius + magnet2.Radius - Vector3.Distance(magnet1.Position, magnet2.Position);
        var separationVector = overlap * 0.5f * collisionNormal;
        magnet1.Position -= separationVector;
        magnet2.Position += separationVector;

        var relativeVelocity = magnet2.Velocity - magnet1.Velocity;

        var velocityAlongNormal = Vector3.Dot(relativeVelocity, collisionNormal);

        if (velocityAlongNormal > 0) return;

        const float restitution = 0.8f;
        var j = -(1 + restitution) * velocityAlongNormal;
        j /= 1 / magnet1.Mass + 1 / magnet2.Mass;

        var impulse = j * collisionNormal;
        magnet1.Velocity -= impulse / magnet1.Mass;
        magnet2.Velocity += impulse / magnet2.Mass;
    }

    public void DetectAndResolveCollisions()
    {
        if (_simulationMode == SimulationMode.Bepu) return;

        foreach (var magnet in Magnets)
        {
            DetectAndResolveCollisionWithGround(magnet);
        }

        for (var i = 0; i < Magnets.Count; i++)
        {
            for (var j = i + 1; j < Magnets.Count; j++)
            {
                var magnet1 = Magnets[i];
                var magnet2 = Magnets[j];

                if (IsColliding(magnet1, magnet2)) ResolveCollision(magnet1, magnet2);
            }
        }
    }

    private void DetectAndResolveCollisionWithGround(Magnet magnet)
    {
        float groundLevel = -Extents.Y / 2;
        float magnetBottom = magnet.Position.Y - magnet.Length / 2;

        if (magnetBottom < groundLevel)
        {
            float newYPosition = groundLevel + magnet.Length / 2;
            magnet.Position = new Vector3(magnet.Position.X, newYPosition, magnet.Position.Z);

            float restitution = 0.5f;
            float newVelocityY = -magnet.Velocity.Y * restitution;
            magnet.Velocity = new Vector3(magnet.Velocity.X, newVelocityY, magnet.Velocity.Z);

            float friction = 0.8f;
            magnet.Velocity = new Vector3(magnet.Velocity.X * friction, magnet.Velocity.Y, magnet.Velocity.Z * friction);
        }
    }

    private static Vector3 GetBaseMagnetPosition(Vector3 simulationExtents)
    {
        var baseHeight = -(simulationExtents.Y / 4.0f);
        return new Vector3(0, baseHeight, 0);
    }

    private static Vector3 GetLevitatingMagnetPosition(Vector3 simulationExtents)
    {
        var basePosition = GetBaseMagnetPosition(simulationExtents);
        return basePosition with { Y = basePosition.Y + MagnetHeight / 2 + GapBetweenMagnets };
    }

    private static Vector3 GetFixedMagnetPosition(Vector3 simulationExtents)
    {
        var levitatingPosition = GetLevitatingMagnetPosition(simulationExtents);

        return new Vector3(
            levitatingPosition.X + 0.3f,
            levitatingPosition.Y + MagnetHeight + GapBetweenMagnets,
            levitatingPosition.Z + 0.3f);
    }

    public static List<Magnet> InitializeTwoMagnets(Vector3 simulationExtents)
    {
        var twoMagnets = new List<Magnet>();

        var levitatingPosition = GetLevitatingMagnetPosition(simulationExtents);
        var levitatingMagnet = new Magnet(levitatingPosition, Vector3.UnitY,
            MagnetMass, MagnetRadius, MagnetHeight, false, MagnetType.Permanent);

        var fixedPosition = GetFixedMagnetPosition(simulationExtents);
        var stabilizingMagnet = new Magnet(fixedPosition, Vector3.UnitY,
            MagnetMass, MagnetRadius, MagnetHeight, true, MagnetType.Permanent);

        twoMagnets.Add(levitatingMagnet);
        twoMagnets.Add(stabilizingMagnet);

        return twoMagnets;
    }
}