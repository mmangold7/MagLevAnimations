using Animations.Shared.Enums;
using Animations.Shared.Models;
using System.Numerics;

namespace Animations.Shared;

public class MagneticManager
{
    private const float MagnetHeight = 1f;
    private const float MagnetRadius = 0.5f;
    private const float MagnetMass = 0.01f;
    private const float GapBetweenMagnets = 0.1f;

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