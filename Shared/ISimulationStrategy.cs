using Animations.Shared.Models;
using System.Numerics;

namespace Animations.Shared;

public interface ISimulationStrategy
{
    public void UpdateMagnetsPositions(
        List<Magnet> magnets, Vector3 gravity, float timeStep);
}