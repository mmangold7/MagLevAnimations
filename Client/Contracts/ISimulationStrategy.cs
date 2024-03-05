using System.Numerics;
using Animations.Client.Models;

namespace Animations.Client.Contracts;

public interface ISimulationStrategy
{
    public void UpdateMagnetsPositions(
        List<Magnet> magnets, Vector3 gravity, float timeStep);
}