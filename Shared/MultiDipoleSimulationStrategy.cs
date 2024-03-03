using Animations.Shared.Models;
using System.Numerics;

namespace Animations.Shared;

public class MultiDipoleSimulationStrategy : ISimulationStrategy
{
    public void UpdateMagnetsPositions(List<Magnet> magnets, Vector3 gravity, float timeStep)
    {
        foreach (var target in magnets.Where(m => !m.IsFixed))
        {
            var totalForce = Vector3.Zero;
            var totalTorque = Vector3.Zero;

            foreach (var source in magnets.Where(m => m != target))
            {
                totalForce += source.ComputeForceOnMagnet(target);
                totalTorque += source.ComputeTorque(target);
            }

            var gravityForce = gravity * target.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / target.Mass;

            target.UpdateAngularVelocity(totalTorque, timeStep);
            target.Velocity += acceleration * timeStep;
            target.UpdatePositionAndAngularPosition(timeStep);
        }
    }
}