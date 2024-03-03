using Animations.Shared.Contracts;
using Animations.Shared.Extensions;
using Animations.Shared.Models;
using System.Numerics;

namespace Animations.Shared.Simulation;

public class SingleDipoleSimulationStrategy : ISimulationStrategy
{
    public void UpdateMagnetsPositions(List<Magnet> magnets, Vector3 gravity, float timeStep)
    {
        foreach (var target in magnets.Where(m => !m.IsFixed))
        {
            var totalForce = Vector3.Zero;
            var totalTorque = Vector3.Zero;

            foreach (var source in magnets.Where(m => m != target))
            {
                Vector3 forceFromSource = Fields.CalculateDipoleDipoleForce(target, source);
                totalForce += forceFromSource;

                Vector3 fieldAtTarget = source.CalculateMagneticFieldAtPoint(target.Position);

                Vector3 torqueOnTarget = target.CalculateTorque(fieldAtTarget);
                totalTorque += torqueOnTarget;
            }

            var gravityForce = gravity * target.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / target.Mass;
            target.Velocity += acceleration * timeStep;
            target.Position += target.Velocity * timeStep;

            target.UpdateAngularVelocity(totalTorque, timeStep);
            target.UpdateOrientation(timeStep);
        }
    }
}