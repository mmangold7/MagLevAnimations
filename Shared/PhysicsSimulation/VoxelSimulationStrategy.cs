using Animations.Shared.Contracts;
using Animations.Shared.Extensions;
using Animations.Shared.Models;
using System.Numerics;

namespace Animations.Shared.Simulation;

public class VoxelSimulationStrategy : ISimulationStrategy
{
    public void UpdateMagnetsPositions(List<Magnet> magnets, Vector3 gravity, float timeStep)
    {
        foreach (var targetMagnet in magnets.Where(m => !m.IsFixed))
        {
            var totalForce = Vector3.Zero;

            foreach (var sourceMagnet in magnets.Where(m => m != targetMagnet))
            {
                foreach (var targetVoxel in targetMagnet.Voxels)
                {
                    var voxelForce = Vector3.Zero;

                    foreach (var sourceVoxel in sourceMagnet.Voxels)
                        voxelForce += Fields.CalculateDipoleDipoleForce(targetVoxel, sourceVoxel);

                    totalForce += voxelForce;
                }
            }

            var gravityForce = gravity * targetMagnet.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / targetMagnet.Mass;
            targetMagnet.Velocity += acceleration * timeStep;
            targetMagnet.Position += targetMagnet.Velocity * timeStep;
        }
    }
}