using System.Numerics;
using Animations.Client.Contracts;
using Animations.Client.Extensions;
using Animations.Client.Models;

namespace Animations.Client.PhysicsSimulation;

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
                        voxelForce += Fields.CalculateDipoleDipoleForceVoxel(targetVoxel, sourceVoxel);

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