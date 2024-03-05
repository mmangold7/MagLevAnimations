using System.Numerics;
using Animations.Client.Contracts;
using Animations.Client.Extensions;
using Animations.Client.Models;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace Animations.Client.PhysicsSimulation;

public class BepuSimulationStrategy : ISimulationStrategy
{
    private BepuPhysics.Simulation _bepuSimulator;

    public BepuSimulationStrategy(Vector3 gravity) => _bepuSimulator = CreateBepuPhysicsSimulator(gravity);

    public void UpdateMagnetsPositions(List<Magnet> magnets, Vector3 gravity, float timeStep)
    {
        _bepuSimulator?.Timestep(timeStep);

        for (var i = 0; i < magnets.Count; i++)
        {
            for (var j = i + 1; j < magnets.Count; j++)
            {
                var magnet1 = magnets[i];
                var magnet2 = magnets[j];

                if (_bepuSimulator == null ||
                    !_bepuSimulator.Bodies.BodyExists(magnet1.PhysicsEngineBodyHandle) ||
                    !_bepuSimulator.Bodies.BodyExists(magnet2.PhysicsEngineBodyHandle)) continue;

                var bodyReference1 = _bepuSimulator.Bodies.GetBodyReference(magnet1.PhysicsEngineBodyHandle);
                var bodyReference2 = _bepuSimulator.Bodies.GetBodyReference(magnet2.PhysicsEngineBodyHandle);

                var forceOnMagnet1 = Fields.CalculateDipoleDipoleForce(magnet1, magnet2);
                bodyReference1.ApplyLinearImpulse(forceOnMagnet1 * timeStep);
                bodyReference2.ApplyLinearImpulse(-forceOnMagnet1 * timeStep);
            }

            UpdateMagnetFromBepuSimulation(magnets[i]);
        }
    }

    private void UpdateMagnetFromBepuSimulation(Magnet targetMagnet)
    {
        var bodyReference = _bepuSimulator.Bodies.GetBodyReference(targetMagnet.PhysicsEngineBodyHandle);
        targetMagnet.Position = bodyReference.Pose.Position;
        targetMagnet.Orientation = bodyReference.Pose.Orientation;
    }

    public static BepuPhysics.Simulation CreateBepuPhysicsSimulator(Vector3 gravity) =>
        BepuPhysics.Simulation.Create(
            new BufferPool(),
            new BepuNarrowPhaseCallbacks(),
            new BepuPoseIntegratorCallbacks(gravity),
            new SolveDescription(4, 1)
        );

    public void AddAllMagnetsToBepuPhysicsSimulator(List<Magnet> magnets)
    {
        foreach (var magnet in magnets) AddMagnetToBepuSimulator(magnet);

    }

    private void AddMagnetToBepuSimulator(Magnet magnet)
    {
        var cylinder = new Cylinder(magnet.Radius, magnet.Length);
        var cylinderIndex = _bepuSimulator.Shapes.Add(cylinder);
        var cylinderInertia = cylinder.ComputeInertia(magnet.Mass);

        var bodyDescription = BodyDescription.CreateDynamic(
            magnet.Position,
            cylinderInertia,
            new CollidableDescription(cylinderIndex, 0.1f),
            new BodyActivityDescription(0.01f));

        var bodyHandle = _bepuSimulator.Bodies.Add(bodyDescription);
        magnet.PhysicsEngineBodyHandle = bodyHandle;
    }

    public struct BepuNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(BepuPhysics.Simulation simulation) { }

        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin) => true;

        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;

        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = 1;
            pairMaterial.MaximumRecoveryVelocity = 2;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
            return true;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }

        public void Dispose() { }
    }

    public readonly struct BepuPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public bool AllowSubstepsForUnconstrainedBodies => false;

        public bool IntegrateVelocityForKinematics => false;

        private readonly Vector3 _gravity;

        public BepuPoseIntegratorCallbacks(Vector3 gravity)
        {
            _gravity = gravity;
        }

        public void Initialize(BepuPhysics.Simulation simulation) { }

        public void PrepareForIntegration(float dt) { }

        public void IntegrateVelocity(
            Vector<int> bodyIndices,
            Vector3Wide position,
            QuaternionWide orientation,
            BodyInertiaWide localInertia,
            Vector<int> integrationMask,
            int workerIndex,
            Vector<float> dt,
            ref BodyVelocityWide velocity)
        {
            Vector3Wide.Broadcast(_gravity, out var wideGravity);

            for (int i = 0; i < Vector<int>.Count; i++)
            {
                if (integrationMask[i] != 0)
                {
                    Vector3Wide.Scale(wideGravity, new Vector<float>(dt[i]), out var gravityDt);
                    Vector3Wide.Add(velocity.Linear, gravityDt, out velocity.Linear);
                }
            }
        }
    }
}