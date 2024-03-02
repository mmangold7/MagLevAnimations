using Animations.Shared.Enums;
using Animations.Shared.Extensions;
using Animations.Shared.Models;
using Animations.Shared.Models.Parameters;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace Animations.Shared;

public class MagneticSimulationManager
{
    private const float MagnetHeight = 1f;
    private const float MagnetRadius = 0.5f;
    private const float MagnetMass = 0.01f;
    private const float GapBetweenMagnets = 0.1f;
    private bool _showGravityField;
    private bool _showMagneticField;
    private int _divisions;
    private int _voxelsPerDivision;
    private float _timeStep;
    private Vector3 _gravity;
    private Vector3 _simulationExtents;
    private SimulationMode _currentMode;
    private Simulation? _physicsSimulator;
    private readonly List<Magnet> _magnets = new();
    private readonly List<FieldVector> _gravityFieldVectors = new();
    private readonly List<FieldVector> _magneticFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousMagneticFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousGravityFieldVectors = new();

    private Vector3 GetBaseMagnetPosition()
    {
        var baseHeight = -(_simulationExtents.Y / 4.0f);
        return new Vector3(0, baseHeight, 0);
    }

    private Vector3 GetLevitatingMagnetPosition()
    {
        var basePosition = GetBaseMagnetPosition();
        return basePosition with { Y = basePosition.Y + MagnetHeight / 2 + GapBetweenMagnets };
    }

    private Vector3 GetFixedMagnetPosition()
    {
        var levitatingPosition = GetLevitatingMagnetPosition();
        return new Vector3(levitatingPosition.X + 0.3f, levitatingPosition.Y + MagnetHeight + GapBetweenMagnets, levitatingPosition.Z + 0.3f);
    }

    public MagneticSimulationManager(SimulationParameters parameters)
    {
        SetSimulationParameters(parameters);
    }

    #region Consumer get/set

    public SimulationStateForVisualization GetSimulationState()
    {
        return new SimulationStateForVisualization
        {
            Magnets = _magnets.Select(ConvertMagnetToJsObject).ToArray(),
            GravityFieldData = _showGravityField ? _gravityFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MagneticFieldData = _showMagneticField ? _magneticFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MiddleSliceOfFieldVectors = Get2DMiddleSliceOfFieldVectors().Select(ConvertFieldToJsObject).ToArray()
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

    public void InitializeTwoMagnets()
    {
        var levitatingPosition = GetLevitatingMagnetPosition();
        var levitatingMagnet = new Magnet(levitatingPosition, Vector3.UnitY,
            MagnetMass, MagnetRadius, MagnetHeight, false, MagnetType.Permanent);

        var fixedPosition = GetFixedMagnetPosition();
        var stabilizingMagnet = new Magnet(fixedPosition, Vector3.UnitY,
            MagnetMass, MagnetRadius, MagnetHeight, true, MagnetType.Permanent);

        AddMagnet(levitatingMagnet);
        AddMagnet(stabilizingMagnet);
    }

    private void AddMagnet(Magnet magnet)
    {
        _magnets.Add(magnet);

        if (_currentMode == SimulationMode.Bepu && _physicsSimulator != null)
        {
            var cylinder = new Cylinder(magnet.Radius, magnet.Length);
            var cylinderIndex = _physicsSimulator.Shapes.Add(cylinder);

            var bodyDescription = BodyDescription.CreateDynamic(
                new RigidPose(magnet.Position, Quaternion.Identity),
                new BodyInertia { InverseMass = 1f / magnet.Mass },
                new CollidableDescription(cylinderIndex, 0.1f),
                new BodyActivityDescription(0.01f));

            var bodyHandle = _physicsSimulator.Bodies.Add(bodyDescription);
            magnet.PhysicsEngineBodyHandle = bodyHandle;
        }
    }

    private void SetSimulationParameters(SimulationParameters parameters)
    {
        _timeStep = parameters.TimeStep;
        _gravity = parameters.Gravity;
        _simulationExtents = parameters.SimulationExtents;
        _divisions = parameters.Divisions;
        _voxelsPerDivision = parameters.VoxelsPerDivision;
        _showGravityField = parameters.ShowGravityField;
        _showMagneticField = parameters.ShowMagneticField;

        if (_currentMode != parameters.Mode)
        {
            _currentMode = parameters.Mode;
            SetupMagnetSimulationForMode(_currentMode);
        }
    }

    #endregion

    #region Mode-specific setup

    private void SetupMagnetSimulationForMode(SimulationMode mode)
    {
        switch (mode)
        {
            case SimulationMode.VoxelBased:
                GenerateVoxels();
                break;
            case SimulationMode.DipoleApproximation:
                ClearVoxels();
                break;
            case SimulationMode.MultipleDipoles:
                ClearVoxels();
                break;
            case SimulationMode.Bepu:
                ClearVoxels();
                _physicsSimulator = CreateBepuPhysicsSimulator();
                AddMagnetsToBepuPhysicsSimulator(_physicsSimulator);
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    private void GenerateVoxels()
    {
        var divisionLength = _simulationExtents.X / _divisions;
        var voxelLength = divisionLength / _voxelsPerDivision;

        foreach (var magnet in _magnets)
            GenerateVoxelsForMagnet(magnet, voxelLength);
    }

    private void ClearVoxels()
    {
        foreach (var magnet in _magnets)
            magnet.VoxelsForApproximation.Clear();
    }

    private void AddMagnetsToBepuPhysicsSimulator(Simulation bepuSim)
    {
        foreach (var magnet in _magnets)
        {
            var cylinder = new Cylinder(MagnetRadius, MagnetHeight);
            var cylinderIndex = bepuSim.Shapes.Add(cylinder);
            var cylinderInertia = cylinder.ComputeInertia(MagnetMass);

            var bodyDescription = BodyDescription.CreateDynamic(
                magnet.Position,
                cylinderInertia,
                new CollidableDescription(cylinderIndex, 0.1f),
                new BodyActivityDescription(0.01f));

            var bodyHandle = bepuSim.Bodies.Add(bodyDescription);
            magnet.PhysicsEngineBodyHandle = bodyHandle;
        }
    }

    private Simulation CreateBepuPhysicsSimulator()
    {
        var bufferPool = new BufferPool();
        var narrowPhaseCallbacks = new NarrowPhaseCallbacks();
        var poseIntegratorCallbacks = new PoseIntegratorCallbacks(_gravity);
        var solveDescription = new SolveDescription(4, 1);
        ITimestepper? timeStepper = null;
        SimulationAllocationSizes? initialAllocationSizes = null;

        return Simulation.Create(
            bufferPool,
            narrowPhaseCallbacks,
            poseIntegratorCallbacks,
            solveDescription,
            timeStepper,
            initialAllocationSizes
        );
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

                    magnet.VoxelsForApproximation.Add(new Voxel(voxelPosition, voxelMagneticMoment, voxelLength));
                }
            }
        }
    }

    public struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation) { }

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

    public readonly struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public bool AllowSubstepsForUnconstrainedBodies => false;

        public bool IntegrateVelocityForKinematics => false;

        private readonly Vector3 _gravity;

        public PoseIntegratorCallbacks(Vector3 gravity)
        {
            this._gravity = gravity;
        }

        public void Initialize(Simulation simulation) { }

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

    #endregion

    public void UpdateSimulation(SimulationParameters parameters)
    {
        SetSimulationParameters(parameters);
        UpdateMagnetsPositionsBasedOnMode(parameters);
        DetectAndResolveCollisions();
        RecalculateFields();
    }

    #region Mechanics Calculations

    private void UpdateMagnetsPositionsBasedOnMode(SimulationParameters parameters)
    {
        switch (_currentMode)
        {
            case SimulationMode.MultipleDipoles:
                ProfilingExtensions.RunWithClockingLog(UpdateMagnetsPositionsUsingMultipleDipoleApproximation);
                break;
            case SimulationMode.DipoleApproximation:
                ProfilingExtensions.RunWithClockingLog(UpdateMagnetsPositionsUsingDipoleApproximation);
                break;
            case SimulationMode.VoxelBased:
                ProfilingExtensions.RunWithClockingLog(UpdateMagnetsPositionsUsingVoxelApproximation);
                break;
            case SimulationMode.Bepu:
                _physicsSimulator?.Timestep(parameters.TimeStep);
                ProfilingExtensions.RunWithClockingLog(ApplyBepuMagneticForces);
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    private void UpdateMagnetsPositionsUsingMultipleDipoleApproximation()
    {
        foreach (var target in _magnets.Where(m => !m.IsFixed))
        {
            var totalForce = Vector3.Zero;
            var totalTorque = Vector3.Zero;

            foreach (var source in _magnets.Where(m => m != target))
            {
                totalForce += source.ComputeForceOnMagnet(target);
                totalTorque += source.ComputeTorque(target);
            }

            var gravityForce = _gravity * target.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / target.Mass;

            target.UpdateAngularVelocity(totalTorque, _timeStep);
            target.Velocity += acceleration * _timeStep;
            target.UpdatePositionAndAngularPosition(_timeStep);
        }
    }

    private void UpdateMagnetsPositionsUsingDipoleApproximation()
    {
        foreach (var target in _magnets.Where(m => !m.IsFixed))
        {
            var totalForce = Vector3.Zero;
            var totalTorque = Vector3.Zero;

            foreach (var source in _magnets.Where(m => m != target))
            {
                Vector3 forceFromSource = CalculateDipoleDipoleForce(target, source);
                totalForce += forceFromSource;

                Vector3 fieldAtTarget = source.CalculateMagneticFieldAtPoint(target.Position);

                Vector3 torqueOnTarget = target.CalculateTorque(fieldAtTarget);
                totalTorque += torqueOnTarget;
            }

            var gravityForce = _gravity * target.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / target.Mass;
            target.Velocity += acceleration * _timeStep;
            target.Position += target.Velocity * _timeStep;

            target.UpdateAngularVelocity(totalTorque, _timeStep);
            target.UpdateOrientation(_timeStep);
        }
    }

    public List<FieldVector> Get2DMiddleSliceOfFieldVectors()
    {
        var middleY = _simulationExtents.Y / 2;
        var sliceFieldVectors = new List<FieldVector>();

        var closestY = _magneticFieldVectors?
            .MinBy(vector => Math.Abs(vector.Position.Y - middleY))
            ?.Position.Y;

        if (_magneticFieldVectors != null)
            sliceFieldVectors.AddRange(_magneticFieldVectors
                .Where(vector => Math.Abs((float)(vector.Position.Y - closestY)) < 1e-6));

        return sliceFieldVectors;
    }

    private void UpdateMagnetsPositionsUsingVoxelApproximation()
    {
        foreach (var targetMagnet in _magnets.Where(m => !m.IsFixed))
        {
            Vector3 totalForce = Vector3.Zero;

            foreach (var sourceMagnet in _magnets.Where(m => m != targetMagnet))
            {
                foreach (var targetVoxel in targetMagnet.VoxelsForApproximation)
                {
                    Vector3 voxelForce = Vector3.Zero;

                    foreach (var sourceVoxel in sourceMagnet.VoxelsForApproximation)
                    {
                        voxelForce += CalculateDipoleDipoleForce(targetVoxel, sourceVoxel);
                    }

                    totalForce += voxelForce;
                }
            }

            var gravityForce = _gravity * targetMagnet.Mass;
            totalForce += gravityForce;
            var acceleration = totalForce / targetMagnet.Mass;
            targetMagnet.Velocity += acceleration * _timeStep;
            targetMagnet.Position += targetMagnet.Velocity * _timeStep;
        }
    }

    private Vector3 CalculateDipoleDipoleForce(Magnet target, Magnet source)
    {
        Vector3 r = target.Position - source.Position;
        float rMagnitude = r.Length();
        Vector3 rHat = r / rMagnitude;

        float mu0 = 4 * (float)Math.PI * 1e-1f;
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

    private void UpdateMagnetFromBepuSimulation(Magnet targetMagnet)
    {
        if (_physicsSimulator == null) return;
        var bodyReference = _physicsSimulator.Bodies.GetBodyReference(targetMagnet.PhysicsEngineBodyHandle);
        targetMagnet.Position = bodyReference.Pose.Position;
        targetMagnet.Orientation = bodyReference.Pose.Orientation;
    }

    public void ApplyBepuMagneticForces()
    {
        for (var i = 0; i < _magnets.Count; i++)
        {
            for (var j = i + 1; j < _magnets.Count; j++)
            {
                var magnet1 = _magnets[i];
                var magnet2 = _magnets[j];

                if (_physicsSimulator != null &&
                    _physicsSimulator.Bodies.BodyExists(magnet1.PhysicsEngineBodyHandle) &&
                    _physicsSimulator.Bodies.BodyExists(magnet2.PhysicsEngineBodyHandle))
                {
                    var bodyReference1 = _physicsSimulator.Bodies.GetBodyReference(magnet1.PhysicsEngineBodyHandle);
                    var bodyReference2 = _physicsSimulator.Bodies.GetBodyReference(magnet2.PhysicsEngineBodyHandle);

                    var forceOnMagnet1 = CalculateDipoleDipoleForce(magnet1, magnet2);
                    bodyReference1.ApplyLinearImpulse(forceOnMagnet1 * _timeStep);
                    bodyReference2.ApplyLinearImpulse(-forceOnMagnet1 * _timeStep);
                }
            }
            UpdateMagnetFromBepuSimulation(_magnets[i]);
        }
    }

    private void DetectAndResolveCollisions()
    {
        if (_currentMode == SimulationMode.Bepu) return;

        for (int i = 0; i < _magnets.Count; i++)
        {
            for (int j = i + 1; j < _magnets.Count; j++)
            {
                Magnet magnet1 = _magnets[i];
                Magnet magnet2 = _magnets[j];

                if (IsColliding(magnet1, magnet2))
                {
                    ResolveCollision(magnet1, magnet2);
                }
            }
        }
    }

    private bool IsColliding(Magnet magnet1, Magnet magnet2)
    {
        Vector3 line1Start = magnet1.Position;
        Vector3 line1End = magnet1.Position + Vector3.Transform(new Vector3(0, 0, magnet1.Length), magnet1.Orientation.ToMatrix());
        Vector3 line2Start = magnet2.Position;
        Vector3 line2End = magnet2.Position + Vector3.Transform(new Vector3(0, 0, magnet2.Length), magnet2.Orientation.ToMatrix());

        var distance = SimulationExtensions.ShortestDistanceBetweenLines(line1Start, line1End, line2Start, line2End);

        return distance < (magnet1.Radius + magnet2.Radius);
    }

    private void ResolveCollision(Magnet magnet1, Magnet magnet2)
    {
        // Compute the normal of the collision
        Vector3 collisionNormal = Vector3.Normalize(magnet2.Position - magnet1.Position);

        // Separate the magnets to eliminate overlap
        float overlap = (magnet1.Radius + magnet2.Radius) - Vector3.Distance(magnet1.Position, magnet2.Position);
        Vector3 separationVector = overlap * 0.5f * collisionNormal;
        magnet1.Position -= separationVector;
        magnet2.Position += separationVector;

        // Compute the relative velocity
        Vector3 relativeVelocity = magnet2.Velocity - magnet1.Velocity;

        // Compute the velocity along the normal
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, collisionNormal);

        // Do not resolve if velocities are separating
        if (velocityAlongNormal > 0) return;

        // Compute restitution (elasticity) and impulse scalar
        float restitution = 0.8f; // Assume some elasticity
        float j = -(1 + restitution) * velocityAlongNormal;
        j /= (1 / magnet1.Mass) + (1 / magnet2.Mass);

        // Apply impulse
        Vector3 impulse = j * collisionNormal;
        magnet1.Velocity -= impulse / magnet1.Mass;
        magnet2.Velocity += impulse / magnet2.Mass;
    }

    #endregion

    #region Field Calculations

    public void RecalculateFields()
    {
        if (_showGravityField) ProfilingExtensions.RunWithClockingLog(CalculateGravityField);
        if (_showMagneticField) ProfilingExtensions.RunWithClockingLog(CalculateMagneticField);
    }

    private void CalculateGravityField()
    {
        _gravityFieldVectors.Clear();
        Vector3 gravityDirection = _gravity == Vector3.Zero
            ? Vector3.Zero
            : Vector3.Normalize(_gravity);

        float divisionLength = _simulationExtents.X / _divisions;
        float gravityMagnitude = Math.Min(_gravity.Y, divisionLength);
        float updateThreshold = 0.1f;

        List<FieldVector> updatedVectors = new List<FieldVector>();

        for (var x = 0; x < _divisions; x++)
        {
            for (var y = 0; y < _divisions; y++)
            {
                for (var z = 0; z < _divisions; z++)
                {
                    var position = CalculatePositionInSpace(x, y, z, _divisions);
                    var flatIndex = x * (int)_simulationExtents.Y * (int)_simulationExtents.Z + y * (int)_simulationExtents.Z + z;
                    var newVector = new FieldVector(position, gravityDirection, gravityMagnitude, flatIndex);

                    if (_previousGravityFieldVectors.ContainsKey(flatIndex))
                    {
                        var previousVector = _previousGravityFieldVectors[flatIndex];
                        if (!(Math.Abs(newVector.Magnitude - previousVector.Magnitude) > updateThreshold)) continue;
                        updatedVectors.Add(newVector);
                        _previousGravityFieldVectors[flatIndex] = newVector;
                    }
                    else
                    {
                        updatedVectors.Add(newVector);
                        _previousGravityFieldVectors.Add(flatIndex, newVector);
                    }
                }
            }
        }

        foreach (var vector in updatedVectors)
        {
            _gravityFieldVectors.Add(vector);
        }
    }

    private void CalculateMagneticField()
    {
        _magneticFieldVectors.Clear();
        float divisionLength = _simulationExtents.X / _divisions;
        float maxMagnitude = 0f;
        float minMagnitude = float.MaxValue;
        float updateThreshold = 0.1f;

        List<FieldVector> updatedVectors = new List<FieldVector>();

        for (int x = 0; x < _divisions; x++)
        {
            for (int y = 0; y < _divisions; y++)
            {
                for (int z = 0; z < _divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, _divisions);
                    Vector3 totalMagneticField = Vector3.Zero;

                    foreach (var magnet in _magnets)
                    {
                        Vector3 magneticField = CalculateFieldAtPoint(magnet, position);
                        totalMagneticField += magneticField;
                    }

                    float magnitude = totalMagneticField.Length();
                    maxMagnitude = Math.Max(maxMagnitude, magnitude);
                    minMagnitude = Math.Min(minMagnitude, magnitude);

                    int flatIndex = x * (int)_simulationExtents.Y * (int)_simulationExtents.Z + y * (int)_simulationExtents.Z + z;
                    FieldVector newVector = new FieldVector(position, totalMagneticField, magnitude, flatIndex);

                    if (_previousMagneticFieldVectors.ContainsKey(flatIndex))
                    {
                        FieldVector previousVector = _previousMagneticFieldVectors[flatIndex];
                        if ((newVector.Direction - previousVector.Direction).Length() / previousVector.Direction.Length() > updateThreshold)
                        {
                            updatedVectors.Add(newVector);
                            _previousMagneticFieldVectors[flatIndex] = newVector;
                        }
                    }
                    else
                    {
                        updatedVectors.Add(newVector);
                        _previousMagneticFieldVectors.Add(flatIndex, newVector);
                    }
                }
            }
        }

        foreach (var vector in updatedVectors)
        {
            float scaledMagnitude = ScaleMagnitude(vector.Magnitude, minMagnitude, maxMagnitude, divisionLength * 0.1f, divisionLength);
            Vector3 normalizedDirection = Vector3.Normalize(vector.Direction) * scaledMagnitude; // Ensure this line is present and correct

            _magneticFieldVectors.Add(new FieldVector(vector.Position, normalizedDirection, scaledMagnitude, vector.Index));
        }
    }

    private Vector3 CalculateFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        switch (_currentMode)
        {
            case SimulationMode.DipoleApproximation:
                return CalculateSingleDipoleFieldAtPoint(sourceMagnet, point);
            case SimulationMode.VoxelBased:
                var totalField = Vector3.Zero;
                foreach (var voxel in sourceMagnet.VoxelsForApproximation)
                    totalField += CalculateFieldFromVoxel(voxel, point);
                return totalField;
            case SimulationMode.MultipleDipoles:
                return sourceMagnet.ComputeMultipleDipoleFieldAtPoint(point);
            case SimulationMode.Bepu:
                return CalculateSingleDipoleFieldAtPoint(sourceMagnet, point);
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    private static Vector3 CalculateSingleDipoleFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        Vector3 r = point - sourceMagnet.Position;
        var mu0 = 4 * (float)Math.PI * 1e-7f;
        var rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        return (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                     (3 * Vector3.Dot(sourceMagnet.Magnetization, r) * r -
                      sourceMagnet.Magnetization * rMagnitude * rMagnitude);
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

    private static float ScaleMagnitude(float value, float min, float max, float minScale, float maxScale)
    {
        var range = max - min;
        if (range == 0) return minScale;
        return (value - min) / (range) * (maxScale - minScale) + minScale;
    }

    private Vector3 CalculatePositionInSpace(int x, int y, int z, int divisions)
    {
        var stepSize = _simulationExtents.X / divisions;
        var offset = stepSize / 2.0f;

        return new Vector3(
            (x * stepSize + offset) - (_simulationExtents.X / 2.0f),
            (y * stepSize + offset) - (_simulationExtents.Y / 2.0f),
            (z * stepSize + offset) - (_simulationExtents.Z / 2.0f)
        );
    }

    #endregion
}