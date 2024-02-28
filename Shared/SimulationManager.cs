using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;

namespace Animations.Shared;

public class SimulationManager
{
    private bool _showGravityField;
    private bool _showMagneticField;
    private int _divisions;
    private int _voxelsPerDivision;
    private float _timeStep;
    private Vector3 _gravity;
    private Vector3 _simulationExtents;
    private SimulationMode _currentMode;
    private readonly Simulation _simulation;

    private readonly List<Magnet> _magnets = new();
    private readonly List<FieldVector> _gravityFieldVectors = new();
    private readonly List<FieldVector> _magneticFieldVectors = new();

    private readonly Dictionary<int, FieldVector> _previousMagneticFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousGravityFieldVectors = new();

    //public api
    public SimulationManager(SimulationParameters parameters)
    {
        var bufferPool = new BufferPool();
        var narrowPhaseCallbacks = new NarrowPhaseCallbacks();
        var poseIntegratorCallbacks = new PoseIntegratorCallbacks(new Vector3(0, parameters.Gravity.Y, 0));
        var solveDescription = new SolveDescription(4, 1);
        ITimestepper timestepper = null;
        SimulationAllocationSizes? initialAllocationSizes = null;

        _simulation = Simulation.Create(
            bufferPool,
            narrowPhaseCallbacks,
            poseIntegratorCallbacks,
            solveDescription,
            timestepper,
            initialAllocationSizes
        );

        var cylinder = new Cylinder(1f, 0.5f); // 1 meter tall, 0.5 meters in radius.
        var cylinderInertia = cylinder.ComputeInertia(10); // 10 kg mass.
        var cylinderIndex = _simulation.Shapes.Add(cylinder);

        var bodyDescription1 = BodyDescription.CreateDynamic(new Vector3(0, 5, 0), cylinderInertia, new CollidableDescription(cylinderIndex, 0.1f), new BodyActivityDescription(0.01f));
        _simulation.Bodies.Add(bodyDescription1);

        var bodyDescription2 = BodyDescription.CreateDynamic(new Vector3(0, 10, 0), cylinderInertia, new CollidableDescription(cylinderIndex, 0.1f), new BodyActivityDescription(0.01f));
        _simulation.Bodies.Add(bodyDescription2);

        SetSimulationParameters(parameters);
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

    public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public bool AllowSubstepsForUnconstrainedBodies => false;

        public bool IntegrateVelocityForKinematics => false;

        private Vector3 gravity;

        public PoseIntegratorCallbacks(Vector3 gravity)
        {
            this.gravity = gravity;
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
            Vector3Wide.Broadcast(gravity, out var wideGravity);

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
    public void InitializeTwoMagnets()
    {
        float baseHeight = -(_simulationExtents.Y / 4.0f);
        float gap = 0.2f;

        float stabilizingMagnetHeight = 1f;
        float fixedMagnetHeight = 1f;

        float stabilizingMagnetMass = .01f;
        float fixedMagnetMass = .01f;

        Vector3 targetPosition = new Vector3(0, baseHeight + stabilizingMagnetHeight / 2 + gap, 0);
        var levitatingMagnet = new Magnet(targetPosition, new Vector3(0, 1, 0), stabilizingMagnetHeight / 2, stabilizingMagnetHeight, 1.0f,
            stabilizingMagnetMass, MagnetType.Permanent, false);

        Vector3 fixedPosition = new Vector3(0.3f, targetPosition.Y + stabilizingMagnetHeight / 2 + fixedMagnetHeight / 2 + gap, 0);
        var stabilizingMagnet = new Magnet(fixedPosition, new Vector3(0, 1, 0), fixedMagnetHeight / 2, fixedMagnetHeight, 1.0f, fixedMagnetMass,
            MagnetType.Permanent, true);

        AddMagnet(levitatingMagnet);
        AddMagnet(stabilizingMagnet);
    }
    public void UpdateSimulation(SimulationParameters parameters)
    {
        SetSimulationParameters(parameters);
        _simulation.Timestep(parameters.TimeStep);
        ApplyMagneticForces();

        switch (_currentMode)
        {
            case SimulationMode.DipoleApproximation:
                {
                    Profiling.RunWithClockingLog(UpdateMagnetPositionsUsingDipoleApproximation);
                    break;
                }
            case SimulationMode.VoxelBased:
                {
                    Profiling.RunWithClockingLog(UpdateMagnetsPositionsUsingVoxelApproximation);
                    break;
                }
        }

        RecalculateFields();
    }
    public void ApplyMagneticForces()
    {
        for (int i = 0; i < _magnets.Count; i++)
        {
            for (int j = i + 1; j < _magnets.Count; j++)
            {
                var magnet1 = _magnets[i];
                var magnet2 = _magnets[j];

                if (_simulation.Bodies.BodyExists(magnet1.PhysicsEngineBodyHandle) &&
                    _simulation.Bodies.BodyExists(magnet2.PhysicsEngineBodyHandle))
                {
                    var bodyReference1 = _simulation.Bodies.GetBodyReference(magnet1.PhysicsEngineBodyHandle);
                    var bodyReference2 = _simulation.Bodies.GetBodyReference(magnet2.PhysicsEngineBodyHandle);

                    Vector3 forceOnMagnet1 = CalculateDipoleDipoleForce(magnet1, magnet2);
                    bodyReference1.ApplyLinearImpulse(forceOnMagnet1 * _timeStep);
                    bodyReference2.ApplyLinearImpulse(-forceOnMagnet1 * _timeStep);
                }
            }
        }
    }
    public void RecalculateFields()
    {
        if (_showGravityField) Profiling.RunWithClockingLog(CalculateGravityField);
        if (_showMagneticField) Profiling.RunWithClockingLog(CalculateMagneticField);
    }
    public SimulationState GetSimulationState()
    {
        return new SimulationState
        {
            Magnets = _magnets.Select(ConvertMagnetToJsObject).ToArray(),
            GravityFieldData = _showGravityField ? _gravityFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MagneticFieldData = _showMagneticField ? _magneticFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null
        };
    }

    //helpers for public api
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
            switch (_currentMode)
            {
                case SimulationMode.VoxelBased:
                    {
                        var divisionLength = _simulationExtents.X / _divisions;
                        var voxelLength = divisionLength / _voxelsPerDivision;

                        foreach (var magnet in _magnets)
                            GenerateVoxelsForMagnet(magnet, voxelLength);
                        break;
                    }
                case SimulationMode.DipoleApproximation:
                    {
                        foreach (var magnet in _magnets)
                            magnet.Voxels.Clear();
                        break;
                    }
            }
        }
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
    private void AddMagnet(Magnet magnet)
    {
        _magnets.Add(magnet);

        var cylinder = new Cylinder(magnet.Radius, magnet.Length);
        var cylinderIndex = _simulation.Shapes.Add(cylinder);

        var bodyDescription = BodyDescription.CreateDynamic(
            new RigidPose(magnet.Position, Quaternion.Identity),
            new BodyInertia { InverseMass = 1f / magnet.Mass },
            new CollidableDescription(cylinderIndex, 0.1f),
            new BodyActivityDescription(0.01f));

        var bodyHandle = _simulation.Bodies.Add(bodyDescription);
        magnet.PhysicsEngineBodyHandle = bodyHandle;
    }
    private void GenerateVoxelsForMagnet(Magnet magnet, float voxelLength)
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

    //frequently calculated i.e. every update/frame
    private void UpdateMagnetsPositionsUsingVoxelApproximation()
    {
        foreach (var targetMagnet in _magnets.Where(m => !m.IsFixed))
        {
            Vector3 totalForce = Vector3.Zero;

            foreach (var sourceMagnet in _magnets.Where(m => m != targetMagnet))
            {
                UpdateMagnetFromBepuSimulation(targetMagnet);

                foreach (var targetVoxel in targetMagnet.Voxels)
                {
                    Vector3 voxelForce = Vector3.Zero;

                    foreach (var sourceVoxel in sourceMagnet.Voxels)
                    {
                        voxelForce += CalculateDipoleDipoleForce(targetVoxel, sourceVoxel);
                    }

                    totalForce += voxelForce;
                }
            }

            Vector3 gravityForce = _gravity * targetMagnet.Mass;
            totalForce += gravityForce;
            Vector3 acceleration = totalForce / targetMagnet.Mass;
            targetMagnet.Velocity += acceleration * _timeStep;
            targetMagnet.Position += targetMagnet.Velocity * _timeStep;
        }
    }
    private void UpdateMagnetFromBepuSimulation(Magnet targetMagnet)
    {
        var bodyReference = _simulation.Bodies.GetBodyReference(targetMagnet.PhysicsEngineBodyHandle);
        targetMagnet.Position = bodyReference.Pose.Position;
        targetMagnet.Orientation = bodyReference.Pose.Orientation;
    }
    private void UpdateMagnetPositionsUsingDipoleApproximation()
    {
        foreach (var target in _magnets.Where(m => !m.IsFixed))
        {
            UpdateMagnetFromBepuSimulation(target);

            Vector3 totalForce = Vector3.Zero;

            foreach (var source in _magnets.Where(m => m != target))
            {
                Vector3 forceFromSource = CalculateDipoleDipoleForce(target, source);
                totalForce += forceFromSource;
            }

            Vector3 gravityForce = _gravity * target.Mass;
            totalForce += gravityForce;
            Vector3 acceleration = totalForce / target.Mass;
            target.Velocity += acceleration * _timeStep;
            target.Position += target.Velocity * _timeStep;
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
    private void CalculateGravityField()
    {
        _gravityFieldVectors.Clear();
        Vector3 gravityDirection = Vector3.Normalize(_gravity);
        float divisionLength = _simulationExtents.X / _divisions;
        float gravityMagnitude = Math.Min(_gravity.Y, divisionLength);
        float updateThreshold = 0.1f;

        List<FieldVector> updatedVectors = new List<FieldVector>();

        for (int x = 0; x < _divisions; x++)
        {
            for (int y = 0; y < _divisions; y++)
            {
                for (int z = 0; z < _divisions; z++)
                {
                    Vector3 position = CalculatePositionInSpace(x, y, z, _divisions);
                    int flatIndex = x * (int)_simulationExtents.Y * (int)_simulationExtents.Z + y * (int)_simulationExtents.Z + z;
                    FieldVector newVector = new FieldVector(position, gravityDirection, gravityMagnitude, flatIndex);

                    if (_previousGravityFieldVectors.ContainsKey(flatIndex))
                    {
                        FieldVector previousVector = _previousGravityFieldVectors[flatIndex];
                        if (Math.Abs(newVector.Magnitude - previousVector.Magnitude) > updateThreshold)
                        {
                            updatedVectors.Add(newVector);
                            _previousGravityFieldVectors[flatIndex] = newVector;
                        }
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
            Vector3 normalizedDirection = Vector3.Normalize(vector.Direction) * scaledMagnitude;

            _magneticFieldVectors.Add(new FieldVector(vector.Position, normalizedDirection, scaledMagnitude, vector.Index));
        }
    }
    private Vector3 CalculateFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        Vector3 totalField = Vector3.Zero;

        switch (_currentMode)
        {
            case SimulationMode.DipoleApproximation:
                {
                    Vector3 r = point - sourceMagnet.Position;
                    float mu0 = 4 * (float)Math.PI * 1e-7f;
                    float rMagnitude = r.Length();

                    if (rMagnitude == 0) return Vector3.Zero;

                    totalField = (mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude)) *
                                 (3 * Vector3.Dot(sourceMagnet.Magnetization, r) * r - sourceMagnet.Magnetization * rMagnitude * rMagnitude);
                    break;
                }
            case SimulationMode.VoxelBased:
                totalField = sourceMagnet.Voxels.Aggregate(totalField, (current, voxel) =>
                    current + CalculateFieldFromVoxel(voxel, point));
                break;
        }

        return totalField;
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
    private float ScaleMagnitude(float value, float min, float max, float newMin, float newMax)
    {
        if (max - min == 0) return newMin;
        return (value - min) / (max - min) * (newMax - newMin) + newMin;
    }
    private Vector3 CalculatePositionInSpace(int x, int y, int z, int divisions)
    {
        float stepSize = _simulationExtents.X / divisions;
        float offset = stepSize / 2.0f;

        return new Vector3(
            (x * stepSize + offset) - (_simulationExtents.X / 2.0f),
            (y * stepSize + offset) - (_simulationExtents.Y / 2.0f),
            (z * stepSize + offset) - (_simulationExtents.Z / 2.0f)
        );
    }
}