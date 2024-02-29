using System.Numerics;

namespace Animations.Shared;

public class CollisionMagnetSimulation
{
    private float _timeStep;
    private Vector3 _gravity;
    private Vector3 _simulationExtents;
    private bool _showGravityField;
    private bool _showMagneticField;
    private int _divisions;
    private readonly List<CollisionMagnet> _collisionMagnets = new();
    private readonly List<FieldVector> _gravityFieldVectors = new();
    private readonly List<FieldVector> _magneticFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousMagneticFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousGravityFieldVectors = new();

    private void SetSimulationParameters(SimulationParameters parameters)
    {
        _timeStep = parameters.TimeStep;
        _gravity = parameters.Gravity;
        _simulationExtents = parameters.SimulationExtents;
        _divisions = parameters.Divisions;
        _showGravityField = parameters.ShowGravityField;
        _showMagneticField = parameters.ShowMagneticField;
    }

    public CollisionMagnetSimulation(SimulationParameters parameters)
    {
        SetSimulationParameters(parameters);
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
        var levitatingMagnet = new CollisionMagnet(targetPosition, new Vector3(0, 1, 0), stabilizingMagnetMass, stabilizingMagnetHeight / 2, stabilizingMagnetHeight, false);

        Vector3 fixedPosition = new Vector3(0.3f, targetPosition.Y + stabilizingMagnetHeight / 2 + fixedMagnetHeight / 2 + gap, 0);
        var stabilizingMagnet = new CollisionMagnet(fixedPosition, new Vector3(0, 1, 0), fixedMagnetMass, fixedMagnetHeight / 2, fixedMagnetHeight, true);

        AddCollisionMagnet(levitatingMagnet);
        AddCollisionMagnet(stabilizingMagnet);
    }

    public SimulationState GetSimulationState()
    {
        return new SimulationState
        {
            Magnets = _collisionMagnets.Select(ConvertMagnetToJsObject).ToArray(),
            GravityFieldData = _showGravityField ? _gravityFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MagneticFieldData = _showMagneticField ? _magneticFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null
        };
    }

    public void UpdateSimulation(SimulationParameters simulationParameters)
    {
        SetSimulationParameters(simulationParameters);

        foreach (var target in _collisionMagnets.Where(m => !m.IsFixed))
        {
            Vector3 totalForce = Vector3.Zero;
            Vector3 totalTorque = Vector3.Zero;

            foreach (var source in _collisionMagnets.Where(m => m != target))
            {
                totalForce += source.ComputeForceOnMagnet(target);
                totalTorque += source.ComputeTorque(target);
            }

            Vector3 gravityForce = _gravity * target.Mass;
            totalForce += gravityForce;
            Vector3 acceleration = totalForce / target.Mass;
            target.UpdateAngularVelocity(totalTorque, _timeStep);
            target.Velocity += acceleration * _timeStep;
            target.UpdatePositionAndAngularPosition(_timeStep);

        }

        DetectAndResolveCollisions();
        RecalculateFields();
    }

    public void RecalculateFields()
    {
        if (_showGravityField) Profiling.RunWithClockingLog(CalculateGravityField);
        if (_showMagneticField) Profiling.RunWithClockingLog(CalculateMagneticField);
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

                    foreach (var magnet in _collisionMagnets)
                    {
                        Vector3 magneticField = magnet.ComputeFieldAtPoint(position);
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

    private object ConvertMagnetToJsObject(CollisionMagnet magnet) => new
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

    private void AddCollisionMagnet(CollisionMagnet CollisionMagnet)
    {
        _collisionMagnets.Add(CollisionMagnet);
    }

    private void DetectAndResolveCollisions()
    {
        for (int i = 0; i < _collisionMagnets.Count; i++)
        {
            for (int j = i + 1; j < _collisionMagnets.Count; j++)
            {
                CollisionMagnet CollisionMagnet1 = _collisionMagnets[i];
                CollisionMagnet CollisionMagnet2 = _collisionMagnets[j];

                if (IsColliding(CollisionMagnet1, CollisionMagnet2))
                {
                    ResolveCollision(CollisionMagnet1, CollisionMagnet2);
                }
            }
        }
    }

    private bool IsColliding(CollisionMagnet magnet1, CollisionMagnet magnet2)
    {
        // Approximate the centerline of each cylinder as a line segment
        Vector3 line1Start = magnet1.Position;
        Vector3 line1End = magnet1.Position + Vector3.Transform(new Vector3(0, 0, magnet1.Length), magnet1.Orientation.ToMatrix());
        Vector3 line2Start = magnet2.Position;
        Vector3 line2End = magnet2.Position + Vector3.Transform(new Vector3(0, 0, magnet2.Length), magnet2.Orientation.ToMatrix());

        // Find the shortest distance between the two line segments (centerlines)
        float distance = ShortestDistanceBetweenLines(line1Start, line1End, line2Start, line2End);

        // Collision if distance is less than the sum of radii
        return distance < (magnet1.Radius + magnet2.Radius);
    }

    // Utility method to calculate the shortest distance between two line segments
    private float ShortestDistanceBetweenLines(Vector3 line1Start, Vector3 line1End, Vector3 line2Start, Vector3 line2End)
    {
        Vector3 u = line1End - line1Start;
        Vector3 v = line2End - line2Start;
        Vector3 w = line1Start - line2Start;

        float a = Vector3.Dot(u, u);
        float b = Vector3.Dot(u, v);
        float c = Vector3.Dot(v, v);
        float d = Vector3.Dot(u, w);
        float e = Vector3.Dot(v, w);

        float denominator = a * c - b * b;
        float sc, tc;

        if (denominator < float.Epsilon)
        {
            sc = 0.0f;
            tc = (b > c ? d / b : e / c);
        }
        else
        {
            sc = (b * e - c * d) / denominator;
            tc = (a * e - b * d) / denominator;
        }

        Vector3 dP = w + (sc * u) - (tc * v);
        return dP.Length();
    }

    private void ResolveCollision(CollisionMagnet magnet1, CollisionMagnet magnet2)
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
}