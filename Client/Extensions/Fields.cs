using System.Numerics;
using Animations.Client.Models;
using Animations.Client.Models.Enums;

namespace Animations.Client.Extensions;

public static class Fields
{
    public static List<FieldVector> CalculateGravityField(
        Vector3 gravity,
        Dictionary<int, FieldVector> previousGravityFieldVectors,
        Vector3 simulationExtents,
        int divisions)
    {
        var gravityDirection = gravity == Vector3.Zero
            ? Vector3.Zero
            : Vector3.Normalize(gravity);

        var divisionLength = simulationExtents.X / divisions;
        var gravityMagnitude = Math.Min(gravity.Y, divisionLength);
        const float updateThreshold = 0.1f;

        var updatedVectors = new List<FieldVector>();

        for (var x = 0; x < divisions; x++)
        {
            for (var y = 0; y < divisions; y++)
            {
                for (var z = 0; z < divisions; z++)
                {
                    var stepSize = simulationExtents.X / divisions;
                    var position = CalculatePositionInSimulationSpace(x, y, z, stepSize, simulationExtents);
                    var flatIndex = x * (int)simulationExtents.Y * (int)simulationExtents.Z + y * (int)simulationExtents.Z + z;
                    var newVector = new FieldVector(position, gravityDirection, gravityMagnitude, flatIndex);

                    if (previousGravityFieldVectors.ContainsKey(flatIndex))
                    {
                        var previousVector = previousGravityFieldVectors[flatIndex];
                        if (!(Math.Abs(newVector.Magnitude - previousVector.Magnitude) > updateThreshold)) continue;
                        updatedVectors.Add(newVector);
                        previousGravityFieldVectors[flatIndex] = newVector;
                    }
                    else
                    {
                        updatedVectors.Add(newVector);
                        previousGravityFieldVectors.Add(flatIndex, newVector);
                    }
                }
            }
        }

        return updatedVectors.ToList();
    }

    public static List<FieldVector> CalculateMagneticField(
        List<Magnet> magnets,
        SimulationMode mode,
        Dictionary<int, FieldVector> previousMagneticFieldVectors,
        Vector3 simulationExtents,
        int divisions)
    {
        var magneticFieldVectors = new List<FieldVector>();
        var divisionLength = simulationExtents.X / divisions;
        var maxMagnitude = 0f;
        var minMagnitude = float.MaxValue;
        var updateThreshold = 0.1f;

        var updatedVectors = new List<FieldVector>();

        for (int x = 0; x < divisions; x++)
        {
            for (int y = 0; y < divisions; y++)
            {
                for (int z = 0; z < divisions; z++)
                {
                    var stepSize = simulationExtents.X / divisions;
                    var position = CalculatePositionInSimulationSpace(x, y, z, stepSize, simulationExtents);
                    var totalMagneticField = Vector3.Zero;

                    foreach (var magnet in magnets)
                    {
                        var magneticField = CalculateFieldAtPoint(mode, magnet, position);
                        totalMagneticField += magneticField;
                    }

                    var magnitude = totalMagneticField.Length();
                    maxMagnitude = Math.Max(maxMagnitude, magnitude);
                    minMagnitude = Math.Min(minMagnitude, magnitude);

                    var flatIndex = x * (int)simulationExtents.Y * (int)simulationExtents.Z + y * (int)simulationExtents.Z + z;
                    var newVector = new FieldVector(position, totalMagneticField, magnitude, flatIndex);

                    if (previousMagneticFieldVectors.ContainsKey(flatIndex))
                    {
                        var previousVector = previousMagneticFieldVectors[flatIndex];
                        if ((newVector.Direction - previousVector.Direction).Length() / previousVector.Direction.Length() > updateThreshold)
                        {
                            updatedVectors.Add(newVector);
                            previousMagneticFieldVectors[flatIndex] = newVector;
                        }
                    }
                    else
                    {
                        updatedVectors.Add(newVector);
                        previousMagneticFieldVectors.Add(flatIndex, newVector);
                    }
                }
            }
        }

        foreach (var vector in updatedVectors)
        {
            var scaledMagnitude = ScaleMagnitude(vector.Magnitude, minMagnitude, maxMagnitude, divisionLength * 0.1f, divisionLength);
            var normalizedDirection = Vector3.Normalize(vector.Direction) * scaledMagnitude; // Ensure this line is present and correct

            magneticFieldVectors.Add(new FieldVector(vector.Position, normalizedDirection, scaledMagnitude, vector.Index));
        }

        return magneticFieldVectors;
    }

    private static Vector3 CalculateFieldAtPoint(SimulationMode mode, Magnet sourceMagnet, Vector3 point)
    {
        switch (mode)
        {
            case SimulationMode.DipoleApproximation:
                return CalculateSingleDipoleFieldAtPoint(sourceMagnet, point);
            case SimulationMode.VoxelBased:
                var totalField = Vector3.Zero;
                foreach (var voxel in sourceMagnet.Voxels)
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

    private static float ScaleMagnitude(float value, float min, float max, float minScale, float maxScale)
    {
        var range = max - min;
        if (range == 0) return minScale;
        return (value - min) / range * (maxScale - minScale) + minScale;
    }

    private static Vector3 CalculatePositionInSimulationSpace(int x, int y, int z, float stepSize, Vector3 simulationExtents)
    {
        var offset = stepSize / 2.0f;

        return new Vector3(
            x * stepSize + offset - simulationExtents.X / 2.0f,
            y * stepSize + offset - simulationExtents.Y / 2.0f,
            z * stepSize + offset - simulationExtents.Z / 2.0f
        );
    }

    public static Vector3 CalculateDipoleDipoleForce(Magnet target, Magnet source)
    {
        Vector3 r = target.Position - source.Position;
        float rMagnitude = r.Length();
        Vector3 rHat = r / rMagnitude;

        float mu0 = 4 * (float)Math.PI * 1e-1f;
        float prefactor = 3 * mu0 / (4 * (float)Math.PI * (float)Math.Pow(rMagnitude, 4));

        float m1DotR = Vector3.Dot(source.Magnetization, rHat);
        float m2DotR = Vector3.Dot(target.Magnetization, rHat);
        float m1DotM2 = Vector3.Dot(source.Magnetization, target.Magnetization);

        Vector3 term1 = m2DotR * source.Magnetization;
        Vector3 term2 = m1DotR * target.Magnetization;
        Vector3 term3 = m1DotM2 * rHat;
        Vector3 term4 = -5 * (m1DotR * m2DotR / (float)Math.Pow(rMagnitude, 2)) * rHat;

        Vector3 force = prefactor * (term1 + term2 + term3 + term4);

        return force;
    }

    public static Vector3 CalculateDipoleDipoleForceVoxel(Voxel targetVoxel, Voxel sourceVoxel)
    {
        Vector3 r = targetVoxel.Position - sourceVoxel.Position;
        float mu0 = 4 * (float)Math.PI * 1e-1f;
        float m1 = sourceVoxel.Magnetization.Length() * sourceVoxel.Volume;
        float m2 = targetVoxel.Magnetization.Length() * targetVoxel.Volume;
        float rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        Vector3 force = mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude) *
                        (3 * (Vector3.Dot(sourceVoxel.Magnetization, r) * targetVoxel.Magnetization) +
                         3 * (Vector3.Dot(targetVoxel.Magnetization, r) * sourceVoxel.Magnetization) -
                         5 * Vector3.Dot(sourceVoxel.Magnetization, targetVoxel.Magnetization) * r / rMagnitude) -
                        mu0 / (3 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude) * m1 * m2 * r;

        return force;
    }

    public static Vector3 CalculateSingleDipoleFieldAtPoint(Magnet sourceMagnet, Vector3 point)
    {
        var r = point - sourceMagnet.Position;
        var mu0 = 4 * (float)Math.PI * 1e-1f;
        var rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        return mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude) *
               (3 * Vector3.Dot(sourceMagnet.Magnetization, r) * r -
                sourceMagnet.Magnetization * rMagnitude * rMagnitude);
    }

    public static Vector3 CalculateFieldFromVoxel(Voxel voxel, Vector3 point)
    {
        var r = point - voxel.Position;
        var mu0 = 4 * (float)Math.PI * 1e-1f;
        var rMagnitude = r.Length();

        if (rMagnitude == 0) return Vector3.Zero;

        var field = mu0 / (4 * (float)Math.PI * rMagnitude * rMagnitude * rMagnitude) *
                    (3 * Vector3.Dot(voxel.Magnetization, r) * r - voxel.Magnetization * rMagnitude * rMagnitude);

        return field;
    }

    public static Matrix4x4 ToMatrix(this Quaternion quaternion)
    {
        return Matrix4x4.CreateFromQuaternion(quaternion);
    }

    public static float ShortestDistanceBetweenLines(Vector3 line1Start, Vector3 line1End, Vector3 line2Start, Vector3 line2End)
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
}