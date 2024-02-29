using System.Numerics;

namespace Animations.Shared.Extensions;

public static class SimulationExtensions
{
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