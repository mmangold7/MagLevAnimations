using System.Numerics;

namespace Animations.Shared;

public static class QuaternionExtensions
{
    public static Matrix4x4 ToMatrix(this Quaternion quaternion)
    {
        return Matrix4x4.CreateFromQuaternion(quaternion);
    }
}