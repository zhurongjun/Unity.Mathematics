using System;
using System.Runtime.CompilerServices;
using Unity.IL2CPP.CompilerServices;
using Unity.Mathematics;

namespace Unity.Mathematics
{
    /// <summary>
    /// A quaternion type for representing rotations.
    /// </summary>
    [Il2CppEagerStaticClassConstruction]
    [Serializable]
    public partial struct quaternion : System.IEquatable<quaternion>, IFormattable
    {
        /// <summary>
        /// The quaternion component values.
        /// </summary>
        public float4 value;

        /// <summary>A quaternion representing the identity transform.</summary>
        public static readonly quaternion identity = new quaternion(0.0f, 0.0f, 0.0f, 1.0f);

        /// <summary>Constructs a quaternion from four float values.</summary>
        /// <param name="x">The quaternion x component.</param>
        /// <param name="y">The quaternion y component.</param>
        /// <param name="z">The quaternion z component.</param>
        /// <param name="w">The quaternion w component.</param>  
        public quaternion(float x, float y, float z, float w) { value.x = x; value.y = y; value.z = z; value.w = w; }

        /// <summary>Constructs a quaternion from float4 vector.</summary>
        /// <param name="value">The quaternion xyzw component values.</param>  
        public quaternion(float4 value) { this.value = value; }

        /// <summary>Implicitly converts a float4 vector to a quaternion.</summary>
        /// <param name="v">The quaternion xyzw component values.</param>
        /// <returns>The quaternion constructed from a float4 vector.</returns>  
        public static implicit operator quaternion(float4 v) { return new quaternion(v); }

        /// <summary>Constructs a unit quaternion from a float3x3 rotation matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The float3x3 orthonormal rotation matrix.</param>
        public quaternion(float3x3 m)
        {
            float3 u = m.c0;
            float3 v = m.c1;
            float3 w = m.c2;

            uint u_sign = (math.asuint(u.x) & 0x80000000);
            float t = v.y + math.asfloat(math.asuint(w.z) ^ u_sign);
            uint4 u_mask = math.uint4((int)u_sign >> 31);
            uint4 t_mask = math.uint4(math.asint(t) >> 31);

            float tr = 1.0f + math.abs(u.x);

            uint4 sign_flips = math.uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & math.uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & math.uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

            value = math.float4(tr, u.y, w.x, v.z) + math.asfloat(math.asuint(math.float4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

            value = math.asfloat((math.asuint(value) & ~u_mask) | (math.asuint(value.zwxy) & u_mask));
            value = math.asfloat((math.asuint(value.wzyx) & ~t_mask) | (math.asuint(value) & t_mask));
            value = math.normalize(value);
        }

        /// <summary>Constructs a unit quaternion from an orthonormal float4x4 matrix.</summary>
        /// <param name="m">The float4x4 orthonormal rotation matrix.</param>
        public quaternion(float4x4 m)
        {
            float4 u = m.c0;
            float4 v = m.c1;
            float4 w = m.c2;

            uint u_sign = (math.asuint(u.x) & 0x80000000);
            float t = v.y + math.asfloat(math.asuint(w.z) ^ u_sign);
            uint4 u_mask = math.uint4((int)u_sign >> 31);
            uint4 t_mask = math.uint4(math.asint(t) >> 31);

            float tr = 1.0f + math.abs(u.x);

            uint4 sign_flips = math.uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & math.uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & math.uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

            value = math.float4(tr, u.y, w.x, v.z) + math.asfloat(math.asuint(math.float4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

            value = math.asfloat((math.asuint(value) & ~u_mask) | (math.asuint(value.zwxy) & u_mask));
            value = math.asfloat((math.asuint(value.wzyx) & ~t_mask) | (math.asuint(value) & t_mask));

            value = math.normalize(value);
        }

        /// <summary>
        /// Returns a quaternion representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The quaternion representing a rotation around an axis.</returns>  
        public static quaternion AxisAngle(float3 axis, float angle)
        {
            float sina, cosa;
            math.sincos(0.5f * angle, out sina, out cosa);
            return math.quaternion(math.float4(axis * sina, cosa));
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in x-y-z order.</returns>  
        public static quaternion EulerXYZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(-1.0f, 1.0f, -1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in x-z-y order.</returns>  
        public static quaternion EulerXZY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.z * s.x
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(1.0f, 1.0f, -1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in y-x-z order.</returns>  
        public static quaternion EulerYXZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.z * s.x
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(-1.0f, 1.0f, 1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in y-z-x order.</returns>  
        public static quaternion EulerYZX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(-1.0f, -1.0f, 1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in z-x-y order.</returns>  
        public static quaternion EulerZXY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(1.0f, -1.0f, -1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in z-y-x order.</returns>  
        public static quaternion EulerZYX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(0.5f * xyz, out s, out c);
            return math.quaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.x * s.z
                math.float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * math.float4(c.xyz, s.x) * math.float4(1.0f, -1.0f, 1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in x-y-z order.</returns>  
        public static quaternion EulerXYZ(float x, float y, float z) { return EulerXYZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in x-z-y order.</returns>  
        public static quaternion EulerXZY(float x, float y, float z) { return EulerXZY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in y-x-z order.</returns>  
        public static quaternion EulerYXZ(float x, float y, float z) { return EulerYXZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in y-z-x order.</returns>  
        public static quaternion EulerYZX(float x, float y, float z) { return EulerYZX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in z-x-y order.</returns>  
        public static quaternion EulerZXY(float x, float y, float z) { return EulerZXY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The quaternion representing the Euler angle rotation in z-y-x order.</returns>  
        public static quaternion EulerZYX(float x, float y, float z) { return EulerZYX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The quaternion representing the Euler angle rotation in the specified order.</returns>  
        public static quaternion Euler(float3 xyz, math.RotationOrder order = math.RotationOrder.ZXY)
        {
            switch (order)
            {
                case math.RotationOrder.XYZ:
                    return EulerXYZ(xyz);
                case math.RotationOrder.XZY:
                    return EulerXZY(xyz);
                case math.RotationOrder.YXZ:
                    return EulerYXZ(xyz);
                case math.RotationOrder.YZX:
                    return EulerYZX(xyz);
                case math.RotationOrder.ZXY:
                    return EulerZXY(xyz);
                case math.RotationOrder.ZYX:
                    return EulerZYX(xyz);
                default:
                    return quaternion.identity;
            }
        }

        /// <summary>
        /// Returns a quaternion constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The quaternion representing the Euler angle rotation in the specified order.</returns>  
        public static quaternion Euler(float x, float y, float z, math.RotationOrder order = math.RotationOrder.Default)
        {
            return Euler(math.float3(x, y, z), order);
        }

        /// <summary>Returns a quaternion that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The quaternion representing a rotation around the x-axis.</returns>  
        public static quaternion RotateX(float angle)
        {
            float sina, cosa;
            math.sincos(0.5f * angle, out sina, out cosa);
            return math.quaternion(sina, 0.0f, 0.0f, cosa);
        }

        /// <summary>Returns a quaternion that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The quaternion representing a rotation around the y-axis.</returns>  
        public static quaternion RotateY(float angle)
        {
            float sina, cosa;
            math.sincos(0.5f * angle, out sina, out cosa);
            return math.quaternion(0.0f, sina, 0.0f, cosa);
        }

        /// <summary>Returns a quaternion that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The quaternion representing a rotation around the z-axis.</returns>  
        public static quaternion RotateZ(float angle)
        {
            float sina, cosa;
            math.sincos(0.5f * angle, out sina, out cosa);
            return math.quaternion(0.0f, 0.0f, sina, cosa);
        }

        /// <summary>
        /// Returns a quaternion view rotation given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use float3x3.LookRotationSafe instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The quaternion view rotation.</returns>  
        public static quaternion LookRotation(float3 forward, float3 up)
        {
            float3 t = math.normalize(math.cross(up, forward));
            return math.quaternion(math.float3x3(t, math.cross(forward, t), forward));
        }

        /// <summary>
        /// Returns a quaternion view rotation given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The quaternion view rotation or the identity quaternion.</returns>
        public static quaternion LookRotationSafe(float3 forward, float3 up)
        {
            float forwardLengthSq = math.dot(forward, forward);
            float upLengthSq = math.dot(up, up);

            forward *= math.rsqrt(forwardLengthSq);
            up *= math.rsqrt(upLengthSq);

            float3 t = math.cross(up, forward);
            float tLengthSq = math.dot(t, t);
            t *= math.rsqrt(tLengthSq);

            float mn = math.min(math.min(forwardLengthSq, upLengthSq), tLengthSq);
            float mx = math.max(math.max(forwardLengthSq, upLengthSq), tLengthSq);

            bool accept = mn > 1e-35f && mx < 1e35f && math.isfinite(forwardLengthSq) && math.isfinite(upLengthSq) && math.isfinite(tLengthSq);
            return math.quaternion(math.select(math.float4(0.0f, 0.0f, 0.0f, 1.0f), math.quaternion(math.float3x3(t, math.cross(forward, t),forward)).value, accept));
        }

        /// <summary>Returns true if the quaternion is equal to a given quaternion, false otherwise.</summary>
        /// <param name="x">The quaternion to compare with.</param>
        /// <returns>True if the quaternion is equal to the input, false otherwise.</returns>  
        public bool Equals(quaternion x) { return value.x == x.value.x && value.y == x.value.y && value.z == x.value.z && value.w == x.value.w; }

        /// <summary>Returns whether true if the quaternion is equal to a given quaternion, false otherwise.</summary>
        /// <param name="x">The object to compare with.</param>
        /// <returns>True if the quaternion is equal to the input, false otherwise.</returns>  
        public override bool Equals(object x) { return x is quaternion && Equals((quaternion)x); }

        /// <summary>Returns a hash code for the quaternion.</summary>
        /// <returns>The hash code of the quaternion.</returns>  
        public override int GetHashCode() { return (int)math.hash(this); }

        /// <summary>Returns a string representation of the quaternion.</summary>
        /// <returns>The string representation of the quaternion.</returns>  
        public override string ToString()
        {
            return string.Format("quaternion({0}f, {1}f, {2}f, {3}f)", value.x, value.y, value.z, value.w);
        }

        /// <summary>Returns a string representation of the quaternion using a specified format and culture-specific format information.</summary>
        /// <param name="format">The format string.</param>
        /// <param name="formatProvider">The format provider to use during string formatting.</param>
        /// <returns>The formatted string representation of the quaternion.</returns>  
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("quaternion({0}f, {1}f, {2}f, {3}f)", value.x.ToString(format, formatProvider), value.y.ToString(format, formatProvider), value.z.ToString(format, formatProvider), value.w.ToString(format, formatProvider));
        }
    }

    public static partial class math
    {
        /// <summary>Returns a quaternion constructed from four float values.</summary>
        /// <param name="x">The x component of the quaternion.</param>
        /// <param name="y">The y component of the quaternion.</param>
        /// <param name="z">The z component of the quaternion.</param>
        /// <param name="w">The w component of the quaternion.</param>
        /// <returns>The quaternion constructed from individual components.</returns>  
        public static quaternion quaternion(float x, float y, float z, float w) { return new quaternion(x, y, z, w); }

        /// <summary>Returns a quaternion constructed from a float4 vector.</summary>
        /// <param name="value">The float4 containing the components of the quaternion.</param>
        /// <returns>The quaternion constructed from a float4.</returns>  
        public static quaternion quaternion(float4 value) { return new quaternion(value); }

        /// <summary>Returns a unit quaternion constructed from a float3x3 rotation matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The float3x3 rotation matrix.</param>
        /// <returns>The quaternion constructed from a float3x3 matrix.</returns>  
        public static quaternion quaternion(float3x3 m) { return new quaternion(m); }

        /// <summary>Returns a unit quaternion constructed from a float4x4 matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The float4x4 matrix (must be orthonormal).</param>
        /// <returns>The quaternion constructed from a float4x4 matrix.</returns>  
        public static quaternion quaternion(float4x4 m) { return new quaternion(m); }

       /// <summary>Returns the conjugate of a quaternion value.</summary>
       /// <param name="q">The quaternion to conjugate.</param>
       /// <returns>The conjugate of the input quaternion.</returns>  
        public static quaternion conjugate(quaternion q)
        {
            return math.quaternion(q.value * math.float4(-1.0f, -1.0f, -1.0f, 1.0f));
        }

       /// <summary>Returns the inverse of a quaternion value.</summary>
       /// <param name="q">The quaternion to invert.</param>
       /// <returns>The quaternion inverse of the input quaternion.</returns>  
        public static quaternion inverse(quaternion q)
        {
            float4 x = q.value;
            return math.quaternion(rcp(math.dot(x, x)) * x * math.float4(-1.0f, -1.0f, -1.0f, 1.0f));
        }

        /// <summary>Returns the math.dot product of two quaternions.</summary>
        /// <param name="a">The first quaternion.</param>
        /// <param name="b">The second quaternion.</param>
        /// <returns>The math.dot product of two quaternions.</returns>  
        public static float dot(quaternion a, quaternion b)
        {
            return math.dot(a.value, b.value);
        }

        /// <summary>Returns the length of a quaternion.</summary>
        /// <param name="q">The input quaternion.</param>
        /// <returns>The length of the input quaternion.</returns>  
        public static float length(quaternion q)
        {
            return sqrt(math.dot(q.value, q.value));
        }

        /// <summary>Returns the squared length of a quaternion.</summary>
        /// <param name="q">The input quaternion.</param>
        /// <returns>The length squared of the input quaternion.</returns>  
        public static float lengthsq(quaternion q)
        {
            return math.dot(q.value, q.value);
        }

        /// <summary>Returns a math.normalized version of a quaternion q by scaling it by 1 / length(q).</summary>
        /// <param name="q">The quaternion to math.normalize.</param>
        /// <returns>The math.normalized quaternion.</returns>  
        public static quaternion normalize(quaternion q)
        {
            float4 x = q.value;
            return math.quaternion(math.rsqrt(math.dot(x, x)) * x);
        }

        /// <summary>
        /// Returns a safe math.normalized version of the q by scaling it by 1 / length(q).
        /// Returns the identity when 1 / length(q) does not produce a finite number.
        /// </summary>
        /// <param name="q">The quaternion to math.normalize.</param>
        /// <returns>The math.normalized quaternion or the identity quaternion.</returns>  
        public static quaternion normalizesafe(quaternion q)
        {
            float4 x = q.value;
            float len = math.dot(x, x);
            return math.quaternion(math.select(Mathematics.quaternion.identity.value, x * math.rsqrt(len), len > FLT_MIN_NORMAL));
        }

        /// <summary>
        /// Returns a safe math.normalized version of the q by scaling it by 1 / length(q).
        /// Returns the given default value when 1 / length(q) does not produce a finite number.
        /// </summary>
        /// <param name="q">The quaternion to math.normalize.</param>
        /// <param name="defaultvalue">The default value.</param>
        /// <returns>The math.normalized quaternion or the default value.</returns>  
        public static quaternion normalizesafe(quaternion q, quaternion defaultvalue)
        {
            float4 x = q.value;
            float len = math.dot(x, x);
            return math.quaternion(math.select(defaultvalue.value, x * math.rsqrt(len), len > FLT_MIN_NORMAL));
        }

        /// <summary>Returns the natural exponent of a quaternion. Assumes w is zero.</summary>
        /// <param name="q">The quaternion with w component equal to zero.</param>
        /// <returns>The natural exponent of the input quaternion.</returns>  
        public static quaternion unitexp(quaternion q)
        {
            float v_rcp_len = math.rsqrt(math.dot(q.value.xyz, q.value.xyz));
            float v_len = rcp(v_rcp_len);
            float sin_v_len, cos_v_len;
            math.sincos(v_len, out sin_v_len, out cos_v_len);
            return math.quaternion(math.float4(q.value.xyz * v_rcp_len * sin_v_len, cos_v_len));
        }

        /// <summary>Returns the natural exponent of a quaternion.</summary>
        /// <param name="q">The quaternion.</param>
        /// <returns>The natural exponent of the input quaternion.</returns>  
        public static quaternion exp(quaternion q)
        {
            float v_rcp_len = math.rsqrt(math.dot(q.value.xyz, q.value.xyz));
            float v_len = rcp(v_rcp_len);
            float sin_v_len, cos_v_len;
            math.sincos(v_len, out sin_v_len, out cos_v_len);
            return math.quaternion(math.float4(q.value.xyz * v_rcp_len * sin_v_len, cos_v_len) * exp(q.value.w));
        }

        /// <summary>Returns the natural logarithm of a unit length quaternion.</summary>
        /// <param name="q">The unit length quaternion.</param>
        /// <returns>The natural logarithm of the unit length quaternion.</returns>  
        public static quaternion unitlog(quaternion q)
        {
            float w = clamp(q.value.w, -1.0f, 1.0f);
            float s = acos(w) * math.rsqrt(1.0f - w*w);
            return math.quaternion(math.float4(q.value.xyz * s, 0.0f));
        }

        /// <summary>Returns the natural logarithm of a quaternion.</summary>
        /// <param name="q">The quaternion.</param>
        /// <returns>The natural logarithm of the input quaternion.</returns>  
        public static quaternion log(quaternion q)
        {
            float v_len_sq = math.dot(q.value.xyz, q.value.xyz);
            float q_len_sq = v_len_sq + q.value.w*q.value.w;

            float s = acos(clamp(q.value.w * math.rsqrt(q_len_sq), -1.0f, 1.0f)) * math.rsqrt(v_len_sq);
            return math.quaternion(math.float4(q.value.xyz * s, 0.5f * log(q_len_sq)));
        }

        /// <summary>Returns the result of transformath.ming the quaternion b by the quaternion a.</summary>
        /// <param name="a">The quaternion on the left.</param>
        /// <param name="b">The quaternion on the right.</param>
        /// <returns>The result of transformath.ming quaternion b by the quaternion a.</returns>  
        public static quaternion mul(quaternion a, quaternion b)
        {
            return math.quaternion(a.value.wwww * b.value + (a.value.xyzx * b.value.wwwx + a.value.yzxy * b.value.zxyy) * math.float4(1.0f, 1.0f, 1.0f, -1.0f) - a.value.zxyz * b.value.yzxz);
        }

        /// <summary>Returns the result of transformath.ming a vector by a quaternion.</summary>
        /// <param name="q">The quaternion transformation.</param>
        /// <param name="v">The vector to transform.</param>
        /// <returns>The transformation of vector v by quaternion q.</returns>  
        public static float3 mul(quaternion q, float3 v)
        {
            float3 t = 2 * math.cross(q.value.xyz, v);
            return v + q.value.w * t + math.cross(q.value.xyz, t);
        }

        /// <summary>Returns the result of rotating a vector by a unit quaternion.</summary>
        /// <param name="q">The quaternion rotation.</param>
        /// <param name="v">The vector to rotate.</param>
        /// <returns>The rotation of vector v by quaternion q.</returns>  
        public static float3 rotate(quaternion q, float3 v)
        {
            float3 t = 2 * math.cross(q.value.xyz, v);
            return v + q.value.w * t + math.cross(q.value.xyz, t);
        }

        /// <summary>Returns the result of a math.normalized linear interpolation between two quaternions q1 and a2 using an interpolation parameter t.</summary>
        /// <remarks>
        /// Prefer to use this over slerp() when you know the distance between q1 and q2 is small. This can be much
        /// higher performance due to avoiding trigonometric function evaluations that occur in slerp().
        /// </remarks>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <param name="t">The interpolation parameter.</param>
        /// <returns>The math.normalized linear interpolation of two quaternions.</returns>  
        public static quaternion nlerp(quaternion q1, quaternion q2, float t)
        {
            float dt = math.dot(q1, q2);
            if(dt < 0.0f)
            {
                q2.value = -q2.value;
            }

            return math.normalize(quaternion(lerp(q1.value, q2.value, t)));
        }

        /// <summary>Returns the result of a spherical interpolation between two quaternions q1 and a2 using an interpolation parameter t.</summary>
        /// <param name="q1">The first quaternion.</param>
        /// <param name="q2">The second quaternion.</param>
        /// <param name="t">The interpolation parameter.</param>
        /// <returns>The spherical linear interpolation of two quaternions.</returns>  
        public static quaternion slerp(quaternion q1, quaternion q2, float t)
        {
            float dt = math.dot(q1, q2);
            if (dt < 0.0f)
            {
                dt = -dt;
                q2.value = -q2.value;
            }

            if (dt < 0.9995f)
            {
                float angle = acos(dt);
                float s = math.rsqrt(1.0f - dt * dt);    // 1.0f / sin(angle)
                float w1 = sin(angle * (1.0f - t)) * s;
                float w2 = sin(angle * t) * s;
                return math.quaternion(q1.value * w1 + q2.value * w2);
            }
            else
            {
                // if the angle is small, use linear interpolation
                return nlerp(q1, q2, t);
            }
        }

        /// <summary>Returns a uint hash code of a quaternion.</summary>
        /// <param name="q">The quaternion to hash.</param>
        /// <returns>The hash code for the input quaternion.</returns>  
        public static uint hash(quaternion q)
        {
            return hash(q.value);
        }

        /// <summary>
        /// Returns a uint4 vector hash code of a quaternion.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        /// <param name="q">The quaternion to hash.</param>
        /// <returns>The uint4 vector hash code of the input quaternion.</returns>  
        public static uint4 hashwide(quaternion q)
        {
            return hashwide(q.value);
        }


        /// <summary>
        /// Transforms the forward vector by a quaternion.
        /// </summary>
        /// <param name="q">The quaternion transformation.</param>
        /// <returns>The forward vector transformed by the input quaternion.</returns>  
        public static float3 forward(quaternion q) { return mul(q, math.float3(0, 0, 1)); }  // for compatibility
    }
}
