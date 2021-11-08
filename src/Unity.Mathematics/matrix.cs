using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.Mathematics
{
    public partial struct float2x2
    {
        /// <summary>
        /// Computes a float2x2 matrix representing a counter-clockwise rotation by an angle in radians.
        /// </summary>
        /// <remarks>
        /// A positive rotation angle will produce a counter-clockwise rotation and a negative rotation angle will
        /// produce a clockwise rotation.
        /// </remarks>
        /// <param name="angle">Rotation angle in radians.</param>
        /// <returns>Returns the 2x2 rotation matrix.</returns>  
        public static float2x2 Rotate(float angle)
        {
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float2x2(c, -s,
                            s,  c);
        }

        /// <summary>Returns a float2x2 matrix representing a uniform scaling of both axes by s.</summary>
        /// <param name="s">The scaling factor.</param>
        /// <returns>The float2x2 matrix representing uniform scale by s.</returns>  
        public static float2x2 Scale(float s)
        {
            return math.float2x2(s,    0.0f,
                            0.0f, s);
        }

        /// <summary>Returns a float2x2 matrix representing a non-uniform axis scaling by x and y.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <returns>The float2x2 matrix representing a non-uniform scale.</returns>  
        public static float2x2 Scale(float x, float y)
        {
            return math.float2x2(x,    0.0f,
                            0.0f, y);
        }

        /// <summary>Returns a float2x2 matrix representing a non-uniform axis scaling by the components of the float2 vector v.</summary>
        /// <param name="v">The float2 containing the x and y axis scaling factors.</param>
        /// <returns>The float2x2 matrix representing a non-uniform scale.</returns>  
        public static float2x2 Scale(float2 v)
        {
            return Scale(v.x, v.y);
        }
    }

    public partial struct float3x3
    {
        /// <summary>
        /// Constructs a float3x3 from the upper left 3x3 of a float4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="float4x4"/> to extract a float3x3 from.</param>
        public float3x3(float4x4 f4x4)
        {
            c0 = f4x4.c0.xyz;
            c1 = f4x4.c1.xyz;
            c2 = f4x4.c2.xyz;
        }

        /// <summary>Constructs a float3x3 matrix from a unit quaternion.</summary>
        /// <param name="q">The quaternion rotation.</param>
        public float3x3(quaternion q)
        {
            float4 v = q.value;
            float4 v2 = v + v;

            uint3 npn = math.uint3(0x80000000, 0x00000000, 0x80000000);
            uint3 nnp = math.uint3(0x80000000, 0x80000000, 0x00000000);
            uint3 pnn = math.uint3(0x00000000, 0x80000000, 0x80000000);
            c0 = v2.y * math.asfloat(math.asuint(v.yxw) ^ npn) - v2.z * math.asfloat(math.asuint(v.zwx) ^ pnn) + math.float3(1, 0, 0);
            c1 = v2.z * math.asfloat(math.asuint(v.wzy) ^ nnp) - v2.x * math.asfloat(math.asuint(v.yxw) ^ npn) + math.float3(0, 1, 0);
            c2 = v2.x * math.asfloat(math.asuint(v.zwx) ^ pnn) - v2.y * math.asfloat(math.asuint(v.wzy) ^ nnp) + math.float3(0, 0, 1);
        }

        /// <summary>
        /// Returns a float3x3 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The rotation axis.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The float3x3 matrix representing the rotation around an axis.</returns>  
        public static float3x3 AxisAngle(float3 axis, float angle)
        {
            float sina, cosa;
            math.sincos(angle, out sina, out cosa);

            float3 u = axis;
            float3 u_yzx = u.yzx;
            float3 u_zxy = u.zxy;
            float3 u_inv_cosa = u - u * cosa;  // u * (1.0f - cosa);
            float4 t = math.float4(u * sina, cosa);

            uint3 ppn = math.uint3(0x00000000, 0x00000000, 0x80000000);
            uint3 npp = math.uint3(0x80000000, 0x00000000, 0x00000000);
            uint3 pnp = math.uint3(0x00000000, 0x80000000, 0x00000000);

            return math.float3x3(
                u.x * u_inv_cosa + math.asfloat(math.asuint(t.wzy) ^ ppn),
                u.y * u_inv_cosa + math.asfloat(math.asuint(t.zwx) ^ npp),
                u.z * u_inv_cosa + math.asfloat(math.asuint(t.yxw) ^ pnp)
                );
            /*
            return math.float3x3(
                cosa + u.x * u.x * (1.0f - cosa),       u.y * u.x * (1.0f - cosa) - u.z * sina, u.z * u.x * (1.0f - cosa) + u.y * sina,
                u.x * u.y * (1.0f - cosa) + u.z * sina, cosa + u.y * u.y * (1.0f - cosa),       u.y * u.z * (1.0f - cosa) - u.x * sina,
                u.x * u.z * (1.0f - cosa) - u.y * sina, u.y * u.z * (1.0f - cosa) + u.x * sina, cosa + u.z * u.z * (1.0f - cosa)
                );
                */
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in x-y-z order.</returns>  
        public static float3x3 EulerXYZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,
                -s.y,       c.y * s.x,                      c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in x-z-y order.</returns>  
        public static float3x3 EulerXZY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,
                s.z,        c.x * c.z,                      -c.z * s.x,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in y-x-z order.</returns>  
        public static float3x3 EulerYXZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,
                -c.x * s.y,                     s.x,        c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in y-z-x order.</returns>  
        public static float3x3 EulerYZX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z,                      -s.z,       c.z * s.y,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in z-x-y order.</returns>  
        public static float3x3 EulerZXY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,
                c.x * s.z,                      c.x * c.z,                      -s.x,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in z-y-x order.</returns>  
        public static float3x3 EulerZYX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float3x3(
                c.y * c.z,                      -c.y * s.z,                     s.y,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in x-y-z order.</returns>  
        public static float3x3 EulerXYZ(float x, float y, float z) { return EulerXYZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in x-z-y order.</returns>  
        public static float3x3 EulerXZY(float x, float y, float z) { return EulerXZY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in y-x-z order.</returns>  
        public static float3x3 EulerYXZ(float x, float y, float z) { return EulerYXZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in y-z-x order.</returns>  
        public static float3x3 EulerYZX(float x, float y, float z) { return EulerYZX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in z-x-y order.</returns>  
        public static float3x3 EulerZXY(float x, float y, float z) { return EulerZXY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in z-y-x order.</returns>  
        public static float3x3 EulerZYX(float x, float y, float z) { return EulerZYX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in the given order.</returns>  
        public static float3x3 Euler(float3 xyz, math.RotationOrder order = math.RotationOrder.Default)
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
                    return float3x3.identity;
            }
        }

        /// <summary>
        /// Returns a float3x3 rotation matrix constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The float3x3 rotation matrix representing the rotation by Euler angles in the given order.</returns>  
        public static float3x3 Euler(float x, float y, float z, math.RotationOrder order = math.RotationOrder.Default)
        {
            return Euler(math.float3(x, y, z), order);
        }

        /// <summary>Returns a float3x3 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The float3x3 rotation matrix representing a rotation around the x-axis.</returns>  
        public static float3x3 RotateX(float angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float3x3(1.0f, 0.0f, 0.0f,
                            0.0f, c,    -s,
                            0.0f, s,    c);
        }

        /// <summary>Returns a float3x3 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The float3x3 rotation matrix representing a rotation around the y-axis.</returns>  
        public static float3x3 RotateY(float angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float3x3(c,    0.0f, s,
                            0.0f, 1.0f, 0.0f,
                            -s,   0.0f, c);
        }

        /// <summary>Returns a float3x3 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The float3x3 rotation matrix representing a rotation around the z-axis.</returns>  
        public static float3x3 RotateZ(float angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float3x3(c,    -s,   0.0f,
                            s,    c,    0.0f,
                            0.0f, 0.0f, 1.0f);
        }

        /// <summary>Returns a float3x3 matrix representing a uniform scaling of all axes by s.</summary>
        /// <param name="s">The uniform scaling factor.</param>
        /// <returns>The float3x3 matrix representing a uniform scale.</returns>  
        public static float3x3 Scale(float s)
        {
            return math.float3x3(s,    0.0f, 0.0f,
                            0.0f, s,    0.0f,
                            0.0f, 0.0f, s);
        }

        /// <summary>Returns a float3x3 matrix representing a non-uniform axis scaling by x, y and z.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <param name="z">The z-axis scaling factor.</param>
        /// <returns>The float3x3 rotation matrix representing a non-uniform scale.</returns>  
        public static float3x3 Scale(float x, float y, float z)
        {
            return math.float3x3(x,    0.0f, 0.0f,
                            0.0f, y,    0.0f,
                            0.0f, 0.0f, z);
        }

        /// <summary>Returns a float3x3 matrix representing a non-uniform axis scaling by the components of the float3 vector v.</summary>
        /// <param name="v">The vector containing non-uniform scaling factors.</param>
        /// <returns>The float3x3 rotation matrix representing a non-uniform scale.</returns>  
        public static float3x3 Scale(float3 v)
        {
            return Scale(v.x, v.y, v.z);
        }

        /// <summary>
        /// Returns a float3x3 view rotation matrix given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use float3x3.LookRotationSafe instead.
        /// </summary>
        /// <param name="forward">The forward vector to align the center of view with.</param>
        /// <param name="up">The up vector to point top of view toward.</param>
        /// <returns>The float3x3 view rotation matrix.</returns>  
        public static float3x3 LookRotation(float3 forward, float3 up)
        {
            float3 t = math.normalize(math.cross(up, forward));
            return math.float3x3(t, math.cross(forward, t), forward);
        }

        /// <summary>
        /// Returns a float3x3 view rotation matrix given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        /// <param name="forward">The forward vector to align the center of view with.</param>
        /// <param name="up">The up vector to point top of view toward.</param>
        /// <returns>The float3x3 view rotation matrix or the identity matrix.</returns>  
        public static float3x3 LookRotationSafe(float3 forward, float3 up)
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
            return math.float3x3(
                math.select(math.float3(1,0,0), t, accept),
                math.select(math.float3(0,1,0), math.cross(forward, t), accept),
                math.select(math.float3(0,0,1), forward, accept));
        }

        /// <summary>
        /// Converts a float4x4 to a float3x3.
        /// </summary>
        /// <param name="f4x4">The float4x4 to convert to a float3x3.</param>
        /// <returns>The float3x3 constructed from the upper left 3x3 of the input float4x4 matrix.</returns>
        public static explicit operator float3x3(float4x4 f4x4)
        {
            return new float3x3(f4x4);
        }
    }

    public partial struct float4x4
    {
        /// <summary>Constructs a float4x4 from a float3x3 rotation matrix and a float3 translation vector.</summary>
        /// <param name="rotation">The float3x3 rotation matrix.</param>
        /// <param name="translation">The translation vector.</param>
        public float4x4(float3x3 rotation, float3 translation)
        {
            c0 = math.float4(rotation.c0, 0.0f);
            c1 = math.float4(rotation.c1, 0.0f);
            c2 = math.float4(rotation.c2, 0.0f);
            c3 = math.float4(translation, 1.0f);
        }

        /// <summary>Constructs a float4x4 from a quaternion and a float3 translation vector.</summary>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="translation">The translation vector.</param>
        public float4x4(quaternion rotation, float3 translation)
        {
            float3x3 rot = math.float3x3(rotation);
            c0 = math.float4(rot.c0, 0.0f);
            c1 = math.float4(rot.c1, 0.0f);
            c2 = math.float4(rot.c2, 0.0f);
            c3 = math.float4(translation, 1.0f);
        }

        /// <summary>Constructs a float4x4 from a RigidTransform.</summary>
        /// <param name="transform">The RigidTransform.</param>
        public float4x4(RigidTransform transform)
        {
            float3x3 rot = math.float3x3(transform.rot);
            c0 = math.float4(rot.c0, 0.0f);
            c1 = math.float4(rot.c1, 0.0f);
            c2 = math.float4(rot.c2, 0.0f);
            c3 = math.float4(transform.pos, 1.0f);
        }

        /// <summary>
        /// Returns a float4x4 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The float4x4 matrix representing the rotation about an axis.</returns>  
        public static float4x4 AxisAngle(float3 axis, float angle)
        {
            float sina, cosa;
            math.sincos(angle, out sina, out cosa);

            float4 u = math.float4(axis, 0.0f);
            float4 u_yzx = u.yzxx;
            float4 u_zxy = u.zxyx;
            float4 u_inv_cosa = u - u * cosa;  // u * (1.0f - cosa);
            float4 t = math.float4(u.xyz * sina, cosa);

            uint4 ppnp = math.uint4(0x00000000, 0x00000000, 0x80000000, 0x00000000);
            uint4 nppp = math.uint4(0x80000000, 0x00000000, 0x00000000, 0x00000000);
            uint4 pnpp = math.uint4(0x00000000, 0x80000000, 0x00000000, 0x00000000);
            uint4 mask = math.uint4(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000);

            return math.float4x4(
                u.x * u_inv_cosa + math.asfloat((math.asuint(t.wzyx) ^ ppnp) & mask),
                u.y * u_inv_cosa + math.asfloat((math.asuint(t.zwxx) ^ nppp) & mask),
                u.z * u_inv_cosa + math.asfloat((math.asuint(t.yxwx) ^ pnpp) & mask),
                math.float4(0.0f, 0.0f, 0.0f, 1.0f)
                );

        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in x-y-z order.</returns>  
        public static float4x4 EulerXYZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,    0.0f,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,    0.0f,
                -s.y,       c.y * s.x,                      c.x * c.y,                      0.0f,
                0.0f,       0.0f,                           0.0f,                           1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in x-z-y order.</returns>  
        public static float4x4 EulerXZY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,    0.0f,
                s.z,        c.x * c.z,                      -c.z * s.x,                     0.0f,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z,    0.0f,
                0.0f,       0.0f,                           0.0f,                           1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in y-x-z order.</returns>  
        public static float4x4 EulerYXZ(float3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,    0.0f,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,    0.0f,
                -c.x * s.y,                     s.x,        c.x * c.y,                      0.0f,
                0.0f,                           0.0f,       0.0f,                           1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in y-z-x order.</returns>  
        public static float4x4 EulerYZX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z,                      -s.z,       c.z * s.y,                      0.0f,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,    0.0f,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z,    0.0f,
                0.0f,                           0.0f,       0.0f,                           1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in z-x-y order.</returns>  
        public static float4x4 EulerZXY(float3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,  0.0f,
                c.x * s.z,                      c.x * c.z,                      -s.x,       0.0f,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y,  0.0f,
                0.0f,                           0.0f,                           0.0f,       1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in z-y-x order.</returns>  
        public static float4x4 EulerZYX(float3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            float3 s, c;
            math.sincos(xyz, out s, out c);
            return math.float4x4(
                c.y * c.z,                      -c.y * s.z,                     s.y,        0.0f,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x, 0.0f,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y,  0.0f,
                0.0f,                           0.0f,                           0.0f,       1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in x-y-z order.</returns>  
        public static float4x4 EulerXYZ(float x, float y, float z) { return EulerXYZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in x-z-y order.</returns>  
        public static float4x4 EulerXZY(float x, float y, float z) { return EulerXZY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in y-x-z order.</returns>  
        public static float4x4 EulerYXZ(float x, float y, float z) { return EulerYXZ(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in y-z-x order.</returns>  
        public static float4x4 EulerYZX(float x, float y, float z) { return EulerYZX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in z-x-y order.</returns>  
        public static float4x4 EulerZXY(float x, float y, float z) { return EulerZXY(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in z-y-x order.</returns>  
        public static float4x4 EulerZYX(float x, float y, float z) { return EulerZYX(math.float3(x, y, z)); }

        /// <summary>
        /// Returns a float4x4 constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in given order.</returns>  
        public static float4x4 Euler(float3 xyz, math.RotationOrder order = math.RotationOrder.Default)
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
                    return float4x4.identity;
            }
        }

        /// <summary>
        /// Returns a float4x4 rotation matrix constructed by first performath.ming 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The float4x4 rotation matrix of the Euler angle rotation in given order.</returns>  
        public static float4x4 Euler(float x, float y, float z, math.RotationOrder order = math.RotationOrder.Default)
        {
            return Euler(math.float3(x, y, z), order);
        }

        /// <summary>Returns a float4x4 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The float4x4 rotation matrix that rotates around the x-axis.</returns>  
        public static float4x4 RotateX(float angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float4x4(1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, c,    -s,   0.0f,
                            0.0f, s,    c,    0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);

        }

        /// <summary>Returns a float4x4 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The float4x4 rotation matrix that rotates around the y-axis.</returns>  
        public static float4x4 RotateY(float angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float4x4(c,    0.0f, s,    0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            -s,   0.0f, c,    0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);

        }

        /// <summary>Returns a float4x4 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The float4x4 rotation matrix that rotates around the z-axis.</returns>  
        public static float4x4 RotateZ(float angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            float s, c;
            math.sincos(angle, out s, out c);
            return math.float4x4(c,    -s,   0.0f, 0.0f,
                            s,    c,    0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);

        }

        /// <summary>Returns a float4x4 scale matrix given 3 axis scales.</summary>
        /// <param name="s">The uniform scaling factor.</param>
        /// <returns>The float4x4 matrix that represents a uniform scale.</returns>  
        public static float4x4 Scale(float s)
        {
            return math.float4x4(s,    0.0f, 0.0f, 0.0f,
                            0.0f, s,    0.0f, 0.0f,
                            0.0f, 0.0f, s,    0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);
        }

        /// <summary>Returns a float4x4 scale matrix given a float3 vector containing the 3 axis scales.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <param name="z">The z-axis scaling factor.</param>
        /// <returns>The float4x4 matrix that represents a non-uniform scale.</returns>  
        public static float4x4 Scale(float x, float y, float z)
        {
            return math.float4x4(x,    0.0f, 0.0f, 0.0f,
                            0.0f, y,    0.0f, 0.0f,
                            0.0f, 0.0f, z,    0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);
        }

        /// <summary>Returns a float4x4 scale matrix given a float3 vector containing the 3 axis scales.</summary>
        /// <param name="scales">The vector containing scale factors for each axis.</param>
        /// <returns>The float4x4 matrix that represents a non-uniform scale.</returns>  
        public static float4x4 Scale(float3 scales)
        {
            return Scale(scales.x, scales.y, scales.z);
        }

        /// <summary>Returns a float4x4 translation matrix given a float3 translation vector.</summary>
        /// <param name="vector">The translation vector.</param>
        /// <returns>The float4x4 translation matrix.</returns>  
        public static float4x4 Translate(float3 vector)
        {
            return math.float4x4(math.float4(1.0f, 0.0f, 0.0f, 0.0f),
                            math.float4(0.0f, 1.0f, 0.0f, 0.0f),
                            math.float4(0.0f, 0.0f, 1.0f, 0.0f),
                            math.float4(vector.x, vector.y, vector.z, 1.0f));
        }

        /// <summary>
        /// Returns a float4x4 view matrix given an eye position, a target point and a unit length up vector.
        /// The up vector is assumed to be unit length, the eye and target points are assumed to be distinct and
        /// the vector between them is assumes to be collinear with the up vector.
        /// If these assumptions are not met use float4x4.LookRotationSafe instead.
        /// </summary>
        /// <param name="eye">The eye position.</param>
        /// <param name="target">The view target position.</param>
        /// <param name="up">The eye up direction.</param>
        /// <returns>The float4x4 view matrix.</returns>  
        public static float4x4 LookAt(float3 eye, float3 target, float3 up)
        {
            float3x3 rot = float3x3.LookRotation(math.normalize(target - eye), up);

            float4x4 matrix;
            matrix.c0 = math.float4(rot.c0, 0.0F);
            matrix.c1 = math.float4(rot.c1, 0.0F);
            matrix.c2 = math.float4(rot.c2, 0.0F);
            matrix.c3 = math.float4(eye, 1.0F);
            return matrix;
        }

        /// <summary>
        /// Returns a float4x4 centered orthographic projection matrix.
        /// </summary>
        /// <param name="width">The width of the view volume.</param>
        /// <param name="height">The height of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        /// <returns>The float4x4 centered orthographic projection matrix.</returns>  
        public static float4x4 Ortho(float width, float height, float near, float far)
        {
            float rcpdx = 1.0f / width;
            float rcpdy = 1.0f / height;
            float rcpdz = 1.0f / (far - near);

            return math.float4x4(
                2.0f * rcpdx,   0.0f,            0.0f,           0.0f,
                0.0f,           2.0f * rcpdy,    0.0f,           0.0f,
                0.0f,           0.0f,           -2.0f * rcpdz,  -(far + near) * rcpdz,
                0.0f,           0.0f,            0.0f,           1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 off-center orthographic projection matrix.
        /// </summary>
        /// <param name="left">The math.minimum x-coordinate of the view volume.</param>
        /// <param name="right">The math.maximum x-coordinate of the view volume.</param>
        /// <param name="bottom">The math.minimum y-coordinate of the view volume.</param>
        /// <param name="top">The math.minimum y-coordinate of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        /// <returns>The float4x4 off-center orthographic projection matrix.</returns>  
        public static float4x4 OrthoOffCenter(float left, float right, float bottom, float top, float near, float far)
        {
            float rcpdx = 1.0f / (right - left);
            float rcpdy = 1.0f / (top - bottom);
            float rcpdz = 1.0f / (far - near);

            return math.float4x4(
                2.0f * rcpdx,   0.0f,           0.0f,               -(right + left) * rcpdx,
                0.0f,           2.0f * rcpdy,   0.0f,               -(top + bottom) * rcpdy,
                0.0f,           0.0f,          -2.0f * rcpdz,       -(far + near) * rcpdz,
                0.0f,           0.0f,           0.0f,                1.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 perspective projection matrix based on field of view.
        /// </summary>
        /// <param name="verticalFov">Vertical Field of view in radians.</param>
        /// <param name="aspect">X:Y aspect ratio.</param>
        /// <param name="near">Distance to near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to far plane. Must be greater than zero.</param>
        /// <returns>The float4x4 perspective projection matrix.</returns>  
        public static float4x4 PerspectiveFov(float verticalFov, float aspect, float near, float far)
        {
            float cotangent = 1.0f / math.tan(verticalFov * 0.5f);
            float rcpdz = 1.0f / (near - far);

            return math.float4x4(
                cotangent / aspect, 0.0f,       0.0f,                   0.0f,
                0.0f,               cotangent,  0.0f,                   0.0f,
                0.0f,               0.0f,       (far + near) * rcpdz,   2.0f * near * far * rcpdz,
                0.0f,               0.0f,      -1.0f,                   0.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 off-center perspective projection matrix.
        /// </summary>
        /// <param name="left">The x-coordinate of the left side of the clipping frustum at the near plane.</param>
        /// <param name="right">The x-coordinate of the right side of the clipping frustum at the near plane.</param>
        /// <param name="bottom">The y-coordinate of the bottom side of the clipping frustum at the near plane.</param>
        /// <param name="top">The y-coordinate of the top side of the clipping frustum at the near plane.</param>
        /// <param name="near">Distance to the near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to the far plane. Must be greater than zero.</param>
        /// <returns>The float4x4 off-center perspective projection matrix.</returns>  
        public static float4x4 PerspectiveOffCenter(float left, float right, float bottom, float top, float near, float far)
        {
            float rcpdz = 1.0f / (near - far);
            float rcpWidth = 1.0f / (right - left);
            float rcpHeight = 1.0f / (top - bottom);

            return math.float4x4(
                2.0f * near * rcpWidth,     0.0f,                       (left + right) * rcpWidth,     0.0f,
                0.0f,                       2.0f * near * rcpHeight,    (bottom + top) * rcpHeight,    0.0f,
                0.0f,                       0.0f,                        (far + near) * rcpdz,          2.0f * near * far * rcpdz,
                0.0f,                       0.0f,                       -1.0f,                          0.0f
                );
        }

        /// <summary>
        /// Returns a float4x4 matrix representing a combined scale-, rotation- and translation transform.
        /// Equivalent to mul(translationTransform, mul(rotationTransform, scaleTransform)).
        /// </summary>
        /// <param name="translation">The translation vector.</param>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="scale">The scaling factors of each axis.</param>
        /// <returns>The float4x4 matrix representing the translation, rotation, and scale by the inputs.</returns>  
        public static float4x4 TRS(float3 translation, quaternion rotation, float3 scale)
        {
            float3x3 r = math.float3x3(rotation);
            return math.float4x4(  math.float4(r.c0 * scale.x, 0.0f),
                              math.float4(r.c1 * scale.y, 0.0f),
                              math.float4(r.c2 * scale.z, 0.0f),
                              math.float4(translation, 1.0f));
        }
    }

    partial class math
    {
        /// <summary>
        /// Extracts a float3x3 from the upper left 3x3 of a float4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="float4x4"/> to extract a float3x3 from.</param>
        /// <returns>Upper left 3x3 matrix as float3x3.</returns>  
        public static float3x3 float3x3(float4x4 f4x4)
        {
            return new float3x3(f4x4);
        }

        /// <summary>Returns a float3x3 matrix constructed from a quaternion.</summary>
        /// <param name="rotation">The quaternion representing a rotation.</param>
        /// <returns>The float3x3 constructed from a quaternion.</returns>  
        public static float3x3 float3x3(quaternion rotation)
        {
            return new float3x3(rotation);
        }

        /// <summary>Returns a float4x4 constructed from a float3x3 rotation matrix and a float3 translation vector.</summary>
        /// <param name="rotation">The float3x3 rotation matrix.</param>
        /// <param name="translation">The translation vector.</param>
        /// <returns>The float4x4 constructed from a rotation and translation.</returns>  
        public static float4x4 float4x4(float3x3 rotation, float3 translation)
        {
            return new float4x4(rotation, translation);
        }

        /// <summary>Returns a float4x4 constructed from a quaternion and a float3 translation vector.</summary>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="translation">The translation vector.</param>
        /// <returns>The float4x4 constructed from a rotation and translation.</returns>  
        public static float4x4 float4x4(quaternion rotation, float3 translation)
        {
            return new float4x4(rotation, translation);
        }

        /// <summary>Returns a float4x4 constructed from a RigidTransform.</summary>
        /// <param name="transform">The rigid transformation.</param>
        /// <returns>The float4x4 constructed from a RigidTransform.</returns>  
        public static float4x4 float4x4(RigidTransform transform)
        {
            return new float4x4(transform);
        }

        /// <summary>Returns an orthomath.normalized version of a float3x3 matrix.</summary>
        /// <param name="i">The float3x3 to be orthomath.normalized.</param>
        /// <returns>The orthomath.normalized float3x3 matrix.</returns>  
        public static float3x3 normalize(float3x3 i)
        {
            float3x3 o;

            float3 u = i.c0;
            float3 v = i.c1 - i.c0 * math.dot(i.c1, i.c0);

            float lenU = math.length(u);
            float lenV = math.length(v);

            bool c = lenU > 1e-30f && lenV > 1e-30f;

            o.c0 = math.select(math.float3(1, 0, 0), u / lenU, c);
            o.c1 = math.select(math.float3(0, 1, 0), v / lenV, c);
            o.c2 = math.cross(o.c0, o.c1);

            return o;
        }
    }
}
