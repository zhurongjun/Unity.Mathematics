// GENERATED CODE
using System;
using System.Runtime.CompilerServices;

#pragma warning disable 0660, 0661

namespace Unity.Mathematics
{
    [System.Serializable]
    public partial struct float4x4 : System.IEquatable<float4x4>, IFormattable
    {
        public float4 c0;
        public float4 c1;
        public float4 c2;
        public float4 c3;

        /// <summary>float4x4 identity transform.</summary>
        public static readonly float4x4 identity = new float4x4(1.0f, 0.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f, 0.0f,   0.0f, 0.0f, 1.0f, 0.0f,   0.0f, 0.0f, 0.0f, 1.0f);

        /// <summary>float4x4 zero value.</summary>
        public static readonly float4x4 zero = new float4x4(0.0f, 0.0f, 0.0f, 0.0f,   0.0f, 0.0f, 0.0f, 0.0f,   0.0f, 0.0f, 0.0f, 0.0f,   0.0f, 0.0f, 0.0f, 0.0f);


        /// <summary>Constructs a float4x4 matrix from four float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(float4 c0, float4 c1, float4 c2, float4 c3)
        { 
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
        }

        /// <summary>Constructs a float4x4 matrix from 16 float values given in row-major order.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(float m00, float m01, float m02, float m03,
                        float m10, float m11, float m12, float m13,
                        float m20, float m21, float m22, float m23,
                        float m30, float m31, float m32, float m33)
        { 
            this.c0 = new float4(m00, m10, m20, m30);
            this.c1 = new float4(m01, m11, m21, m31);
            this.c2 = new float4(m02, m12, m22, m32);
            this.c3 = new float4(m03, m13, m23, m33);
        }

        /// <summary>Constructs a float4x4 matrix from a single float value by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(float v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
        }

        /// <summary>Constructs a float4x4 matrix from a single bool value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(bool v)
        {
            this.c0 = math.select(new float4(0.0f), new float4(1.0f), v);
            this.c1 = math.select(new float4(0.0f), new float4(1.0f), v);
            this.c2 = math.select(new float4(0.0f), new float4(1.0f), v);
            this.c3 = math.select(new float4(0.0f), new float4(1.0f), v);
        }

        /// <summary>Constructs a float4x4 matrix from a bool4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(bool4x4 v)
        {
            this.c0 = math.select(new float4(0.0f), new float4(1.0f), v.c0);
            this.c1 = math.select(new float4(0.0f), new float4(1.0f), v.c1);
            this.c2 = math.select(new float4(0.0f), new float4(1.0f), v.c2);
            this.c3 = math.select(new float4(0.0f), new float4(1.0f), v.c3);
        }

        /// <summary>Constructs a float4x4 matrix from a single int value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(int v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
        }

        /// <summary>Constructs a float4x4 matrix from a int4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(int4x4 v)
        {
            this.c0 = v.c0;
            this.c1 = v.c1;
            this.c2 = v.c2;
            this.c3 = v.c3;
        }

        /// <summary>Constructs a float4x4 matrix from a single uint value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(uint v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
        }

        /// <summary>Constructs a float4x4 matrix from a uint4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(uint4x4 v)
        {
            this.c0 = v.c0;
            this.c1 = v.c1;
            this.c2 = v.c2;
            this.c3 = v.c3;
        }

        /// <summary>Constructs a float4x4 matrix from a single double value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(double v)
        {
            this.c0 = (float4)v;
            this.c1 = (float4)v;
            this.c2 = (float4)v;
            this.c3 = (float4)v;
        }

        /// <summary>Constructs a float4x4 matrix from a double4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float4x4(double4x4 v)
        {
            this.c0 = (float4)v.c0;
            this.c1 = (float4)v.c1;
            this.c2 = (float4)v.c2;
            this.c3 = (float4)v.c3;
        }


        /// <summary>Implicitly converts a single float value to a float4x4 matrix by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(float v) { return new float4x4(v); }

        /// <summary>Explicitly converts a single bool value to a float4x4 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float4x4(bool v) { return new float4x4(v); }

        /// <summary>Explicitly converts a bool4x4 matrix to a float4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float4x4(bool4x4 v) { return new float4x4(v); }

        /// <summary>Implicitly converts a single int value to a float4x4 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(int v) { return new float4x4(v); }

        /// <summary>Implicitly converts a int4x4 matrix to a float4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(int4x4 v) { return new float4x4(v); }

        /// <summary>Implicitly converts a single uint value to a float4x4 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(uint v) { return new float4x4(v); }

        /// <summary>Implicitly converts a uint4x4 matrix to a float4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(uint4x4 v) { return new float4x4(v); }

        /// <summary>Explicitly converts a single double value to a float4x4 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float4x4(double v) { return new float4x4(v); }

        /// <summary>Explicitly converts a double4x4 matrix to a float4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float4x4(double4x4 v) { return new float4x4(v); }


        /// <summary>Returns the result of a componentwise multiplication operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator * (float4x4 lhs, float4x4 rhs) { return new float4x4 (lhs.c0 * rhs.c0, lhs.c1 * rhs.c1, lhs.c2 * rhs.c2, lhs.c3 * rhs.c3); }

        /// <summary>Returns the result of a componentwise multiplication operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator * (float4x4 lhs, float rhs) { return new float4x4 (lhs.c0 * rhs, lhs.c1 * rhs, lhs.c2 * rhs, lhs.c3 * rhs); }

        /// <summary>Returns the result of a componentwise multiplication operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator * (float lhs, float4x4 rhs) { return new float4x4 (lhs * rhs.c0, lhs * rhs.c1, lhs * rhs.c2, lhs * rhs.c3); }


        /// <summary>Returns the result of a componentwise addition operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator + (float4x4 lhs, float4x4 rhs) { return new float4x4 (lhs.c0 + rhs.c0, lhs.c1 + rhs.c1, lhs.c2 + rhs.c2, lhs.c3 + rhs.c3); }

        /// <summary>Returns the result of a componentwise addition operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator + (float4x4 lhs, float rhs) { return new float4x4 (lhs.c0 + rhs, lhs.c1 + rhs, lhs.c2 + rhs, lhs.c3 + rhs); }

        /// <summary>Returns the result of a componentwise addition operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator + (float lhs, float4x4 rhs) { return new float4x4 (lhs + rhs.c0, lhs + rhs.c1, lhs + rhs.c2, lhs + rhs.c3); }


        /// <summary>Returns the result of a componentwise subtraction operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator - (float4x4 lhs, float4x4 rhs) { return new float4x4 (lhs.c0 - rhs.c0, lhs.c1 - rhs.c1, lhs.c2 - rhs.c2, lhs.c3 - rhs.c3); }

        /// <summary>Returns the result of a componentwise subtraction operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator - (float4x4 lhs, float rhs) { return new float4x4 (lhs.c0 - rhs, lhs.c1 - rhs, lhs.c2 - rhs, lhs.c3 - rhs); }

        /// <summary>Returns the result of a componentwise subtraction operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator - (float lhs, float4x4 rhs) { return new float4x4 (lhs - rhs.c0, lhs - rhs.c1, lhs - rhs.c2, lhs - rhs.c3); }


        /// <summary>Returns the result of a componentwise division operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator / (float4x4 lhs, float4x4 rhs) { return new float4x4 (lhs.c0 / rhs.c0, lhs.c1 / rhs.c1, lhs.c2 / rhs.c2, lhs.c3 / rhs.c3); }

        /// <summary>Returns the result of a componentwise division operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator / (float4x4 lhs, float rhs) { return new float4x4 (lhs.c0 / rhs, lhs.c1 / rhs, lhs.c2 / rhs, lhs.c3 / rhs); }

        /// <summary>Returns the result of a componentwise division operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator / (float lhs, float4x4 rhs) { return new float4x4 (lhs / rhs.c0, lhs / rhs.c1, lhs / rhs.c2, lhs / rhs.c3); }


        /// <summary>Returns the result of a componentwise modulus operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator % (float4x4 lhs, float4x4 rhs) { return new float4x4 (lhs.c0 % rhs.c0, lhs.c1 % rhs.c1, lhs.c2 % rhs.c2, lhs.c3 % rhs.c3); }

        /// <summary>Returns the result of a componentwise modulus operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator % (float4x4 lhs, float rhs) { return new float4x4 (lhs.c0 % rhs, lhs.c1 % rhs, lhs.c2 % rhs, lhs.c3 % rhs); }

        /// <summary>Returns the result of a componentwise modulus operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator % (float lhs, float4x4 rhs) { return new float4x4 (lhs % rhs.c0, lhs % rhs.c1, lhs % rhs.c2, lhs % rhs.c3); }


        /// <summary>Returns the result of a componentwise increment operation on a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator ++ (float4x4 val) { return new float4x4 (++val.c0, ++val.c1, ++val.c2, ++val.c3); }


        /// <summary>Returns the result of a componentwise decrement operation on a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator -- (float4x4 val) { return new float4x4 (--val.c0, --val.c1, --val.c2, --val.c3); }


        /// <summary>Returns the result of a componentwise less than operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 < rhs.c0, lhs.c1 < rhs.c1, lhs.c2 < rhs.c2, lhs.c3 < rhs.c3); }

        /// <summary>Returns the result of a componentwise less than operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 < rhs, lhs.c1 < rhs, lhs.c2 < rhs, lhs.c3 < rhs); }

        /// <summary>Returns the result of a componentwise less than operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator < (float lhs, float4x4 rhs) { return new bool4x4 (lhs < rhs.c0, lhs < rhs.c1, lhs < rhs.c2, lhs < rhs.c3); }


        /// <summary>Returns the result of a componentwise less or equal operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 <= rhs.c0, lhs.c1 <= rhs.c1, lhs.c2 <= rhs.c2, lhs.c3 <= rhs.c3); }

        /// <summary>Returns the result of a componentwise less or equal operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 <= rhs, lhs.c1 <= rhs, lhs.c2 <= rhs, lhs.c3 <= rhs); }

        /// <summary>Returns the result of a componentwise less or equal operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator <= (float lhs, float4x4 rhs) { return new bool4x4 (lhs <= rhs.c0, lhs <= rhs.c1, lhs <= rhs.c2, lhs <= rhs.c3); }


        /// <summary>Returns the result of a componentwise greater than operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 > rhs.c0, lhs.c1 > rhs.c1, lhs.c2 > rhs.c2, lhs.c3 > rhs.c3); }

        /// <summary>Returns the result of a componentwise greater than operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 > rhs, lhs.c1 > rhs, lhs.c2 > rhs, lhs.c3 > rhs); }

        /// <summary>Returns the result of a componentwise greater than operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator > (float lhs, float4x4 rhs) { return new bool4x4 (lhs > rhs.c0, lhs > rhs.c1, lhs > rhs.c2, lhs > rhs.c3); }


        /// <summary>Returns the result of a componentwise greater or equal operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 >= rhs.c0, lhs.c1 >= rhs.c1, lhs.c2 >= rhs.c2, lhs.c3 >= rhs.c3); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 >= rhs, lhs.c1 >= rhs, lhs.c2 >= rhs, lhs.c3 >= rhs); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator >= (float lhs, float4x4 rhs) { return new bool4x4 (lhs >= rhs.c0, lhs >= rhs.c1, lhs >= rhs.c2, lhs >= rhs.c3); }


        /// <summary>Returns the result of a componentwise unary minus operation on a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator - (float4x4 val) { return new float4x4 (-val.c0, -val.c1, -val.c2, -val.c3); }


        /// <summary>Returns the result of a componentwise unary plus operation on a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 operator + (float4x4 val) { return new float4x4 (+val.c0, +val.c1, +val.c2, +val.c3); }


        /// <summary>Returns the result of a componentwise equality operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 == rhs.c0, lhs.c1 == rhs.c1, lhs.c2 == rhs.c2, lhs.c3 == rhs.c3); }

        /// <summary>Returns the result of a componentwise equality operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 == rhs, lhs.c1 == rhs, lhs.c2 == rhs, lhs.c3 == rhs); }

        /// <summary>Returns the result of a componentwise equality operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator == (float lhs, float4x4 rhs) { return new bool4x4 (lhs == rhs.c0, lhs == rhs.c1, lhs == rhs.c2, lhs == rhs.c3); }


        /// <summary>Returns the result of a componentwise not equal operation on two float4x4 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (float4x4 lhs, float4x4 rhs) { return new bool4x4 (lhs.c0 != rhs.c0, lhs.c1 != rhs.c1, lhs.c2 != rhs.c2, lhs.c3 != rhs.c3); }

        /// <summary>Returns the result of a componentwise not equal operation on a float4x4 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (float4x4 lhs, float rhs) { return new bool4x4 (lhs.c0 != rhs, lhs.c1 != rhs, lhs.c2 != rhs, lhs.c3 != rhs); }

        /// <summary>Returns the result of a componentwise not equal operation on a float value and a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool4x4 operator != (float lhs, float4x4 rhs) { return new bool4x4 (lhs != rhs.c0, lhs != rhs.c1, lhs != rhs.c2, lhs != rhs.c3); }



        /// <summary>Returns the float4 element at a specified index.</summary>
        unsafe public float4 this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 4)
                    throw new System.ArgumentException("index must be between[0...3]");
#endif
                fixed (float4x4* array = &this) { return ((float4*)array)[index]; }
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 4)
                    throw new System.ArgumentException("index must be between[0...3]");
#endif
                fixed (float4* array = &c0) { array[index] = value; }
            }
        }

        /// <summary>Returns true if the float4x4 is equal to a given float4x4, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(float4x4 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2) && c3.Equals(rhs.c3); }

        /// <summary>Returns true if the float4x4 is equal to a given float4x4, false otherwise.</summary>
        public override bool Equals(object o) { return Equals((float4x4)o); }


        /// <summary>Returns a hash code for the float4x4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() { return (int)math.hash(this); }


        /// <summary>Returns a string representation of the float4x4.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("float4x4({0}f, {1}f, {2}f, {3}f,  {4}f, {5}f, {6}f, {7}f,  {8}f, {9}f, {10}f, {11}f,  {12}f, {13}f, {14}f, {15}f)", c0.x, c1.x, c2.x, c3.x, c0.y, c1.y, c2.y, c3.y, c0.z, c1.z, c2.z, c3.z, c0.w, c1.w, c2.w, c3.w);
        }

        /// <summary>Returns a string representation of the float4x4 using a specified format and culture-specific format information.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("float4x4({0}f, {1}f, {2}f, {3}f,  {4}f, {5}f, {6}f, {7}f,  {8}f, {9}f, {10}f, {11}f,  {12}f, {13}f, {14}f, {15}f)", c0.x.ToString(format, formatProvider), c1.x.ToString(format, formatProvider), c2.x.ToString(format, formatProvider), c3.x.ToString(format, formatProvider), c0.y.ToString(format, formatProvider), c1.y.ToString(format, formatProvider), c2.y.ToString(format, formatProvider), c3.y.ToString(format, formatProvider), c0.z.ToString(format, formatProvider), c1.z.ToString(format, formatProvider), c2.z.ToString(format, formatProvider), c3.z.ToString(format, formatProvider), c0.w.ToString(format, formatProvider), c1.w.ToString(format, formatProvider), c2.w.ToString(format, formatProvider), c3.w.ToString(format, formatProvider));
        }

    }

    public static partial class math
    {
        /// <summary>Returns a float4x4 matrix constructed from four float4 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(float4 c0, float4 c1, float4 c2, float4 c3) { return new float4x4(c0, c1, c2, c3); }

        /// <summary>Returns a float4x4 matrix constructed from from 16 float values given in row-major order.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(float m00, float m01, float m02, float m03,
                                        float m10, float m11, float m12, float m13,
                                        float m20, float m21, float m22, float m23,
                                        float m30, float m31, float m32, float m33)
        {
            return new float4x4(m00, m01, m02, m03,
                                m10, m11, m12, m13,
                                m20, m21, m22, m23,
                                m30, m31, m32, m33);
        }

        /// <summary>Returns a float4x4 matrix constructed from a single float value by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(float v) { return new float4x4(v); }

        /// <summary>Returns a float4x4 matrix constructed from a single bool value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(bool v) { return new float4x4(v); }

        /// <summary>Return a float4x4 matrix constructed from a bool4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(bool4x4 v) { return new float4x4(v); }

        /// <summary>Returns a float4x4 matrix constructed from a single int value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(int v) { return new float4x4(v); }

        /// <summary>Return a float4x4 matrix constructed from a int4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(int4x4 v) { return new float4x4(v); }

        /// <summary>Returns a float4x4 matrix constructed from a single uint value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(uint v) { return new float4x4(v); }

        /// <summary>Return a float4x4 matrix constructed from a uint4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(uint4x4 v) { return new float4x4(v); }

        /// <summary>Returns a float4x4 matrix constructed from a single double value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(double v) { return new float4x4(v); }

        /// <summary>Return a float4x4 matrix constructed from a double4x4 matrix by componentwise conversion.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 float4x4(double4x4 v) { return new float4x4(v); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 rotate(float4x4 a, float3 b)
        {
            return (a.c0 * b.x + a.c1 * b.y + a.c2 * b.z).xyz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 transform(float4x4 a, float3 b)
        {
            return (a.c0 * b.x + a.c1 * b.y + a.c2 * b.z + a.c3).xyz;
        }

        /// <summary>Return the float4x4 transpose of a float4x4 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float4x4 transpose(float4x4 v)
        {
            return float4x4(
                v.c0.x, v.c0.y, v.c0.z, v.c0.w,
                v.c1.x, v.c1.y, v.c1.z, v.c1.w,
                v.c2.x, v.c2.y, v.c2.z, v.c2.w,
                v.c3.x, v.c3.y, v.c3.z, v.c3.w);
        }

        /// <summary>Returns the float4x4 full inverse of a float4x4 matrix.</summary>
        public static float4x4 inverse(float4x4 m)
        {
            float4 c0 = m.c0;
            float4 c1 = m.c1;
            float4 c2 = m.c2;
            float4 c3 = m.c3;

            float4 r0y_r1y_r0x_r1x = movelh(c1, c0);
            float4 r0z_r1z_r0w_r1w = movelh(c2, c3);
            float4 r2y_r3y_r2x_r3x = movehl(c0, c1);
            float4 r2z_r3z_r2w_r3w = movehl(c3, c2);

            float4 r1y_r2y_r1x_r2x = shuffle(c1, c0, ShuffleComponent.LeftY, ShuffleComponent.LeftZ, ShuffleComponent.RightY, ShuffleComponent.RightZ);
            float4 r1z_r2z_r1w_r2w = shuffle(c2, c3, ShuffleComponent.LeftY, ShuffleComponent.LeftZ, ShuffleComponent.RightY, ShuffleComponent.RightZ);
            float4 r3y_r0y_r3x_r0x = shuffle(c1, c0, ShuffleComponent.LeftW, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightX);
            float4 r3z_r0z_r3w_r0w = shuffle(c2, c3, ShuffleComponent.LeftW, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightX);

            float4 r0_wzyx = shuffle(r0z_r1z_r0w_r1w, r0y_r1y_r0x_r1x, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);
            float4 r1_wzyx = shuffle(r0z_r1z_r0w_r1w, r0y_r1y_r0x_r1x, ShuffleComponent.LeftW, ShuffleComponent.LeftY, ShuffleComponent.RightY, ShuffleComponent.RightW);
            float4 r2_wzyx = shuffle(r2z_r3z_r2w_r3w, r2y_r3y_r2x_r3x, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);
            float4 r3_wzyx = shuffle(r2z_r3z_r2w_r3w, r2y_r3y_r2x_r3x, ShuffleComponent.LeftW, ShuffleComponent.LeftY, ShuffleComponent.RightY, ShuffleComponent.RightW);
            float4 r0_xyzw = shuffle(r0y_r1y_r0x_r1x, r0z_r1z_r0w_r1w, ShuffleComponent.LeftZ, ShuffleComponent.LeftX, ShuffleComponent.RightX, ShuffleComponent.RightZ);

            // Calculate remaining inner term pairs. inner terms have zw=-xy, so we only have to calculate xy and can pack two pairs per vector.
            float4 inner12_23 = r1y_r2y_r1x_r2x * r2z_r3z_r2w_r3w - r1z_r2z_r1w_r2w * r2y_r3y_r2x_r3x;
            float4 inner02_13 = r0y_r1y_r0x_r1x * r2z_r3z_r2w_r3w - r0z_r1z_r0w_r1w * r2y_r3y_r2x_r3x;
            float4 inner30_01 = r3z_r0z_r3w_r0w * r0y_r1y_r0x_r1x - r3y_r0y_r3x_r0x * r0z_r1z_r0w_r1w;

            // Expand inner terms back to 4 components. zw signs still need to be flipped
            float4 inner12 = shuffle(inner12_23, inner12_23, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            float4 inner23 = shuffle(inner12_23, inner12_23, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            float4 inner02 = shuffle(inner02_13, inner02_13, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            float4 inner13 = shuffle(inner02_13, inner02_13, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            // Calculate minors
            float4 minors0 = r3_wzyx * inner12 - r2_wzyx * inner13 + r1_wzyx * inner23;

            float4 denom = r0_xyzw * minors0;

            // Horizontal sum of denominator. Free sign flip of z and w compensates for missing flip in inner terms.
            denom = denom + shuffle(denom, denom, ShuffleComponent.LeftY, ShuffleComponent.LeftX, ShuffleComponent.RightW, ShuffleComponent.RightZ);   // x+y        x+y            z+w            z+w
            denom = denom - shuffle(denom, denom, ShuffleComponent.LeftZ, ShuffleComponent.LeftZ, ShuffleComponent.RightX, ShuffleComponent.RightX);   // x+y-z-w  x+y-z-w        z+w-x-y        z+w-x-y

            float4 rcp_denom_ppnn = float4(1.0f) / denom;
            float4x4 res;
            res.c0 = minors0 * rcp_denom_ppnn;

            float4 inner30 = shuffle(inner30_01, inner30_01, ShuffleComponent.LeftX, ShuffleComponent.LeftZ, ShuffleComponent.RightZ, ShuffleComponent.RightX);
            float4 inner01 = shuffle(inner30_01, inner30_01, ShuffleComponent.LeftY, ShuffleComponent.LeftW, ShuffleComponent.RightW, ShuffleComponent.RightY);

            float4 minors1 = r2_wzyx * inner30 - r0_wzyx * inner23 - r3_wzyx * inner02;
            res.c1 = minors1 * rcp_denom_ppnn;

            float4 minors2 = r0_wzyx * inner13 - r1_wzyx * inner30 - r3_wzyx * inner01;
            res.c2 = minors2 * rcp_denom_ppnn;

            float4 minors3 = r1_wzyx * inner02 - r0_wzyx * inner12 + r2_wzyx * inner01;
            res.c3 = minors3 * rcp_denom_ppnn;
            return res;
        }

        // Fast matrix inverse for rigid transforms (Orthonormal basis and translation)
        public static float4x4 fastinverse(float4x4 m)
        {
            float4 c0 = m.c0;
            float4 c1 = m.c1;
            float4 c2 = m.c2;
            float4 pos = m.c3;

            float4 zero = float4(0);

            float4 t0 = unpacklo(c0, c2);
            float4 t1 = unpacklo(c1, zero);
            float4 t2 = unpackhi(c0, c2);
            float4 t3 = unpackhi(c1, zero);

            float4 r0 = unpacklo(t0, t1);
            float4 r1 = unpackhi(t0, t1);
            float4 r2 = unpacklo(t2, t3);

            pos = -(r0 * pos.x + r1 * pos.y + r2 * pos.z);
            pos.w = 1.0f;

            return float4x4(r0, r1, r2, pos);
        }

        /// <summary>Returns the determinant of a float4x4 matrix.</summary>
        public static float determinant(float4x4 m)
        {
            float4 c0 = m.c0;
            float4 c1 = m.c1;
            float4 c2 = m.c2;
            float4 c3 = m.c3;

            float m00 = c1.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c1.z * c3.w - c1.w * c3.z) + c3.y * (c1.z * c2.w - c1.w * c2.z);
            float m01 = c0.y * (c2.z * c3.w - c2.w * c3.z) - c2.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c2.w - c0.w * c2.z);
            float m02 = c0.y * (c1.z * c3.w - c1.w * c3.z) - c1.y * (c0.z * c3.w - c0.w * c3.z) + c3.y * (c0.z * c1.w - c0.w * c1.z);
            float m03 = c0.y * (c1.z * c2.w - c1.w * c2.z) - c1.y * (c0.z * c2.w - c0.w * c2.z) + c2.y * (c0.z * c1.w - c0.w * c1.z);

            return c0.x * m00 - c1.x * m01 + c2.x * m02 - c3.x * m03;
        }

        /// <summary>Returns a uint hash code of a float4x4 vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint hash(float4x4 v)
        {
            return csum(asuint(v.c0) * uint4(0x515D90F5u, 0xEC9F68F3u, 0xF9EA92D5u, 0xC2FAFCB9u) + 
                        asuint(v.c1) * uint4(0x616E9CA1u, 0xC5C5394Bu, 0xCAE78587u, 0x7A1541C9u) + 
                        asuint(v.c2) * uint4(0xF83BD927u, 0x6A243BCBu, 0x509B84C9u, 0x91D13847u) + 
                        asuint(v.c3) * uint4(0x52F7230Fu, 0xCF286E83u, 0xE121E6ADu, 0xC9CA1249u)) + 0x69B60C81u;
        }

        /// <summary>
        /// Returns a uint4 vector hash code of a float4x4 vector.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 hashwide(float4x4 v)
        {
            return (asuint(v.c0) * uint4(0xE0EB6C25u, 0xF648BEABu, 0x6BDB2B07u, 0xEF63C699u) + 
                    asuint(v.c1) * uint4(0x9001903Fu, 0xA895B9CDu, 0x9D23B201u, 0x4B01D3E1u) + 
                    asuint(v.c2) * uint4(0x7461CA0Du, 0x79725379u, 0xD6258E5Bu, 0xEE390C97u) + 
                    asuint(v.c3) * uint4(0x9C8A2F05u, 0x4DDC6509u, 0x7CF083CBu, 0x5C4D6CEDu)) + 0xF9137117u;
        }

    }
}
