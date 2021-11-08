//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated. To update the generation of this file, modify and re-run Unity.Mathematics.CodeGen.
// </auto-generated>
//------------------------------------------------------------------------------
using System;
using System.Runtime.CompilerServices;
using Unity.IL2CPP.CompilerServices;

#pragma warning disable 0660, 0661

namespace Unity.Mathematics
{
    /// <summary>A 4x3 matrix of bools.</summary>
    [System.Serializable]
    [Il2CppEagerStaticClassConstruction]
    public partial struct bool4x3 : System.IEquatable<bool4x3>
    {
        /// <summary>Column 0 of the matrix.</summary>
        public bool4 c0;
        /// <summary>Column 1 of the matrix.</summary>
        public bool4 c1;
        /// <summary>Column 2 of the matrix.</summary>
        public bool4 c2;


        /// <summary>Constructs a bool4x3 matrix from three bool4 vectors.</summary>
        /// <param name="c0">The matrix column c0 will be set to this value.</param>
        /// <param name="c1">The matrix column c1 will be set to this value.</param>
        /// <param name="c2">The matrix column c2 will be set to this value.</param>
        public bool4x3(bool4 c0, bool4 c1, bool4 c2)
        {
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
        }

        /// <summary>Constructs a bool4x3 matrix from 12 bool values given in row-major order.</summary>
        /// <param name="m00">The matrix at row 0, column 0 will be set to this value.</param>
        /// <param name="m01">The matrix at row 0, column 1 will be set to this value.</param>
        /// <param name="m02">The matrix at row 0, column 2 will be set to this value.</param>
        /// <param name="m10">The matrix at row 1, column 0 will be set to this value.</param>
        /// <param name="m11">The matrix at row 1, column 1 will be set to this value.</param>
        /// <param name="m12">The matrix at row 1, column 2 will be set to this value.</param>
        /// <param name="m20">The matrix at row 2, column 0 will be set to this value.</param>
        /// <param name="m21">The matrix at row 2, column 1 will be set to this value.</param>
        /// <param name="m22">The matrix at row 2, column 2 will be set to this value.</param>
        /// <param name="m30">The matrix at row 3, column 0 will be set to this value.</param>
        /// <param name="m31">The matrix at row 3, column 1 will be set to this value.</param>
        /// <param name="m32">The matrix at row 3, column 2 will be set to this value.</param>
        public bool4x3(bool m00, bool m01, bool m02,
                       bool m10, bool m11, bool m12,
                       bool m20, bool m21, bool m22,
                       bool m30, bool m31, bool m32)
        {
            this.c0 = new bool4(m00, m10, m20, m30);
            this.c1 = new bool4(m01, m11, m21, m31);
            this.c2 = new bool4(m02, m12, m22, m32);
        }

        /// <summary>Constructs a bool4x3 matrix from a single bool value by assigning it to every component.</summary>
        /// <param name="v">bool to convert to bool4x3</param>
        public bool4x3(bool v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
        }


        /// <summary>Implicitly converts a single bool value to a bool4x3 matrix by assigning it to every component.</summary>
        /// <param name="v">bool to convert to bool4x3</param>
        /// <returns>Converted value.</returns>
        public static implicit operator bool4x3(bool v) { return new bool4x3(v); }


        /// <summary>Returns the result of a componentwise equality operation on two bool4x3 matrices.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise equality.</param>
        /// <returns>bool4x3 result of the componentwise equality.</returns>
        public static bool4x3 operator == (bool4x3 lhs, bool4x3 rhs) { return new bool4x3 (lhs.c0 == rhs.c0, lhs.c1 == rhs.c1, lhs.c2 == rhs.c2); }

        /// <summary>Returns the result of a componentwise equality operation on a bool4x3 matrix and a bool value.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side bool to use to compute componentwise equality.</param>
        /// <returns>bool4x3 result of the componentwise equality.</returns>
        public static bool4x3 operator == (bool4x3 lhs, bool rhs) { return new bool4x3 (lhs.c0 == rhs, lhs.c1 == rhs, lhs.c2 == rhs); }

        /// <summary>Returns the result of a componentwise equality operation on a bool value and a bool4x3 matrix.</summary>
        /// <param name="lhs">Left hand side bool to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise equality.</param>
        /// <returns>bool4x3 result of the componentwise equality.</returns>
        public static bool4x3 operator == (bool lhs, bool4x3 rhs) { return new bool4x3 (lhs == rhs.c0, lhs == rhs.c1, lhs == rhs.c2); }


        /// <summary>Returns the result of a componentwise not equal operation on two bool4x3 matrices.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise not equal.</param>
        /// <returns>bool4x3 result of the componentwise not equal.</returns>
        public static bool4x3 operator != (bool4x3 lhs, bool4x3 rhs) { return new bool4x3 (lhs.c0 != rhs.c0, lhs.c1 != rhs.c1, lhs.c2 != rhs.c2); }

        /// <summary>Returns the result of a componentwise not equal operation on a bool4x3 matrix and a bool value.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side bool to use to compute componentwise not equal.</param>
        /// <returns>bool4x3 result of the componentwise not equal.</returns>
        public static bool4x3 operator != (bool4x3 lhs, bool rhs) { return new bool4x3 (lhs.c0 != rhs, lhs.c1 != rhs, lhs.c2 != rhs); }

        /// <summary>Returns the result of a componentwise not equal operation on a bool value and a bool4x3 matrix.</summary>
        /// <param name="lhs">Left hand side bool to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise not equal.</param>
        /// <returns>bool4x3 result of the componentwise not equal.</returns>
        public static bool4x3 operator != (bool lhs, bool4x3 rhs) { return new bool4x3 (lhs != rhs.c0, lhs != rhs.c1, lhs != rhs.c2); }


        /// <summary>Returns the result of a componentwise not operation on a bool4x3 matrix.</summary>
        /// <param name="val">Value to use when computing the componentwise not.</param>
        /// <returns>bool4x3 result of the componentwise not.</returns>
        public static bool4x3 operator ! (bool4x3 val) { return new bool4x3 (!val.c0, !val.c1, !val.c2); }


        /// <summary>Returns the result of a componentwise bitwise and operation on two bool4x3 matrices.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise and.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise and.</param>
        /// <returns>bool4x3 result of the componentwise bitwise and.</returns>
        public static bool4x3 operator & (bool4x3 lhs, bool4x3 rhs) { return new bool4x3 (lhs.c0 & rhs.c0, lhs.c1 & rhs.c1, lhs.c2 & rhs.c2); }

        /// <summary>Returns the result of a componentwise bitwise and operation on a bool4x3 matrix and a bool value.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise and.</param>
        /// <param name="rhs">Right hand side bool to use to compute componentwise bitwise and.</param>
        /// <returns>bool4x3 result of the componentwise bitwise and.</returns>
        public static bool4x3 operator & (bool4x3 lhs, bool rhs) { return new bool4x3 (lhs.c0 & rhs, lhs.c1 & rhs, lhs.c2 & rhs); }

        /// <summary>Returns the result of a componentwise bitwise and operation on a bool value and a bool4x3 matrix.</summary>
        /// <param name="lhs">Left hand side bool to use to compute componentwise bitwise and.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise and.</param>
        /// <returns>bool4x3 result of the componentwise bitwise and.</returns>
        public static bool4x3 operator & (bool lhs, bool4x3 rhs) { return new bool4x3 (lhs & rhs.c0, lhs & rhs.c1, lhs & rhs.c2); }


        /// <summary>Returns the result of a componentwise bitwise or operation on two bool4x3 matrices.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise or.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise or.</returns>
        public static bool4x3 operator | (bool4x3 lhs, bool4x3 rhs) { return new bool4x3 (lhs.c0 | rhs.c0, lhs.c1 | rhs.c1, lhs.c2 | rhs.c2); }

        /// <summary>Returns the result of a componentwise bitwise or operation on a bool4x3 matrix and a bool value.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise or.</param>
        /// <param name="rhs">Right hand side bool to use to compute componentwise bitwise or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise or.</returns>
        public static bool4x3 operator | (bool4x3 lhs, bool rhs) { return new bool4x3 (lhs.c0 | rhs, lhs.c1 | rhs, lhs.c2 | rhs); }

        /// <summary>Returns the result of a componentwise bitwise or operation on a bool value and a bool4x3 matrix.</summary>
        /// <param name="lhs">Left hand side bool to use to compute componentwise bitwise or.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise or.</returns>
        public static bool4x3 operator | (bool lhs, bool4x3 rhs) { return new bool4x3 (lhs | rhs.c0, lhs | rhs.c1, lhs | rhs.c2); }


        /// <summary>Returns the result of a componentwise bitwise exclusive or operation on two bool4x3 matrices.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise exclusive or.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise exclusive or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise exclusive or.</returns>
        public static bool4x3 operator ^ (bool4x3 lhs, bool4x3 rhs) { return new bool4x3 (lhs.c0 ^ rhs.c0, lhs.c1 ^ rhs.c1, lhs.c2 ^ rhs.c2); }

        /// <summary>Returns the result of a componentwise bitwise exclusive or operation on a bool4x3 matrix and a bool value.</summary>
        /// <param name="lhs">Left hand side bool4x3 to use to compute componentwise bitwise exclusive or.</param>
        /// <param name="rhs">Right hand side bool to use to compute componentwise bitwise exclusive or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise exclusive or.</returns>
        public static bool4x3 operator ^ (bool4x3 lhs, bool rhs) { return new bool4x3 (lhs.c0 ^ rhs, lhs.c1 ^ rhs, lhs.c2 ^ rhs); }

        /// <summary>Returns the result of a componentwise bitwise exclusive or operation on a bool value and a bool4x3 matrix.</summary>
        /// <param name="lhs">Left hand side bool to use to compute componentwise bitwise exclusive or.</param>
        /// <param name="rhs">Right hand side bool4x3 to use to compute componentwise bitwise exclusive or.</param>
        /// <returns>bool4x3 result of the componentwise bitwise exclusive or.</returns>
        public static bool4x3 operator ^ (bool lhs, bool4x3 rhs) { return new bool4x3 (lhs ^ rhs.c0, lhs ^ rhs.c1, lhs ^ rhs.c2); }



        /// <summary>Returns the bool4 element at a specified index.</summary>
        unsafe public bool4 this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 3)
                    throw new System.ArgumentException("index must be between[0...2]");
#endif
                fixed (bool4x3* array = &this) { return ((bool4*)array)[index]; }
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 3)
                    throw new System.ArgumentException("index must be between[0...2]");
#endif
                fixed (bool4* array = &c0) { array[index] = value; }
            }
        }

        /// <summary>Returns true if the bool4x3 is equal to a given bool4x3, false otherwise.</summary>
        /// <param name="rhs">Right hand side argument to compare equality with.</param>
        /// <returns>The result of the equality comparison.</returns>
        public bool Equals(bool4x3 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2); }

        /// <summary>Returns true if the bool4x3 is equal to a given bool4x3, false otherwise.</summary>
        /// <param name="o">Right hand side argument to compare equality with.</param>
        /// <returns>The result of the equality comparison.</returns>
        public override bool Equals(object o) { return o is bool4x3 && Equals((bool4x3)o); }


        /// <summary>Returns a hash code for the bool4x3.</summary>
        /// <returns>The computed hash code.</returns>
        public override int GetHashCode() { return (int)math.hash(this); }


        /// <summary>Returns a string representation of the bool4x3.</summary>
        /// <returns>String representation of the value.</returns>
        public override string ToString()
        {
            return string.Format("bool4x3({0}, {1}, {2},  {3}, {4}, {5},  {6}, {7}, {8},  {9}, {10}, {11})", c0.x, c1.x, c2.x, c0.y, c1.y, c2.y, c0.z, c1.z, c2.z, c0.w, c1.w, c2.w);
        }

    }

    public static partial class math
    {
        /// <summary>Returns a bool4x3 matrix constructed from three bool4 vectors.</summary>
        /// <param name="c0">The matrix column c0 will be set to this value.</param>
        /// <param name="c1">The matrix column c1 will be set to this value.</param>
        /// <param name="c2">The matrix column c2 will be set to this value.</param>
        /// <returns>bool4x3 constructed from arguments.</returns>
        public static bool4x3 bool4x3(bool4 c0, bool4 c1, bool4 c2) { return new bool4x3(c0, c1, c2); }

        /// <summary>Returns a bool4x3 matrix constructed from from 12 bool values given in row-major order.</summary>
        /// <param name="m00">The matrix at row 0, column 0 will be set to this value.</param>
        /// <param name="m01">The matrix at row 0, column 1 will be set to this value.</param>
        /// <param name="m02">The matrix at row 0, column 2 will be set to this value.</param>
        /// <param name="m10">The matrix at row 1, column 0 will be set to this value.</param>
        /// <param name="m11">The matrix at row 1, column 1 will be set to this value.</param>
        /// <param name="m12">The matrix at row 1, column 2 will be set to this value.</param>
        /// <param name="m20">The matrix at row 2, column 0 will be set to this value.</param>
        /// <param name="m21">The matrix at row 2, column 1 will be set to this value.</param>
        /// <param name="m22">The matrix at row 2, column 2 will be set to this value.</param>
        /// <param name="m30">The matrix at row 3, column 0 will be set to this value.</param>
        /// <param name="m31">The matrix at row 3, column 1 will be set to this value.</param>
        /// <param name="m32">The matrix at row 3, column 2 will be set to this value.</param>
        /// <returns>bool4x3 constructed from arguments.</returns>
        public static bool4x3 bool4x3(bool m00, bool m01, bool m02,
                                      bool m10, bool m11, bool m12,
                                      bool m20, bool m21, bool m22,
                                      bool m30, bool m31, bool m32)
        {
            return new bool4x3(m00, m01, m02,
                               m10, m11, m12,
                               m20, m21, m22,
                               m30, m31, m32);
        }

        /// <summary>Returns a bool4x3 matrix constructed from a single bool value by assigning it to every component.</summary>
        /// <param name="v">bool to convert to bool4x3</param>
        /// <returns>Converted value.</returns>
        public static bool4x3 bool4x3(bool v) { return new bool4x3(v); }

        /// <summary>Return the bool3x4 transpose of a bool4x3 matrix.</summary>
        /// <param name="v">Value to transpose.</param>
        /// <returns>Transposed value.</returns>
        public static bool3x4 transpose(bool4x3 v)
        {
            return bool3x4(
                v.c0.x, v.c0.y, v.c0.z, v.c0.w,
                v.c1.x, v.c1.y, v.c1.z, v.c1.w,
                v.c2.x, v.c2.y, v.c2.z, v.c2.w);
        }

        /// <summary>Returns a uint hash code of a bool4x3 matrix.</summary>
        /// <param name="v">Matrix value to hash.</param>
        /// <returns>uint hash of the argument.</returns>
        public static uint hash(bool4x3 v)
        {
            return csum(select(uint4(0xEADF0775u, 0x747A9D7Bu, 0x4111F799u, 0xB5F05AF1u), uint4(0xFD80290Bu, 0x8B65ADB7u, 0xDFF4F563u, 0x7069770Du), v.c0) +
                        select(uint4(0xD1224537u, 0xE99ED6F3u, 0x48125549u, 0xEEE2123Bu), uint4(0xE3AD9FE5u, 0xCE1CF8BFu, 0x7BE39F3Bu, 0xFAB9913Fu), v.c1) +
                        select(uint4(0xB4501269u, 0xE04B89FDu, 0xDB3DE101u, 0x7B6D1B4Bu), uint4(0x58399E77u, 0x5EAC29C9u, 0xFC6014F9u, 0x6BF6693Fu), v.c2));
        }

        /// <summary>
        /// Returns a uint4 vector hash code of a bool4x3 matrix.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        /// <param name="v">Matrix value to hash.</param>
        /// <returns>uint4 hash of the argument.</returns>
        public static uint4 hashwide(bool4x3 v)
        {
            return (select(uint4(0x9D1B1D9Bu, 0xF842F5C1u, 0xA47EC335u, 0xA477DF57u), uint4(0xC4B1493Fu, 0xBA0966D3u, 0xAFBEE253u, 0x5B419C01u), v.c0) +
                    select(uint4(0x515D90F5u, 0xEC9F68F3u, 0xF9EA92D5u, 0xC2FAFCB9u), uint4(0x616E9CA1u, 0xC5C5394Bu, 0xCAE78587u, 0x7A1541C9u), v.c1) +
                    select(uint4(0xF83BD927u, 0x6A243BCBu, 0x509B84C9u, 0x91D13847u), uint4(0x52F7230Fu, 0xCF286E83u, 0xE121E6ADu, 0xC9CA1249u), v.c2));
        }

    }
}
