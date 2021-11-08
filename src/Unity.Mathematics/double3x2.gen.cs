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
    /// <summary>A 3x2 matrix of doubles.</summary>
    [System.Serializable]
    [Il2CppEagerStaticClassConstruction]
    public partial struct double3x2 : System.IEquatable<double3x2>, IFormattable
    {
        /// <summary>Column 0 of the matrix.</summary>
        public double3 c0;
        /// <summary>Column 1 of the matrix.</summary>
        public double3 c1;

        /// <summary>double3x2 zero value.</summary>
        public static readonly double3x2 zero;

        /// <summary>Constructs a double3x2 matrix from two double3 vectors.</summary>
        /// <param name="c0">The matrix column c0 will be set to this value.</param>
        /// <param name="c1">The matrix column c1 will be set to this value.</param>
        public double3x2(double3 c0, double3 c1)
        {
            this.c0 = c0;
            this.c1 = c1;
        }

        /// <summary>Constructs a double3x2 matrix from 6 double values given in row-major order.</summary>
        /// <param name="m00">The matrix at row 0, column 0 will be set to this value.</param>
        /// <param name="m01">The matrix at row 0, column 1 will be set to this value.</param>
        /// <param name="m10">The matrix at row 1, column 0 will be set to this value.</param>
        /// <param name="m11">The matrix at row 1, column 1 will be set to this value.</param>
        /// <param name="m20">The matrix at row 2, column 0 will be set to this value.</param>
        /// <param name="m21">The matrix at row 2, column 1 will be set to this value.</param>
        public double3x2(double m00, double m01,
                         double m10, double m11,
                         double m20, double m21)
        {
            this.c0 = new double3(m00, m10, m20);
            this.c1 = new double3(m01, m11, m21);
        }

        /// <summary>Constructs a double3x2 matrix from a single double value by assigning it to every component.</summary>
        /// <param name="v">double to convert to double3x2</param>
        public double3x2(double v)
        {
            this.c0 = v;
            this.c1 = v;
        }

        /// <summary>Constructs a double3x2 matrix from a single bool value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">bool to convert to double3x2</param>
        public double3x2(bool v)
        {
            this.c0 = math.select(new double3(0.0), new double3(1.0), v);
            this.c1 = math.select(new double3(0.0), new double3(1.0), v);
        }

        /// <summary>Constructs a double3x2 matrix from a bool3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">bool3x2 to convert to double3x2</param>
        public double3x2(bool3x2 v)
        {
            this.c0 = math.select(new double3(0.0), new double3(1.0), v.c0);
            this.c1 = math.select(new double3(0.0), new double3(1.0), v.c1);
        }

        /// <summary>Constructs a double3x2 matrix from a single int value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">int to convert to double3x2</param>
        public double3x2(int v)
        {
            this.c0 = v;
            this.c1 = v;
        }

        /// <summary>Constructs a double3x2 matrix from a int3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">int3x2 to convert to double3x2</param>
        public double3x2(int3x2 v)
        {
            this.c0 = v.c0;
            this.c1 = v.c1;
        }

        /// <summary>Constructs a double3x2 matrix from a single uint value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">uint to convert to double3x2</param>
        public double3x2(uint v)
        {
            this.c0 = v;
            this.c1 = v;
        }

        /// <summary>Constructs a double3x2 matrix from a uint3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">uint3x2 to convert to double3x2</param>
        public double3x2(uint3x2 v)
        {
            this.c0 = v.c0;
            this.c1 = v.c1;
        }

        /// <summary>Constructs a double3x2 matrix from a single float value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">float to convert to double3x2</param>
        public double3x2(float v)
        {
            this.c0 = v;
            this.c1 = v;
        }

        /// <summary>Constructs a double3x2 matrix from a float3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">float3x2 to convert to double3x2</param>
        public double3x2(float3x2 v)
        {
            this.c0 = v.c0;
            this.c1 = v.c1;
        }


        /// <summary>Implicitly converts a single double value to a double3x2 matrix by assigning it to every component.</summary>
        /// <param name="v">double to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(double v) { return new double3x2(v); }

        /// <summary>Explicitly converts a single bool value to a double3x2 matrix by converting it to double and assigning it to every component.</summary>
        /// <param name="v">bool to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static explicit operator double3x2(bool v) { return new double3x2(v); }

        /// <summary>Explicitly converts a bool3x2 matrix to a double3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">bool3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static explicit operator double3x2(bool3x2 v) { return new double3x2(v); }

        /// <summary>Implicitly converts a single int value to a double3x2 matrix by converting it to double and assigning it to every component.</summary>
        /// <param name="v">int to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(int v) { return new double3x2(v); }

        /// <summary>Implicitly converts a int3x2 matrix to a double3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">int3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(int3x2 v) { return new double3x2(v); }

        /// <summary>Implicitly converts a single uint value to a double3x2 matrix by converting it to double and assigning it to every component.</summary>
        /// <param name="v">uint to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(uint v) { return new double3x2(v); }

        /// <summary>Implicitly converts a uint3x2 matrix to a double3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">uint3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(uint3x2 v) { return new double3x2(v); }

        /// <summary>Implicitly converts a single float value to a double3x2 matrix by converting it to double and assigning it to every component.</summary>
        /// <param name="v">float to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(float v) { return new double3x2(v); }

        /// <summary>Implicitly converts a float3x2 matrix to a double3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">float3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static implicit operator double3x2(float3x2 v) { return new double3x2(v); }


        /// <summary>Returns the result of a componentwise multiplication operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise multiplication.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise multiplication.</param>
        /// <returns>double3x2 result of the componentwise multiplication.</returns>
        public static double3x2 operator * (double3x2 lhs, double3x2 rhs) { return new double3x2 (lhs.c0 * rhs.c0, lhs.c1 * rhs.c1); }

        /// <summary>Returns the result of a componentwise multiplication operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise multiplication.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise multiplication.</param>
        /// <returns>double3x2 result of the componentwise multiplication.</returns>
        public static double3x2 operator * (double3x2 lhs, double rhs) { return new double3x2 (lhs.c0 * rhs, lhs.c1 * rhs); }

        /// <summary>Returns the result of a componentwise multiplication operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise multiplication.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise multiplication.</param>
        /// <returns>double3x2 result of the componentwise multiplication.</returns>
        public static double3x2 operator * (double lhs, double3x2 rhs) { return new double3x2 (lhs * rhs.c0, lhs * rhs.c1); }


        /// <summary>Returns the result of a componentwise addition operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise addition.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise addition.</param>
        /// <returns>double3x2 result of the componentwise addition.</returns>
        public static double3x2 operator + (double3x2 lhs, double3x2 rhs) { return new double3x2 (lhs.c0 + rhs.c0, lhs.c1 + rhs.c1); }

        /// <summary>Returns the result of a componentwise addition operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise addition.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise addition.</param>
        /// <returns>double3x2 result of the componentwise addition.</returns>
        public static double3x2 operator + (double3x2 lhs, double rhs) { return new double3x2 (lhs.c0 + rhs, lhs.c1 + rhs); }

        /// <summary>Returns the result of a componentwise addition operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise addition.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise addition.</param>
        /// <returns>double3x2 result of the componentwise addition.</returns>
        public static double3x2 operator + (double lhs, double3x2 rhs) { return new double3x2 (lhs + rhs.c0, lhs + rhs.c1); }


        /// <summary>Returns the result of a componentwise subtraction operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise subtraction.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise subtraction.</param>
        /// <returns>double3x2 result of the componentwise subtraction.</returns>
        public static double3x2 operator - (double3x2 lhs, double3x2 rhs) { return new double3x2 (lhs.c0 - rhs.c0, lhs.c1 - rhs.c1); }

        /// <summary>Returns the result of a componentwise subtraction operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise subtraction.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise subtraction.</param>
        /// <returns>double3x2 result of the componentwise subtraction.</returns>
        public static double3x2 operator - (double3x2 lhs, double rhs) { return new double3x2 (lhs.c0 - rhs, lhs.c1 - rhs); }

        /// <summary>Returns the result of a componentwise subtraction operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise subtraction.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise subtraction.</param>
        /// <returns>double3x2 result of the componentwise subtraction.</returns>
        public static double3x2 operator - (double lhs, double3x2 rhs) { return new double3x2 (lhs - rhs.c0, lhs - rhs.c1); }


        /// <summary>Returns the result of a componentwise division operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise division.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise division.</param>
        /// <returns>double3x2 result of the componentwise division.</returns>
        public static double3x2 operator / (double3x2 lhs, double3x2 rhs) { return new double3x2 (lhs.c0 / rhs.c0, lhs.c1 / rhs.c1); }

        /// <summary>Returns the result of a componentwise division operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise division.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise division.</param>
        /// <returns>double3x2 result of the componentwise division.</returns>
        public static double3x2 operator / (double3x2 lhs, double rhs) { return new double3x2 (lhs.c0 / rhs, lhs.c1 / rhs); }

        /// <summary>Returns the result of a componentwise division operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise division.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise division.</param>
        /// <returns>double3x2 result of the componentwise division.</returns>
        public static double3x2 operator / (double lhs, double3x2 rhs) { return new double3x2 (lhs / rhs.c0, lhs / rhs.c1); }


        /// <summary>Returns the result of a componentwise modulus operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise modulus.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise modulus.</param>
        /// <returns>double3x2 result of the componentwise modulus.</returns>
        public static double3x2 operator % (double3x2 lhs, double3x2 rhs) { return new double3x2 (lhs.c0 % rhs.c0, lhs.c1 % rhs.c1); }

        /// <summary>Returns the result of a componentwise modulus operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise modulus.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise modulus.</param>
        /// <returns>double3x2 result of the componentwise modulus.</returns>
        public static double3x2 operator % (double3x2 lhs, double rhs) { return new double3x2 (lhs.c0 % rhs, lhs.c1 % rhs); }

        /// <summary>Returns the result of a componentwise modulus operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise modulus.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise modulus.</param>
        /// <returns>double3x2 result of the componentwise modulus.</returns>
        public static double3x2 operator % (double lhs, double3x2 rhs) { return new double3x2 (lhs % rhs.c0, lhs % rhs.c1); }


        /// <summary>Returns the result of a componentwise increment operation on a double3x2 matrix.</summary>
        /// <param name="val">Value to use when computing the componentwise increment.</param>
        /// <returns>double3x2 result of the componentwise increment.</returns>
        public static double3x2 operator ++ (double3x2 val) { return new double3x2 (++val.c0, ++val.c1); }


        /// <summary>Returns the result of a componentwise decrement operation on a double3x2 matrix.</summary>
        /// <param name="val">Value to use when computing the componentwise decrement.</param>
        /// <returns>double3x2 result of the componentwise decrement.</returns>
        public static double3x2 operator -- (double3x2 val) { return new double3x2 (--val.c0, --val.c1); }


        /// <summary>Returns the result of a componentwise less than operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise less than.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise less than.</param>
        /// <returns>bool3x2 result of the componentwise less than.</returns>
        public static bool3x2 operator < (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 < rhs.c0, lhs.c1 < rhs.c1); }

        /// <summary>Returns the result of a componentwise less than operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise less than.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise less than.</param>
        /// <returns>bool3x2 result of the componentwise less than.</returns>
        public static bool3x2 operator < (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 < rhs, lhs.c1 < rhs); }

        /// <summary>Returns the result of a componentwise less than operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise less than.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise less than.</param>
        /// <returns>bool3x2 result of the componentwise less than.</returns>
        public static bool3x2 operator < (double lhs, double3x2 rhs) { return new bool3x2 (lhs < rhs.c0, lhs < rhs.c1); }


        /// <summary>Returns the result of a componentwise less or equal operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise less or equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise less or equal.</param>
        /// <returns>bool3x2 result of the componentwise less or equal.</returns>
        public static bool3x2 operator <= (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 <= rhs.c0, lhs.c1 <= rhs.c1); }

        /// <summary>Returns the result of a componentwise less or equal operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise less or equal.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise less or equal.</param>
        /// <returns>bool3x2 result of the componentwise less or equal.</returns>
        public static bool3x2 operator <= (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 <= rhs, lhs.c1 <= rhs); }

        /// <summary>Returns the result of a componentwise less or equal operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise less or equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise less or equal.</param>
        /// <returns>bool3x2 result of the componentwise less or equal.</returns>
        public static bool3x2 operator <= (double lhs, double3x2 rhs) { return new bool3x2 (lhs <= rhs.c0, lhs <= rhs.c1); }


        /// <summary>Returns the result of a componentwise greater than operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise greater than.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise greater than.</param>
        /// <returns>bool3x2 result of the componentwise greater than.</returns>
        public static bool3x2 operator > (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 > rhs.c0, lhs.c1 > rhs.c1); }

        /// <summary>Returns the result of a componentwise greater than operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise greater than.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise greater than.</param>
        /// <returns>bool3x2 result of the componentwise greater than.</returns>
        public static bool3x2 operator > (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 > rhs, lhs.c1 > rhs); }

        /// <summary>Returns the result of a componentwise greater than operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise greater than.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise greater than.</param>
        /// <returns>bool3x2 result of the componentwise greater than.</returns>
        public static bool3x2 operator > (double lhs, double3x2 rhs) { return new bool3x2 (lhs > rhs.c0, lhs > rhs.c1); }


        /// <summary>Returns the result of a componentwise greater or equal operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise greater or equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise greater or equal.</param>
        /// <returns>bool3x2 result of the componentwise greater or equal.</returns>
        public static bool3x2 operator >= (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 >= rhs.c0, lhs.c1 >= rhs.c1); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise greater or equal.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise greater or equal.</param>
        /// <returns>bool3x2 result of the componentwise greater or equal.</returns>
        public static bool3x2 operator >= (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 >= rhs, lhs.c1 >= rhs); }

        /// <summary>Returns the result of a componentwise greater or equal operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise greater or equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise greater or equal.</param>
        /// <returns>bool3x2 result of the componentwise greater or equal.</returns>
        public static bool3x2 operator >= (double lhs, double3x2 rhs) { return new bool3x2 (lhs >= rhs.c0, lhs >= rhs.c1); }


        /// <summary>Returns the result of a componentwise unary minus operation on a double3x2 matrix.</summary>
        /// <param name="val">Value to use when computing the componentwise unary minus.</param>
        /// <returns>double3x2 result of the componentwise unary minus.</returns>
        public static double3x2 operator - (double3x2 val) { return new double3x2 (-val.c0, -val.c1); }


        /// <summary>Returns the result of a componentwise unary plus operation on a double3x2 matrix.</summary>
        /// <param name="val">Value to use when computing the componentwise unary plus.</param>
        /// <returns>double3x2 result of the componentwise unary plus.</returns>
        public static double3x2 operator + (double3x2 val) { return new double3x2 (+val.c0, +val.c1); }


        /// <summary>Returns the result of a componentwise equality operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise equality.</param>
        /// <returns>bool3x2 result of the componentwise equality.</returns>
        public static bool3x2 operator == (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 == rhs.c0, lhs.c1 == rhs.c1); }

        /// <summary>Returns the result of a componentwise equality operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise equality.</param>
        /// <returns>bool3x2 result of the componentwise equality.</returns>
        public static bool3x2 operator == (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 == rhs, lhs.c1 == rhs); }

        /// <summary>Returns the result of a componentwise equality operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise equality.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise equality.</param>
        /// <returns>bool3x2 result of the componentwise equality.</returns>
        public static bool3x2 operator == (double lhs, double3x2 rhs) { return new bool3x2 (lhs == rhs.c0, lhs == rhs.c1); }


        /// <summary>Returns the result of a componentwise not equal operation on two double3x2 matrices.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise not equal.</param>
        /// <returns>bool3x2 result of the componentwise not equal.</returns>
        public static bool3x2 operator != (double3x2 lhs, double3x2 rhs) { return new bool3x2 (lhs.c0 != rhs.c0, lhs.c1 != rhs.c1); }

        /// <summary>Returns the result of a componentwise not equal operation on a double3x2 matrix and a double value.</summary>
        /// <param name="lhs">Left hand side double3x2 to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side double to use to compute componentwise not equal.</param>
        /// <returns>bool3x2 result of the componentwise not equal.</returns>
        public static bool3x2 operator != (double3x2 lhs, double rhs) { return new bool3x2 (lhs.c0 != rhs, lhs.c1 != rhs); }

        /// <summary>Returns the result of a componentwise not equal operation on a double value and a double3x2 matrix.</summary>
        /// <param name="lhs">Left hand side double to use to compute componentwise not equal.</param>
        /// <param name="rhs">Right hand side double3x2 to use to compute componentwise not equal.</param>
        /// <returns>bool3x2 result of the componentwise not equal.</returns>
        public static bool3x2 operator != (double lhs, double3x2 rhs) { return new bool3x2 (lhs != rhs.c0, lhs != rhs.c1); }




        /// <summary>Returns true if the double3x2 is equal to a given double3x2, false otherwise.</summary>
        /// <param name="rhs">Right hand side argument to compare equality with.</param>
        /// <returns>The result of the equality comparison.</returns>
        public bool Equals(double3x2 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1); }

        /// <summary>Returns true if the double3x2 is equal to a given double3x2, false otherwise.</summary>
        /// <param name="o">Right hand side argument to compare equality with.</param>
        /// <returns>The result of the equality comparison.</returns>
        public override bool Equals(object o) { return o is double3x2 && Equals((double3x2)o); }


        /// <summary>Returns a hash code for the double3x2.</summary>
        /// <returns>The computed hash code.</returns>
        public override int GetHashCode() { return (int)math.hash(this); }


        /// <summary>Returns a string representation of the double3x2.</summary>
        /// <returns>String representation of the value.</returns>
        public override string ToString()
        {
            return string.Format("double3x2({0}, {1},  {2}, {3},  {4}, {5})", c0.x, c1.x, c0.y, c1.y, c0.z, c1.z);
        }

        /// <summary>Returns a string representation of the double3x2 using a specified format and culture-specific format information.</summary>
        /// <param name="format">Format string to use during string formatting.</param>
        /// <param name="formatProvider">Format provider to use during string formatting.</param>
        /// <returns>String representation of the value.</returns>
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("double3x2({0}, {1},  {2}, {3},  {4}, {5})", c0.x.ToString(format, formatProvider), c1.x.ToString(format, formatProvider), c0.y.ToString(format, formatProvider), c1.y.ToString(format, formatProvider), c0.z.ToString(format, formatProvider), c1.z.ToString(format, formatProvider));
        }

    }

    public static partial class math
    {
        /// <summary>Returns a double3x2 matrix constructed from two double3 vectors.</summary>
        /// <param name="c0">The matrix column c0 will be set to this value.</param>
        /// <param name="c1">The matrix column c1 will be set to this value.</param>
        /// <returns>double3x2 constructed from arguments.</returns>
        public static double3x2 double3x2(double3 c0, double3 c1) { return new double3x2(c0, c1); }

        /// <summary>Returns a double3x2 matrix constructed from from 6 double values given in row-major order.</summary>
        /// <param name="m00">The matrix at row 0, column 0 will be set to this value.</param>
        /// <param name="m01">The matrix at row 0, column 1 will be set to this value.</param>
        /// <param name="m10">The matrix at row 1, column 0 will be set to this value.</param>
        /// <param name="m11">The matrix at row 1, column 1 will be set to this value.</param>
        /// <param name="m20">The matrix at row 2, column 0 will be set to this value.</param>
        /// <param name="m21">The matrix at row 2, column 1 will be set to this value.</param>
        /// <returns>double3x2 constructed from arguments.</returns>
        public static double3x2 double3x2(double m00, double m01,
                                          double m10, double m11,
                                          double m20, double m21)
        {
            return new double3x2(m00, m01,
                                 m10, m11,
                                 m20, m21);
        }

        /// <summary>Returns a double3x2 matrix constructed from a single double value by assigning it to every component.</summary>
        /// <param name="v">double to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(double v) { return new double3x2(v); }

        /// <summary>Returns a double3x2 matrix constructed from a single bool value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">bool to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(bool v) { return new double3x2(v); }

        /// <summary>Return a double3x2 matrix constructed from a bool3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">bool3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(bool3x2 v) { return new double3x2(v); }

        /// <summary>Returns a double3x2 matrix constructed from a single int value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">int to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(int v) { return new double3x2(v); }

        /// <summary>Return a double3x2 matrix constructed from a int3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">int3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(int3x2 v) { return new double3x2(v); }

        /// <summary>Returns a double3x2 matrix constructed from a single uint value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">uint to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(uint v) { return new double3x2(v); }

        /// <summary>Return a double3x2 matrix constructed from a uint3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">uint3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(uint3x2 v) { return new double3x2(v); }

        /// <summary>Returns a double3x2 matrix constructed from a single float value by converting it to double and assigning it to every component.</summary>
        /// <param name="v">float to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(float v) { return new double3x2(v); }

        /// <summary>Return a double3x2 matrix constructed from a float3x2 matrix by componentwise conversion.</summary>
        /// <param name="v">float3x2 to convert to double3x2</param>
        /// <returns>Converted value.</returns>
        public static double3x2 double3x2(float3x2 v) { return new double3x2(v); }

        /// <summary>Return the double2x3 transpose of a double3x2 matrix.</summary>
        /// <param name="v">Value to transpose.</param>
        /// <returns>Transposed value.</returns>
        public static double2x3 transpose(double3x2 v)
        {
            return double2x3(
                v.c0.x, v.c0.y, v.c0.z,
                v.c1.x, v.c1.y, v.c1.z);
        }

        /// <summary>Returns a uint hash code of a double3x2 matrix.</summary>
        /// <param name="v">Matrix value to hash.</param>
        /// <returns>uint hash of the argument.</returns>
        public static uint hash(double3x2 v)
        {
            return csum(fold_to_uint(v.c0) * uint3(0xEE390C97u, 0x9C8A2F05u, 0x4DDC6509u) +
                        fold_to_uint(v.c1) * uint3(0x7CF083CBu, 0x5C4D6CEDu, 0xF9137117u)) + 0xE857DCE1u;
        }

        /// <summary>
        /// Returns a uint3 vector hash code of a double3x2 matrix.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        /// <param name="v">Matrix value to hash.</param>
        /// <returns>uint3 hash of the argument.</returns>
        public static uint3 hashwide(double3x2 v)
        {
            return (fold_to_uint(v.c0) * uint3(0xF62213C5u, 0x9CDAA959u, 0xAA269ABFu) +
                    fold_to_uint(v.c1) * uint3(0xD54BA36Fu, 0xFD0847B9u, 0x8189A683u)) + 0xB139D651u;
        }

    }
}
