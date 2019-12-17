//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------
using System;
using NUnit.Framework;
using Unity.PerformanceTesting;
using Unity.Burst;
using Unity.Collections;

namespace Unity.Mathematics.PerformanceTests
{
    public partial class TestMul
    {
        [BurstCompile]
        public class float4x4_float4x4
        {
            public struct Arguments
            {
                public float4x4 m1;
                public float4x4 m2;

                public void Init()
                {
                    m1 = float4x4.identity;
                    m2 = float4x4.identity;
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m1 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float4x4_float4x4_mono()
        {
            float4x4_float4x4.TestFunction testFunction = float4x4_float4x4.MonoTestFunction;
            var args = new float4x4_float4x4.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float4x4_float4x4_burst()
        {
            FunctionPointer<float4x4_float4x4.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float4x4_float4x4.TestFunction>(float4x4_float4x4.BurstTestFunction);
            var args = new float4x4_float4x4.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class float4x4_float4
        {
            public struct Arguments
            {
                public float4x4 m1;
                public float4 m2;

                public void Init()
                {
                    m1 = float4x4.identity;
                    m2 = new float4(1.0f, 0.0f, 0.0f, 1.0f);
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m2 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float4x4_float4_mono()
        {
            float4x4_float4.TestFunction testFunction = float4x4_float4.MonoTestFunction;
            var args = new float4x4_float4.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float4x4_float4_burst()
        {
            FunctionPointer<float4x4_float4.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float4x4_float4.TestFunction>(float4x4_float4.BurstTestFunction);
            var args = new float4x4_float4.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class quaternion_quaternion
        {
            public struct Arguments
            {
                public quaternion q1;
                public quaternion q2;

                public void Init()
                {
                    q1 = quaternion.identity;
                    q2 = quaternion.identity;
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.q2 = math.mul(args.q1, args.q2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void quaternion_quaternion_mono()
        {
            quaternion_quaternion.TestFunction testFunction = quaternion_quaternion.MonoTestFunction;
            var args = new quaternion_quaternion.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void quaternion_quaternion_burst()
        {
            FunctionPointer<quaternion_quaternion.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<quaternion_quaternion.TestFunction>(quaternion_quaternion.BurstTestFunction);
            var args = new quaternion_quaternion.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class float3x3_float3x3
        {
            public struct Arguments
            {
                public float3x3 m1;
                public float3x3 m2;

                public void Init()
                {
                    m1 = float3x3.identity;
                    m2 = float3x3.identity;
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m2 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float3x3_float3x3_mono()
        {
            float3x3_float3x3.TestFunction testFunction = float3x3_float3x3.MonoTestFunction;
            var args = new float3x3_float3x3.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float3x3_float3x3_burst()
        {
            FunctionPointer<float3x3_float3x3.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float3x3_float3x3.TestFunction>(float3x3_float3x3.BurstTestFunction);
            var args = new float3x3_float3x3.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class float2x2_float2x2
        {
            public struct Arguments
            {
                public float2x2 m1;
                public float2x2 m2;

                public void Init()
                {
                    m1 = float2x2.identity;
                    m2 = float2x2.identity;
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m2 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float2x2_float2x2_mono()
        {
            float2x2_float2x2.TestFunction testFunction = float2x2_float2x2.MonoTestFunction;
            var args = new float2x2_float2x2.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float2x2_float2x2_burst()
        {
            FunctionPointer<float2x2_float2x2.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float2x2_float2x2.TestFunction>(float2x2_float2x2.BurstTestFunction);
            var args = new float2x2_float2x2.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class float3x3_float3
        {
            public struct Arguments
            {
                public float3x3 m1;
                public float3 m2;

                public void Init()
                {
                    m1 = float3x3.identity;
                    m2 = new float3(1.0f, 0.0f, 0.0f);
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m2 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float3x3_float3_mono()
        {
            float3x3_float3.TestFunction testFunction = float3x3_float3.MonoTestFunction;
            var args = new float3x3_float3.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float3x3_float3_burst()
        {
            FunctionPointer<float3x3_float3.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float3x3_float3.TestFunction>(float3x3_float3.BurstTestFunction);
            var args = new float3x3_float3.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
        [BurstCompile]
        public class float2x2_float2
        {
            public struct Arguments
            {
                public float2x2 m1;
                public float2 m2;

                public void Init()
                {
                    m1 = float2x2.identity;
                    m2 = new float2(1.0f, 0.0f);
                }
            }

            public static void CommonTestFunction(ref Arguments args)
            {
                for (int i = 0; i < 10000; ++i)
                {
                    args.m2 = math.mul(args.m1, args.m2);
                }
            }

            public static void MonoTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            [BurstCompile]
            public static void BurstTestFunction(ref Arguments args)
            {
                CommonTestFunction(ref args);
            }

            public delegate void TestFunction(ref Arguments args);
        }

        [Test, Performance]
        public void float2x2_float2_mono()
        {
            float2x2_float2.TestFunction testFunction = float2x2_float2.MonoTestFunction;
            var args = new float2x2_float2.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }

        [Test, Performance]
        public void float2x2_float2_burst()
        {
            FunctionPointer<float2x2_float2.TestFunction> testFunction = BurstCompiler.CompileFunctionPointer<float2x2_float2.TestFunction>(float2x2_float2.BurstTestFunction);
            var args = new float2x2_float2.Arguments();
            args.Init();

            Measure.Method(() =>
            {
                testFunction.Invoke(ref args);
            })
            .WarmupCount(1)
            .MeasurementCount(10)
            .Run();
        }
    }
}
