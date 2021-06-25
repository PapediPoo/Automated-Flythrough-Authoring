using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace RSUtils
{
    public class Utils
    {
        public static float BiLerp(float[] x, float[] q)
        {
            System.Diagnostics.Debug.Assert(x.Length == 2);
            System.Diagnostics.Debug.Assert(q.Length == 4);

            float r0 = Mathf.Lerp(q[0], q[2], x[0]);
            float r1 = Mathf.Lerp(q[1], q[3], x[0]);

            return Mathf.Lerp(r0, r1, x[1]);
        }

        public static float TriLerp(float[] x, float[] q)
        {
            System.Diagnostics.Debug.Assert(x.Length == 3);
            System.Diagnostics.Debug.Assert(q.Length == 8);

            float x00 = Mathf.Lerp(q[0], q[4], x[0]);
            float x10 = Mathf.Lerp(q[2], q[6], x[0]);
            float x01 = Mathf.Lerp(q[1], q[5], x[0]);
            float x11 = Mathf.Lerp(q[3], q[7], x[0]);

            float r0 = Mathf.Lerp(x00, x01, x[1]);
            float r1 = Mathf.Lerp(x10, x11, x[1]);

            return Mathf.Lerp(r0, r1, x[2]);
        }

        public static Vector3 VToV3(Vector<double> v)
        {
            return new Vector3((float)v.At(0), (float)v.At(1), (float)v.At(2));
        }

        public static Vector<double> V3ToV(Vector3 v3)
        {
            return Vector<double>.Build.DenseOfArray(new double[] { v3.x, v3.y, v3.z });
        }
    }
}
