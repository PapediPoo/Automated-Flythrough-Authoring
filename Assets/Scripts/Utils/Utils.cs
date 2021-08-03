using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace RSUtils
{
    public class Utils
    {
        public static Vector3 VToV3(Vector<double> v)
        {
            return new Vector3((float)v.At(0), (float)v.At(1), (float)v.At(2));
        }

        public static Vector3 VToV3(Vector<float> v)
        {
            return new Vector3(v.At(0), v.At(1), v.At(2));
        }

        public static Vector<double> V3ToV(Vector3 v3, Vector<double> v=null)
        {
            if(v == null)
            {
                v = Vector<double>.Build.Dense(3);
            }
            v[0] = v3.x;
            v[1] = v3.y;
            v[2] = v3.z;

            return v;
        }
    }
}
