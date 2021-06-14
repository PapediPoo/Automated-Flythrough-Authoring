using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using MathNet.Numerics.LinearAlgebra;
using RSUtils;

public class ControlPointHandler : IHandler<(RSGrid rsgrid, bool[,,] bitmap, float bitmap_w, float[,,] visibilitymap, float visibility_w, float[,,] distancetransformmap, float distancetransform_w, float mindistance_w, int num_cps), List<Vector<double>>>
{
    /// <summary>
    /// Given the different maps, their weights and an grid, generates and returns a list of positions that correspond to the optimal positions for control points
    /// </summary>
    /// <param name="input">A tuple consisting of the different maps and their weights.
    /// mindistance_w denotes the weighting factor for penalties, if a control point is close to previous control points
    /// num_cps denotes the number of control points to be generated
    /// </param>
    /// <returns>A list of optimally placed control points with |control points| = num_cps </returns>
    public List<Vector<double>> Invoke((RSGrid rsgrid, bool[,,] bitmap, float bitmap_w, float[,,] visibilitymap, float visibility_w, float[,,] distancetransformmap, float distancetransform_w, float mindistance_w, int num_cps) input)
    {
        List<(Vector<double>, Vector<float>)> points = new List<(Vector<double>, Vector<float>)>();
        List<Vector<double>> cpoints = new List<Vector<double>>();

        // First generate a list of all potential control points
        input.rsgrid.ForAllIndexed((x, i) => {
            points.Add((x, i));
            return -1d;
        });


        // The score function by which points are evaluated
        Func<Vector<double>, Vector<float>, double> score = (p, i) =>
        {
            return (input.visibility_w * (1f - TryGet(i, input.visibilitymap, x => x, Mathf.Epsilon)))
            + (input.bitmap_w * TryGet(i, input.bitmap, x => x ? 1 : 0, 1))
            + (input.distancetransform_w * TryGet(i, input.distancetransformmap, x => Mathf.Clamp01(1f / x), Mathf.Epsilon))
            + (input.mindistance_w * Mathf.Clamp01(1f / GetMinCPDistance(p, cpoints)));
        };

        // Order the points in the list according to their score. Take the first point, then re-evaluate all the other points
        for (int i = 0; i < input.num_cps; i++)
        {
            var l = points.OrderBy(x => (int)score(x.Item1, x.Item2)).ToList();
            cpoints.Add(l[0].Item1);
            points = l.Skip(1).ToList();
        }

        return cpoints;
    }

    /// <summary>
    /// Tries to index a multid. array "arr" with "index" and maps the return value to a float.
    /// Returns default_val if IndexOutOfgRange or NullReference
    /// </summary>
    /// <typeparam name="T">The type of the elements in the multid. array</typeparam>
    /// <param name="index">The index of the element given as a float vector</param>
    /// <param name="arr">The array to be indexed</param>
    /// <param name="map">The mapping function for the returned element to a float</param>
    /// <param name="default_val">The value to be returned if an exception occurs</param>
    /// <returns></returns>
    private float TryGet<T>(Vector<float> index, T[,,] arr, Func<T, float> map, float default_val)
    {
        int x = (int)index.At(0);
        int y = (int)index.At(1);
        int z = (int)index.At(2);
        try
        {
            return map((T)arr.GetValue(x, y, z));
        }
        catch (IndexOutOfRangeException)
        {
            return default_val;
        }
        catch (NullReferenceException)
        {
            return default_val;
        }
    }

    //public double Score(Vector<double> x)
    //{
    //    float s = (visibilityWeight * (1f - GetInterpolated(x, GetVisibility)))
    //        + (bitmapWeight * GetInterpolated(x, GetBitmap))
    //        + (distanceTransformWeight * Mathf.Clamp01(1f / GetInterpolated(x, GetDistanceTransform)))
    //        + (minDistanceWeight * Mathf.Clamp01(1f / GetMinCPDistance(x)));
    //    return s;
    //}

    //private float GetInterpolated(Vector<double> pos, Func<Vector<double>, float> function)
    //{
    //    Vector3 pos3 = new Vector3((float)pos.At(0), (float)pos.At(1), (float)pos.At(2));
    //    pos3 = (pos3 + col.bounds.extents - col.bounds.center) / size;
    //    float[] x = new float[] { pos3.x - Mathf.Floor(pos3.x), pos3.y - Mathf.Floor(pos3.y), pos3.z - Mathf.Floor(pos3.z) };
    //    pos = RSUtils.Utils.V3ToV(pos3);
    //    float[] q = new float[]
    //    {
    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{0, 0, 0})),

    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{0, 1, 0})),
    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{0, 0, 1})),

    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{0, 1, 1})),
    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{1, 0, 0})),

    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{1, 1, 0})),
    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{1, 0, 1})),

    //        function(pos + Vector<double>.Build.DenseOfArray(new double[]{1, 1, 1}))
    //    };
    //    return RSUtils.Utils.TriLerp(x, q);
    //    //return function(pos);
    //}

    //private float GetVisibility(Vector<double> pos)
    //{
    //    int x = (int)pos.At(0);
    //    int y = (int)pos.At(1);
    //    int z = (int)pos.At(2);
    //    try
    //    {
    //        return (float)visibility.GetValue(x, y, z);
    //    }
    //    catch (IndexOutOfRangeException)
    //    {
    //        return Mathf.Epsilon;
    //    }
    //    catch (NullReferenceException)
    //    {
    //        return Mathf.Epsilon;
    //    }
    //}

    //private float GetBitmap(Vector<double> pos)
    //{
    //    int x = (int)pos.At(0);
    //    int y = (int)pos.At(1);
    //    int z = (int)pos.At(2);

    //    // print("getbitmap");

    //    try
    //    {
    //        // print((bool)bitmap.GetValue(x, y, z));
    //        if ((bool)bitmap.GetValue(x, y, z))
    //        {
    //            return 1f;
    //        }
    //        else
    //        {
    //            return 0f;
    //        }
    //    }
    //    catch (IndexOutOfRangeException)
    //    {
    //        return 1f;
    //    }
    //}

    //private float GetDistanceTransform(Vector<double> pos)
    //{
    //    int x = (int)pos.At(0);
    //    int y = (int)pos.At(1);
    //    int z = (int)pos.At(2);

    //    try
    //    {
    //        // print((bool)bitmap.GetValue(x, y, z));

    //        return Mathf.Max((float)distanceTransform.GetValue(x, y, z), Mathf.Epsilon);
    //    }
    //    catch (IndexOutOfRangeException)
    //    {
    //        return Mathf.Epsilon;
    //    }
    //}

    /// <summary>
    /// Given an evaluation position and a list of control points, calculates the distance to the closest control point
    /// </summary>
    /// <param name="pos">The position to be evaluated</param>
    /// <param name="controlPoints">The list of given control points</param>
    /// <returns>The min distance to any control point </returns>
    private float GetMinCPDistance(Vector<double> pos, List<Vector<double>> controlPoints)
    {
        Vector3 posV3 = RSUtils.Utils.VToV3(pos);
        float min = Mathf.Infinity;
        foreach (Vector<double> p in controlPoints)
        {
            min = Mathf.Min(min, (float)p.Subtract(pos).L1Norm());
        }
        return Mathf.Max(min, Mathf.Epsilon);
    }
}
