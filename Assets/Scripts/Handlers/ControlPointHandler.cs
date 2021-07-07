using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using MathNet.Numerics.LinearAlgebra;
using RSUtils;

public class ControlPointHandler : IHandler<(MapContainer container, ControlPointSettings settings), List<Vector<double>>>
{
    /// <summary>
    /// Given the different maps, their weights and an grid, generates and returns a list of positions that correspond to the optimal positions for control points
    /// </summary>
    /// <param name="input">A tuple consisting of the different maps and their weights.
    /// mindistance_w denotes the weighting factor for penalties, if a control point is close to previous control points
    /// num_cps denotes the number of control points to be generated
    /// </param>
    /// <returns>A list of optimally placed control points with |control points| = num_cps </returns>
    public List<Vector<double>> Invoke((MapContainer container, ControlPointSettings settings) input)
    {
        List<(Vector<double>, Vector<float>)> points = new List<(Vector<double>, Vector<float>)>();
        List<Vector<double>> cpoints = new List<Vector<double>>();

        // First generate a list of all potential control points
        input.container.rsgrid.ForAllIndexed((x, i) => {
            points.Add((x, i));
            return -1d;
        });


        // The score function by which points are evaluated
        Func<Vector<double>, Vector<float>, double> score = (p, i) =>
        {
            var v = (input.settings.visibility_weight * (1f - TryGet(i, input.container.visibility, x => x, Mathf.Epsilon)))
            + (input.settings.bitmap_weight * TryGet(i, input.container.bitmap, x => x ? 1 : 0, 1))
            + (input.settings.distance_weight * TryGet(i, input.container.distancetransform, x => Mathf.Clamp01(1f / x), Mathf.Epsilon))
            + (input.settings.cp_distance_weight * (1f - Mathf.InverseLerp(0f, 10f, GetMinCPDistance(p, cpoints))));
            return v * 100;
        };

        // Order the points in the list according to their score. Take the first point, then re-evaluate all the other points

        for (int i = 0; i < input.settings.num_control_points; i++)
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
