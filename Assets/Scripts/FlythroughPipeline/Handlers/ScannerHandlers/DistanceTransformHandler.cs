using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using System;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Generates a distance transform map of the grid given a bitmap
/// Note: The distance transform IGNORES THE Y AXIS
/// 
/// Version 0.1
/// Author: Robin Schmidiger
/// Date: May 2021
/// </summary>
public class DistanceTransformHandler : IHandler<(RSGrid, bool[,,], int), float[,,]>
{
    /// <summary>
    /// 
    /// </summary>
    /// <param name="input">Tuple consisting of a grid an the bitmap that corresponds to the grid</param>
    /// <returns>the distance transform of the grid as a multid. array. The distances are given in [u]</returns>
    public float[,,] Invoke((RSGrid, bool[,,], int) input)
    {
        RSGrid rsgrid = input.Item1;
        bool[,,] bitmap = input.Item2;
        int iterations = input.Item3;

        // Initialize the distance transform as the bitmap.
        float[,,] last_result = rsgrid.ForAllIndexed((_, i) => {
            return (bool)bitmap.GetValue((int)i.At(0), (int)i.At(1), (int)i.At(2)) ? 0f : Mathf.Infinity;
        });


        for (int current_iteration = 0; current_iteration < iterations; current_iteration++)
        {
            Func<Vector<double>, Vector<float>, float> f = (_, i) =>
            {
                int x = (int)i.At(0);
                int y = (int)i.At(1);
                int z = (int)i.At(2);

                // Update the distance of adjacent cells to the distance of the current cell + distance between cells.
                // Only do this if the calculated distance is smaller than the current distance
                float locmin = Mathf.Infinity;
                for (int xo = -1; xo <= 1; xo++)
                {
                    for (int zo = -1; zo <= 1; zo++)
                    {
                        try
                        {
                            Vector3 o = new Vector3(xo, 0, zo) * (float)rsgrid.GetCellSize();
                            locmin = Mathf.Min(locmin, (float)last_result.GetValue(x + xo, y, z + zo) + o.magnitude);
                        }
                        catch (System.IndexOutOfRangeException)
                        {

                        }
                    }
                }
                return locmin;
            };
            last_result = rsgrid.ForAllIndexed(f);
        }

        return last_result;
    }
}
