using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RSUtils;
using System;
using MathNet.Numerics.LinearAlgebra;

using UnityEngine;

/// <summary>
/// Handler that generates a bitmap inside the grid
/// 
/// Version: 0.4
/// Author: Robin Schmidiger
/// Date: May 2021
/// </summary>
public class BitmapHandler : IHandler<(RSGrid, LayerMask), bool[,,]>
{
    public bool[,,] Invoke((RSGrid, LayerMask) input)
    {
        RSGrid rsgrid = input.Item1;
        LayerMask mask = input.Item2;
        Func<Vector<double>, bool> f = x =>
        {
            Vector3 pos = RSUtils.Utils.VToV3(x);
            return Physics.CheckSphere(pos, (float)rsgrid.GetCellSize() / 2f, mask.value);
        };

        return rsgrid.ForAll(f);
    }
}

/// <summary>
/// Generates a visibility map inside the grid given a bitmap of the area
/// 
/// Version 0.3
/// Author: Robin Schmidiger
/// Date: May 2021
/// </summary>
public class VisibilitymapHandler : IHandler<(RSGrid, LayerMask), float[,,]>
{
    /// <summary>
    /// 
    /// </summary>
    /// <param name="input">tuple consisting of a Grid and a valid bitmap that corresponds to the grid</param>
    /// <returns>A multid. array that corresponds to the visibility at each cell in the grid
    /// The value of a cell is the fraction of total cells that are visible from the current cell
    /// </returns>
    public float[,,] Invoke((RSGrid, LayerMask) input)
    {
        RSGrid rsgrid = input.Item1;
        LayerMask mask = input.Item2;

        float total_cells = rsgrid.GetLengths().ToArray().Aggregate((a, b) => a * b);

        // Corresponds to a nested loop, that iterates over all cells for each cell
        // the idea is to cast a ray from each cell to each other cell and see if there is an intersection

        Func<Vector<double>, float> f = x =>
        {
            Func<Vector<double>, int> g = y =>
            {
                Vector3 p1 = RSUtils.Utils.VToV3(x);
                Vector3 p2 = RSUtils.Utils.VToV3(y);
                return Physics.Raycast(p1, p2 - p1, (p2 - p1).magnitude, mask.value) ? 0 : 1;
            };

            // divide the number of successful raycasts by the total number of cells to get the coefficient for the current cell.
            float v = rsgrid.ForAll(g).Cast<int>().Sum() / total_cells;
            //Debug.Log(v);
            return v;
        };

        return rsgrid.ForAll(f);
    }
}

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
        }) ;


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

public class Scanner
{
    public static bool[,,] BuildBitmap(Collider col, float size, LayerMask mask)
    {
        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        //bitmap = new Mat(sizes, MatType.CV_32FC1, 0f);
        bool[,,] bitmap = new bool[sizes[0], sizes[1], sizes[2]];

        if (col != null && size > 0f)
        {
            for (int x = 0; x < sizes[0]; x++)
            {
                for (int y = 0; y < sizes[1]; y++)
                {
                    for (int z = 0; z < sizes[2]; z++)
                    {
                        Vector3 pos = col.bounds.center - col.bounds.extents + (size * new Vector3(x, y, z));
                        if (Physics.CheckSphere(pos, size / 2f, mask.value))
                        {
                            bitmap.SetValue(true, x, y, z);
                        }
                        else
                        {
                            bitmap.SetValue(false, x, y, z);

                        }
                    }
                }
            }


            Debug.Log("bitmap done");
        }
        return bitmap;
    }
    public static float[,,] BuildVisibilityMap(Collider col, float size)
    {
        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        // visibility = new Mat(sizes, MatType.CV_32FC1, 1f);
        float[,,] visibility = new float[sizes[0], sizes[1], sizes[2]];
        for (int x1 = 0; x1 < sizes[0]; x1++)
        {
            for (int y1 = 0; y1 < sizes[1]; y1++)
            {
                for (int z1 = 0; z1 < sizes[2]; z1++)
                {
                    for (int x2 = 0; x2 < sizes[0]; x2++)
                    {
                        for (int y2 = 0; y2 < sizes[1]; y2++)
                        {
                            for (int z2 = 0; z2 < sizes[2]; z2++)
                            {
                                Vector3 pos1 = col.bounds.center - col.bounds.extents + (size * new Vector3(x1, y1, z1));
                                Vector3 pos2 = col.bounds.center - col.bounds.extents + (size * new Vector3(x2, y2, z2));
                                if (!Physics.Raycast(pos1, pos2 - pos1, (pos2 - pos1).magnitude))
                                {
                                    // float v = visibility.At<float>(x1, y1, z1);
                                    // visibility.Set<float>(x1, y1, z1, v + 1);
                                    visibility[x1, y1, z1]++;
                                }
                            }
                        }
                    }
                }
            }
        }

        float minVal = visibility.Cast<float>().Min();
        float maxVal = visibility.Cast<float>().Max();

        for (int x = 0; x < sizes[0]; x++)
        {
            for (int y = 0; y < sizes[1]; y++)
            {
                for (int z = 0; z < sizes[2]; z++)
                {
                    float v = ((float)visibility.GetValue(x, y, z) - minVal) / (maxVal - minVal);
                    visibility.SetValue(v, x, y, z);
                }
            }
        }

        Debug.Log("visibility done");
        return visibility;
    }

    public static float[,,] BuildDistanceTransform(Collider col, float size, bool[,,] bitmap)
    {

        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        float[,,] distanceTransform = new float[sizes[0], sizes[1], sizes[2]];



        for (int x = 0; x < sizes[0]; x++)
        {
            for (int y = 0; y < sizes[1]; y++)
            {
                for (int z = 0; z < sizes[2]; z++)
                {
                    if ((bool)bitmap.GetValue(x, y, z))
                    {
                        distanceTransform.SetValue(0f, x, y, z);
                    }
                    else
                    {
                        distanceTransform.SetValue(Mathf.Infinity, x, y, z);
                    }
                }
            }
        }

        for (int iter = 0; iter < 10; iter++)
        {
            for (int x = 0; x < sizes[0]; x++)
            {
                for (int y = 0; y < sizes[1]; y++)
                {
                    for (int z = 0; z < sizes[2]; z++)
                    {
                        float locmin = Mathf.Infinity;
                        for (int xo = -1; xo <= 1; xo++)
                        {

                            for (int zo = -1; zo <= 1; zo++)
                            {
                                try
                                {
                                    Vector3 o = new Vector3(xo, 0, zo) * size;
                                    locmin = Mathf.Min(locmin, (float)distanceTransform.GetValue(x + xo, y, z + zo) + o.magnitude);
                                }
                                catch (System.IndexOutOfRangeException)
                                {

                                }
                            }
                        }
                        distanceTransform.SetValue(locmin, x, y, z);
                    }
                }
            }
        }


        Debug.Log("distance transform done");
        return distanceTransform;
    }
}