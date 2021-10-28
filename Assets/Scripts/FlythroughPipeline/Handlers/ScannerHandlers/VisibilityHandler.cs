using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using System;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;

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

        float total_cells = (float)rsgrid.GetLengths().ToArray().Aggregate((a, b) => a * b);

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
