using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System;

public class RSGrid
{
    /// <summary>
    /// Describes a 3d grid and provides helper functions for applying functions for each cell of the grid
    /// Version: 0.1
    /// Author: Robin Schmidiger
    /// Date: Mai 2021
    /// </summary>

    double cell_size;
    Vector<double> corner;
    Vector<float> lengths;  // Note: mathnet doesn't support int vectors as of yet. Just use float for now
    IEnumerable<Vector<double>> all_positions;

    public Vector<double> GetPosition(Vector<float> at)
    {
        return corner + at.Map(x => (double)x + 0.5f).Multiply(cell_size);
    }

    public Vector<float> GetIndex(Vector<double> at)
    {
        return ((at - corner) / cell_size).Map(x => (float)x - 0.5f);
    }

    public float InterpolateGet(Vector<float> i, float[,,] map, float default_value)
    {
        var k = i.Map(x => Mathf.Floor(x));

        var f = new Func<float, float, float, float>((x, y, z) =>
        {
            try
            {
                return map[(int)x, (int)y, (int)z];
            }
            catch (IndexOutOfRangeException)
            {
                return default_value;
            }
        });
        var v = RSUtils.Utils.TriLerp(
            new float[] { i.At(0) % 1, i.At(1) % 1, i.At(2) % 1 },
            new float[] {
                f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 0),
                f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 1),
                f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 0),
                f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 1),
                f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 0),
                f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 1),
                f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 0),
                f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 1)
            }
            );
        if(v == Mathf.Infinity)
        {
            return default_value;
        }
        //Debug.Log(v);
        return v;
    }

    public RSGrid(double cell_size, Vector<double> corner, Vector<float> lengths)
    {
        this.cell_size = cell_size;
        this.corner = corner;
        this.lengths = lengths;
    }

    /// <summary>
    /// Executes the function f for each cell in the grid
    /// The parameter of f is the world position of the cell
    /// Stores the results of applying f as a multidim. array and returns it
    /// NOTE: Allocates memory
    /// </summary>
    /// <typeparam name="T">Return type of function f</typeparam>
    /// <param name="f">Function to be applied for each cell. takes 3D position vector as input</param>
    /// <returns>Returns multidim. array with f applied to each cell position</returns>
    public T[,,] ForAll<T>(Func<Vector<double>, T> f) {
        return ForAllIndexed((x, _) => f(x));
    }

    /// <summary>
    /// Similarly to ForAll, applies f for each cell in the grid.
    /// It supplies f with the positin AND the index of the cell.
    /// Stores the results of applying f as a multidim. array and returns it
    /// NOTE: Allocates memory
    /// </summary>
    /// <typeparam name="T">Return type of function f</typeparam>
    /// <param name="f">Function to be applied for each cell position and index. Takes 3D position vector and index vector as input. NOTE: index vector is a float vector</param>
    /// <returns>Returns multidim. array with f applied to each cell position AND index</returns>
    public T[,,] ForAllIndexed<T>(Func<Vector<double>, Vector<float>, T> f)
    {
        T[,,] output = new T[(int)lengths.At(0), (int)lengths.At(1), (int)lengths.At(2)];
        for (int u = 0; u < lengths.At(0); u++)
        {
            for (int v = 0; v < lengths.At(1); v++)
            {
                for (int w = 0; w < lengths.At(2); w++)
                {
                    Vector<float> vec = Vector<float>.Build.DenseOfArray(new float[] { u, v, w });
                    output[u, v, w] = f(GetPosition(vec), vec);
                }
            }
        }
        return output;
    }

    public double GetCellSize()
    {
        return cell_size;
    }

    public Vector<double> GetLowerBound()
    {
        return corner;
    }

    public Vector<double> GetUpperBound()
    {
        return corner + (cell_size * lengths.Map(x => (double)x));
    }

    public Vector<float> GetLengths()
    {
        return lengths;
    }

    /// <summary>
    /// Helper function for constructing a grid using the bbox of a collider and the cell size
    /// </summary>
    /// <param name="col">Collider to be used for the extents of the grid</param>
    /// <param name="cell_size">unit size of a single cell in the grid. Cells are perfect 3d-cubes</param>
    /// <returns>An insance of RSGrid that fits inside the param col.</returns>
    public static RSGrid BuildFromCollider(Collider col, float cell_size)
    {
        Vector<double> corner = RSUtils.Utils.V3ToV(col.bounds.center - col.bounds.extents);
        Vector<double> lengths = RSUtils.Utils.V3ToV(col.bounds.size);
        return new RSGrid(cell_size, corner, lengths.Map(x => Mathf.Floor((float)x / cell_size)));
    }
}
