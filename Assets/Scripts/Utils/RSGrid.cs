using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System;

[System.Serializable]
public class RSGrid
{
    /// <summary>
    /// Describes a 3d grid and provides helper functions for applying functions for each cell of the grid
    /// Version: 0.2
    /// Author: Robin Schmidiger
    /// Date: June 2021
    /// </summary>

    public double cell_size;
    public Vector<double> corner;
    public Vector<float> lengths;  // Note: mathnet doesn't support int vectors as of yet. Just use float for now

    public Vector<double> GetPosition(Vector<float> at)
    {
        return corner + at.Map(x => (double)x + 0.5f).Multiply(cell_size);
    }

    public Vector<float> GetIndex(Vector<double> at)
    {
        return ((at - corner) / cell_size).Map(x => (float)x - 0.5f);
    }

    public float LerpGet(Vector<float> i, float[,,] map, float default_value)
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
        if(float.IsNaN(v))
        {
            return default_value;
        }
        //Debug.Log(v);
        return v;
    }

    public Vector<double> GradientGet(Vector<float> i, float[,,] map, float default_value)
    {
        var k = i.Map(x => (float)Math.Floor(x));
        var f = new Func<float, float, float, float>((x, y, z) =>
        {
            try
            {
                var v = map[(int)x, (int)y, (int)z];
                if(v != Mathf.Infinity)
                {
                    return v;
                }
                else
                {
                    return default_value;
                }
            }
            catch (IndexOutOfRangeException)
            {
                return default_value;
            }
        });
        float[] xs = new float[]{
            f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 0) - f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 0),
            f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 1) - f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 1),
            f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 0) - f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 0),
            f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 1) - f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 1)
        };

        float[] ys = new float[] {
            f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 0) - f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 0),
            f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 1) - f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 1),
            f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 0) - f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 0),
            f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 1) - f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 1)
        };

        float[] zs = new float[] {
            f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 1) - f(k.At(0) + 0, k.At(1) + 0, k.At(2) + 0),
            f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 1) - f(k.At(0) + 0, k.At(1) + 1, k.At(2) + 0),
            f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 1) - f(k.At(0) + 1, k.At(1) + 0, k.At(2) + 0),
            f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 1) - f(k.At(0) + 1, k.At(1) + 1, k.At(2) + 0)
        };

        var xv = RSUtils.Utils.BiLerp(new float[] { i.At(1) % 1, i.At(2) % 1 }, xs);
        var yv = RSUtils.Utils.BiLerp(new float[] { i.At(0) % 1, i.At(2) % 1 }, ys);
        var zv = RSUtils.Utils.BiLerp(new float[] { i.At(0) % 1, i.At(1) % 1 }, zs);

        return Vector<double>.Build.DenseOfArray(new double[] { xv, yv, zv });

    }

    public RSGrid(double cell_size, Vector<double> corner, Vector<float> lengths)
    {
        this.cell_size = cell_size;
        this.corner = corner;
        this.lengths = lengths;

        //this.cornerv3 = RSUtils.Utils.VToV3(corner);
        //this.lengthsv3 = RSUtils.Utils.VToV3(lengths);
    }

    private void Rebuild()
    {
        if(corner != null && lengths != null)
        {
            return;
        }

        //corner = RSUtils.Utils.V3ToV(cornerv3);
        //lengths = RSUtils.Utils.V3ToV(lengthsv3).Map(x => (float)x);
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
    public T[,,] ForAll<T>(Func<Vector<double>, T> f, bool void_function = false) {
        return ForAllIndexed((x, _) => f(x), void_function);
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
    public T[,,] ForAllIndexed<T>(Func<Vector<double>, Vector<float>, T> f, bool void_function = false)
    {
        T[,,] output = null;
        if (!void_function)
        {
            output = new T[(int)lengths.At(0), (int)lengths.At(1), (int)lengths.At(2)];
        }
        Vector<float> vec = Vector<float>.Build.Dense(3);
        for (int u = 0; u < lengths.At(0); u++)
        {
            for (int v = 0; v < lengths.At(1); v++)
            {
                for (int w = 0; w < lengths.At(2); w++)
                {
                    vec = Vector<float>.Build.DenseOfArray(new float[] { u, v, w });
                    //vec.SetValues(new float[] { u, v, w});

                    if (output != null)
                    {
                        output[u, v, w] = f(GetPosition(vec), vec);
                    }
                    else
                    {
                        f(GetPosition(vec), vec);
                    }
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
