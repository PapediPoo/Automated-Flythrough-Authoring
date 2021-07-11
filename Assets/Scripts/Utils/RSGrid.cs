using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Diagnostics;

[System.Serializable]
public class RSGrid
{
    //TODO: reduce number of runtime mem allocs!

    /// <summary>
    /// Describes a 3d grid and provides helper functions for applying functions for each cell of the grid
    /// Version: 0.2
    /// Author: Robin Schmidiger
    /// Date: July 2021
    /// </summary>

    public double cell_size;
    public Vector<double> corner;
    public Vector<float> lengths;  // Note: mathnet doesn't support int vectors as of yet. Just use float for now

    /// <summary>
    /// Returns the world space position of the cell specified by the index vector.
    /// Preconditions: index.Count == 3 && 0 <= index[i] < lengths[i]
    /// </summary>
    /// <param name="index">The index for which the position should be evaluated. </param>
    /// <returns></returns>
    public Vector<double> GetPosition(Vector<float> index)
    {
        // Check preconditions
        System.Diagnostics.Debug.Assert(index.Count == 3);
        for(int i = 0; i < index.Count; i++)
        {
            System.Diagnostics.Debug.Assert(0 <= index[i] && index[i] < lengths[i]);
        }

        // Calculate and return actual position
        // Note: the 0.5f ensures that the returned position is at the middle point of the cell
        return corner + index.Map(x => (double)x + 0.5f).Multiply(cell_size);
    }

    /// <summary>
    /// returns the index that maps th the specified position
    /// Precondition: position.Count == 3
    /// </summary>
    /// <param name="position">The position vector for which the index should be returned</param>
    /// <returns></returns>
    public Vector<float> GetIndex(Vector<double> position)
    {
        // check preconditions
        System.Diagnostics.Debug.Assert(position.Count == 3);

        // calculate index from position and return
        return ((position - corner) / cell_size).Map(x => (float)x - 0.5f);
    }

    /// <summary>
    /// When specifying a map and a floating-point index, returns the linearly interpolated value of the map.
    /// If the index is out of bounds, returns the default value
    /// For example: 
    /// map = [[0, 1], [2, 3]], index = [0.5, 0.5]
    /// returns 1.5
    /// </summary>
    /// <param name="index">The index vector to be evaluated at</param>
    /// <param name="map">The map to take values from</param>
    /// <param name="default_value">The default value if the index is OOB</param>
    /// <returns></returns>
    public float GetLerp(Vector<float> index, float[,,] map, float default_value)
    {
        // Check preconditions
        System.Diagnostics.Debug.Assert(index.Count == 3);
        System.Diagnostics.Debug.Assert(map.Rank == index.Count);

        // The input index rounded to the next lower int
        var int_index = index.Map(x => Mathf.Floor(x));

        // Value-retrieving helper function that handles index OOB exceptions
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

        // Do trilinear interpolation
        var v = RSUtils.Utils.TriLerp(
            new float[] { index.At(0) % 1, index.At(1) % 1, index.At(2) % 1 },
            new float[] {
                f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 0),
                f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 1),
                f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 0),
                f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 1),
                f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 0),
                f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 1),
                f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 0),
                f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 1)
            }
            );
        if(float.IsNaN(v))
        {
            return default_value;
        }
        //Debug.Log(v);
        return v;
    }

    /// <summary>
    /// Returns the gradient of the linear interpolation induced by the map at the index position
    /// </summary>
    /// <param name="index">The index at which the gradient should be evaluated</param>
    /// <param name="map">The map for which the gradient should be evaluated</param>
    /// <param name="default_value">The default value for IOOB</param>
    /// <returns></returns>
    public Vector<double> GetLerpGradient(Vector<float> index, float[,,] map, float default_value)
    {
        // Index vector with components rounded to next lower int
        var int_index = index.Map(x => (float)Math.Floor(x));

        // retrieve helper-function that returns the default_value on IOOB
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

        // Gradients along x direction
        float[] xs = new float[]{
            f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 0) - f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 0),
            f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 1) - f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 1),
            f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 0) - f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 0),
            f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 1) - f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 1)
        };

        // Gradients along y direction
        float[] ys = new float[] {
            f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 0) - f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 0),
            f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 1) - f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 1),
            f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 0) - f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 0),
            f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 1) - f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 1)
        };

        // Gradients along z direction
        float[] zs = new float[] {
            f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 1) - f(int_index.At(0) + 0, int_index.At(1) + 0, int_index.At(2) + 0),
            f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 1) - f(int_index.At(0) + 0, int_index.At(1) + 1, int_index.At(2) + 0),
            f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 1) - f(int_index.At(0) + 1, int_index.At(1) + 0, int_index.At(2) + 0),
            f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 1) - f(int_index.At(0) + 1, int_index.At(1) + 1, int_index.At(2) + 0)
        };

        // do bilinear interpolation on all directional gradients and compose the gradient vector
        var xv = RSUtils.Utils.BiLerp(new float[] { index.At(1) % 1, index.At(2) % 1 }, xs);
        var yv = RSUtils.Utils.BiLerp(new float[] { index.At(0) % 1, index.At(2) % 1 }, ys);
        var zv = RSUtils.Utils.BiLerp(new float[] { index.At(0) % 1, index.At(1) % 1 }, zs);
        return Vector<double>.Build.DenseOfArray(new double[] { xv, yv, zv });

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
    /// <param name="void_function">If f doesn't return anything meaningful, set void_function to true. This avoids unnecessary memory allocation.</param>
    /// <returns>Returns multidim. array with f applied to each cell position AND index</returns>
    public T[,,] ForAllIndexed<T>(Func<Vector<double>, Vector<float>, T> f, bool void_function = false)
    {
        // Alloc an output multi-array if needed
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
                    // For some reason rebuilding the vector completely for every iteration is faster than editing the vector components.
                    // Note: this produces a lot of garbage
                    vec = Vector<float>.Build.DenseOfArray(new float[] { u, v, w });

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

    public Vector<double> GetLowerCorner()
    {
        return corner;
    }

    public Vector<double> GetUpperCorner()
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
