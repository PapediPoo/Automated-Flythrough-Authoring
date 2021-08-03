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
