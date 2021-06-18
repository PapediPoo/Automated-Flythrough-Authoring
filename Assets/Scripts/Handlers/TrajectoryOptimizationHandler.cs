using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.ObjectiveFunctions;
using MathNet.Numerics.Interpolation;
using System.Linq;
using System;
public class TrajectoryOptimizationHandler : IHandler<(List<Vector<double>>, int, IUnconstrainedMinimizer, IObjectiveFunction), List<Vector<double>>>
{

    public List<Vector<double>> Invoke((List<Vector<double>>, int, IUnconstrainedMinimizer, IObjectiveFunction) input)
    {
        var control_points = input.Item1;
        var num_trajectory_points = input.Item2;

        var lower_bound = Vector<double>.Build.Dense(3 * num_trajectory_points, -100);
        var upper_bound = Vector<double>.Build.Dense(3 * num_trajectory_points, 100);
        //var upper_bound = input.Item3.GetUpperBound();
        var init_guess = CPToInitGuess(control_points, num_trajectory_points);

        IUnconstrainedMinimizer algorithm = input.Item3;
        var objective = new ForwardDifferenceGradientObjectiveFunction(input.Item4, lower_bound, upper_bound);
        var result = algorithm.FindMinimum(objective, init_guess);

        List<Vector<double>> optimized_trajectory = new List<Vector<double>>();
        for(int i = 0; i < num_trajectory_points;i++)
        {
            optimized_trajectory.Add(Vector<double>.Build.DenseOfArray(
                new double[] {
                    result.MinimizingPoint.At(3 * i + 0),
                    result.MinimizingPoint.At(3 * i + 1),
                    result.MinimizingPoint.At(3 * i + 2)
                    //init_guess.At(3 * i + 0),
                    //init_guess.At(3 * i + 1),
                    //init_guess.At(3 * i + 2),
                }));
        }

        return optimized_trajectory;
    }

    public static IUnconstrainedMinimizer BuildMinimizer(double function_progress_tolerance, double gradient_tolerance, double parameter_tolerance, int max_iterations)
    {
        return new BfgsMinimizer(gradient_tolerance, parameter_tolerance, function_progress_tolerance, max_iterations);
    }

    public static IObjectiveFunction BuildObjectiveFunction(List<Vector<double>> control_points, double distance_weight, double velocity_weight, double acceleration_weight, RSGrid rsgrid, float[,,] distance_transform , double collision_weight, int num_optimization_points)
    {
        var init_guess = CPToInitGuess(control_points, num_optimization_points);
        //ObjectiveFunction.Value(x => Score(x, init_guess, 1f, 1f, 1f, 1f));
        //var times = new double[control_points.Count];
        //times[0] = 0;
        //double current_dist = 0d;
        //for (int i = 1; i < control_points.Count; i++)
        //{
        //    current_dist += (control_points[i - 1] - control_points[i]).L2Norm();
        //    times[i] = current_dist;
        //}
        //double max_time = current_dist;

        //var control_point_interpolation = new IInterpolation[]
        //{
        //    LinearSpline.Interpolate(times, control_points.ConvertAll(x => x.At(0))),
        //    LinearSpline.Interpolate(times, control_points.ConvertAll(x => x.At(1))),
        //    LinearSpline.Interpolate(times, control_points.ConvertAll(x => x.At(2)))
        //};

        int n = init_guess.Count;
        var (Ad, Fd) = GenerateDistanceMatrices(n, init_guess);
        var Av = GenerateVelocityMatrix(n);
        var Aa = GenerateAccelerationMatrix(n);

        var trajectory_point_array = new Vector<double>[num_optimization_points];
        for(int i = 0; i < num_optimization_points; i++)
        {
            trajectory_point_array[i] = Vector<double>.Build.Dense(3);
        }

        return ObjectiveFunction.Value(x =>
        {
            var X = Matrix<double>.Build.Dense(n, 1, x.ToArray());
            for(int i = 0; i < num_optimization_points; i++)
            {
                x.CopySubVectorTo(trajectory_point_array[i], 3 * i, 0, 3);
            }
            ////return (distance_weight * DistanceScore(X, control_point_interpolation, max_time)) + ();
            //return (distance_weight * DistanceScore(X, init_guess))
            //+ (velocity_weight * VelocityScore(X))
            //+ (acceleration_weight * AccelerationScore(X));
            return (distance_weight * ((.5d * X.Transpose() * Ad * X) + (Fd.Transpose() * X)).At(0, 0))
            + (velocity_weight * (.5d * X.Transpose() * Av * X).At(0, 0))
            + (acceleration_weight * (.5d * X.Transpose() * Aa * X).At(0, 0))
            + (collision_weight * Array.ConvertAll(trajectory_point_array, x => 1d / (rsgrid.InterpolateGet(rsgrid.GetIndex(x), distance_transform, 0.01f) + 0.01f)).Sum());

        });
    }

    private static double DistanceScore(Matrix<double> X, Vector<double> control_points)
    {
        int n = X.RowCount;
        var A = Matrix<double>.Build.Diagonal(n, n, Enumerable.Repeat(2d, n).ToArray());
        var f = Matrix<double>.Build.Dense(n, 1, (-2 * control_points).ToArray());

        var Ed = (0.5d * (X.Transpose() * A * X)) + (f.Transpose() * X);
        return Ed.At(0, 0);
    }

    private static (Matrix<double>, Matrix<double>) GenerateDistanceMatrices(int n, Vector<double> control_points)
    {
        var A = Matrix<double>.Build.Diagonal(n, n, Enumerable.Repeat(2d, n).ToArray());
        var f = Matrix<double>.Build.Dense(n, 1, (-2 * control_points).ToArray());
        return (A, f);
    }

    /// <summary>
    /// Evaluates the following system:
    /// v = X^T A X
    /// where A =
    /// | 1 -2  0 ..  0 |
    /// | 0  2 -2 ..  0 |
    /// ..
    /// | 0  0  0 .. -2 |
    /// | 0  0  0 ..  1 |
    /// </summary>
    /// <param name="X">The X for evaluating the system</param>
    /// <returns>returns the value of v</returns>
    private static double VelocityScore(Matrix<double> X)
    {
        int n = X.RowCount;
        var A = Matrix<double>.Build.SparseDiagonal(n, 2);
        A[0, 0] = 1; A[1, 1] = 1; A[2, 2] = 1;
        A[n - 1, n - 1] = 1; A[n - 2, n - 2] = 1; A[n - 3, n - 3] = 1;
        for (int i = 0; i < n - 3; i++)
        {
            A[i, i + 3] = -2;
        }

        var Ev = 0.5d * (X.Transpose() * A * X);

        return Ev.At(0, 0);
    }

    private static Matrix<double> GenerateVelocityMatrix(int n)
    {
        var A = Matrix<double>.Build.SparseDiagonal(n, 2);
        A[0, 0] = 1; A[1, 1] = 1; A[2, 2] = 1;
        A[n - 1, n - 1] = 1; A[n - 2, n - 2] = 1; A[n - 3, n - 3] = 1;
        for (int i = 0; i < n - 3; i++)
        {
            A[i, i + 3] = -2;
        }
        return A;
    }

    /// <summary>
    /// Evaluates the following system:
    /// v = X^T A X
    /// where A =
    /// | 1 -4  2  0 ..  0  0 |
    /// | 0  5 -8  2 ..  0  0 |
    /// | 0  0  6 -8 ..  0  0 |
    /// ..
    /// | 0  0  0  0 ..  5 -4 |
    /// | 0  0  0  0 ..  0  1 |
    /// </summary>
    /// <param name="X">The X for evaluating the system</param>
    /// <returns>returns the value of v</returns>
    public static double AccelerationScore(Matrix<double> X)
    {
        int n = X.RowCount;
        var A = Matrix<double>.Build.SparseDiagonal(n, 6);
        A[0, 0] = 1; A[1, 1] = 1; A[2, 2] = 1;
        A[n - 1, n - 1] = 1; A[n - 2, n - 2] = 1; A[n - 3, n - 3] = 1;
        A[3, 3] = 5; A[4, 4] = 5; A[5, 5] = 5;
        A[n - 4, n - 4] = 5; A[n - 5, n - 5] = 5; A[n - 6, n - 6] = 5;
        for (int i = 0; i < n - 3; i++)
        {
            A[i, i + 3] = -8;
        }
        A[0, 3] = -4; A[1, 4] = -4; A[2, 5] = -4;
        A[n - 4, n - 1] = -4; A[n - 5, n - 2] = -4; A[n - 6, n - 3] = -4;

        for (int i = 0; i < n - 6; i++)
        {
            A[i, i + 6] = 2;
        }

        var Ea = X.Transpose() * A * X;
        return Ea.At(0, 0);

    }

    private static Matrix<double> GenerateAccelerationMatrix(int n)
    {
        var A = Matrix<double>.Build.SparseDiagonal(n, 6);
        A[0, 0] = 1; A[1, 1] = 1; A[2, 2] = 1;
        A[n - 1, n - 1] = 1; A[n - 2, n - 2] = 1; A[n - 3, n - 3] = 1;
        A[3, 3] = 5; A[4, 4] = 5; A[5, 5] = 5;
        A[n - 4, n - 4] = 5; A[n - 5, n - 5] = 5; A[n - 6, n - 6] = 5;
        for (int i = 0; i < n - 3; i++)
        {
            A[i, i + 3] = -8;
        }
        A[0, 3] = -4; A[1, 4] = -4; A[2, 5] = -4;
        A[n - 4, n - 1] = -4; A[n - 5, n - 2] = -4; A[n - 6, n - 3] = -4;

        for (int i = 0; i < n - 6; i++)
        {
            A[i, i + 6] = 2;
        }
        return A;
    }

    private static Vector<double> CPToInitGuess(List<Vector<double>> control_points, int num_optimization_points)
    {
        List<double> time = new List<double>();
        time.Add(0);
        double current_dist = 0d;
        for(int i = 1; i < control_points.Count; i++)
        {
            current_dist += (control_points[i - 1] - control_points[i]).L2Norm();
            time.Add(current_dist);
        }

        var lix = LinearSpline.Interpolate(time, control_points.ConvertAll(x => (double)x.At(0)));
        var liy = LinearSpline.Interpolate(time, control_points.ConvertAll(x => (double)x.At(1)));
        var liz = LinearSpline.Interpolate(time, control_points.ConvertAll(x => (double)x.At(2)));

        double[] init_guess = new double[3 * num_optimization_points];
        for(int i = 0; i < num_optimization_points; i++)
        {
            init_guess[3 * i + 0] = lix.Interpolate(i * current_dist / num_optimization_points);
            init_guess[3 * i + 1] = liy.Interpolate(i * current_dist / num_optimization_points);
            init_guess[3 * i + 2] = liz.Interpolate(i * current_dist / num_optimization_points);

        }

        return Vector<double>.Build.DenseOfArray(init_guess);
    }
}
