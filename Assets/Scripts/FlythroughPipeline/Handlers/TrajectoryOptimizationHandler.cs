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
//public class TrajectoryOptimizationHandler : IHandler<(List<Vector<double>>, int, IUnconstrainedMinimizer, IObjectiveFunction), List<Vector<double>>>
public class TrajectoryOptimizationHandler : IHandler<(Vector<double>, LBFGS, IObjectiveFunction), Vector<double>>
{
    public Vector<double> Invoke((Vector<double>, LBFGS, IObjectiveFunction) input)
    {
        var control_points = input.Item1;
        var objective = input.Item3;

        var algorithm = input.Item2;

        var result = algorithm.FindMinimum(objective, control_points);

        //return result.MinimizingPoint;
        return result;
    }

    public static IObjectiveFunction BuildObjectiveFunction(List<Vector<double>> control_points, TrajectorySettings settings, MapContainer container)
    {
        var init_guess = CPToInitGuess(control_points, settings.num_trajectory_points);

        int n = init_guess.Count;
        var (Ad, fd, Gd, fgd) = GenerateDistanceMatrices(n, init_guess);
        var (Av, Gv) = GenerateVelocityMatrix(n);
        var (Aa, Ga) = GenerateAccelerationMatrix(n);

        var trajectory_point_array = new Vector<double>[settings.num_trajectory_points];
        for(int i = 0; i < settings.num_trajectory_points; i++)
        {
            trajectory_point_array[i] = Vector<double>.Build.Dense(3);
        }

        var objective = ObjectiveFunction.Gradient(x =>
        {
            var X = Matrix<double>.Build.Dense(n, 1, x.ToArray());
            for(int i = 0; i < settings.num_trajectory_points; i++)
            {
                x.CopySubVectorTo(trajectory_point_array[i], 3 * i, 0, 3);
            }

            var rch = new RaycastHit();
            var height_val = Array.ConvertAll(trajectory_point_array, y => {
                Physics.Raycast(Utils.VToV3(y), Vector3.down, out rch);
                return rch.distance;
            });

            var f_dist = new Func<double, double, double, double>((a, b, y) =>
            {
                return (y - b) * (y - b) * a / (b * b);
            });

            var fd_dist = new Func<double, double, double, double>((a, b, y) =>
            {
                return (y - b) * (2 * a) / (b * b);
            });

            var height_grad = new Converter<float, double[]>(y => {
                return new double[] { 0, 2 * (y - settings.desired_height), 0 };
            });


            var a = 50d;
            var b = 1d;

            var dist_grad = Array.ConvertAll(trajectory_point_array, y => fd_dist(a, b, container.rsgrid.GetLerp(container.rsgrid.GetIndex(y), container.distancetransform, 0f)) * container.rsgrid.GetLerpGradient(container.rsgrid.GetIndex(y), container.distancetransform, 0f));


            double value = (settings.distance_weight * ((X.Transpose() * Ad * X) + (fd.Transpose() * X)).At(0, 0))
          + (settings.velocity_weight * (X.Transpose() * Av * X).At(0, 0))
          + (settings.acceleration_weight * (X.Transpose() * Aa * X).At(0, 0))
          + (settings.collision_weight * Array.ConvertAll(trajectory_point_array, y => f_dist(a, b, container.rsgrid.GetLerp(container.rsgrid.GetIndex(y), container.distancetransform, 0f))).Sum())
          + (settings.height_weight * height_val.Aggregate((y, z) => y + Mathf.Pow(z - settings.desired_height, 2)));

            //Debug.Log(container.rsgrid.GradientGet(container.rsgrid.GetIndex(trajectory_point_array[0]), container.heightmap, 10f));

            Vector<double> gradient = (settings.distance_weight * (Gd * X + fgd).Column(0))
            + (settings.velocity_weight * (Gv * X).Column(0))
            + (settings.acceleration_weight * (Ga * X).Column(0))
            + (settings.collision_weight * Vector<double>.Build.DenseOfEnumerable(dist_grad.SelectMany(y => y)))
            + (settings.height_weight * Vector<double>.Build.DenseOfEnumerable(Array.ConvertAll(height_val, height_grad).SelectMany(y => y)));
            return new Tuple<double, Vector<double>>(value, gradient);
        });

        return objective;
    }

    private static (Matrix<double>, Matrix<double>, Matrix<double>, Matrix<double>) GenerateDistanceMatrices(int n, Vector<double> control_points)
    {
        var A = Matrix<double>.Build.Diagonal(n, n, 1d);
        var f = Matrix<double>.Build.Dense(n, 1, (-2 * control_points).ToArray());
        var G = A + A.Transpose();
        var fg = f;
        return (A, f, G, fg);
    }

    private static (Matrix<double>, Matrix<double>) GenerateVelocityMatrix(int n)
    {
        var A = Matrix<double>.Build.SparseDiagonal(n, 2);
        A[0, 0] = 1; A[1, 1] = 1; A[2, 2] = 1;
        A[n - 1, n - 1] = 1; A[n - 2, n - 2] = 1; A[n - 3, n - 3] = 1;
        for (int i = 0; i < n - 3; i++)
        {
            A[i, i + 3] = -2;
        }
        var G = A + A.Transpose();

        return (A, G);
    }

    private static (Matrix<double>, Matrix<double>) GenerateAccelerationMatrix(int n)
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

        var G = A + A.Transpose();


        return (A, G);
    }

    public static Vector<double> CPToInitGuess(List<Vector<double>> control_points, int num_optimization_points)
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
            init_guess[3 * i + 0] = lix.Interpolate(i * current_dist / (num_optimization_points-1));
            init_guess[3 * i + 1] = liy.Interpolate(i * current_dist / (num_optimization_points - 1));
            init_guess[3 * i + 2] = liz.Interpolate(i * current_dist / (num_optimization_points - 1));

        }

        return Vector<double>.Build.DenseOfArray(init_guess);
    }
}
