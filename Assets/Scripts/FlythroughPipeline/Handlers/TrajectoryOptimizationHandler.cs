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
    public struct HeightAtPointContainer
    {
        public Func<float, float> f;
        public Func<float, float> df;
        public float MAX_DIST;
        public float MAX_VALUE;
    }

    public struct DistanceAtPointContainer
    {
        public Func<float, float> f;
        public Func<float, float> df;
        public float MAX_DIST;
        public float MAX_VALUE;
        public Collider[] cols;
    }

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



        var Dc = DistanceAtPointAlloc(.5f, 25f, 10);
        var Hc = HeightAtPointAlloc(5f, 10f, settings.desired_height);

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

            var dist_valgrad = Array.ConvertAll(trajectory_point_array, y => DistanceAtPoint(Utils.VToV3(y), Dc, settings));
            var height_valgrad = Array.ConvertAll(trajectory_point_array, y => HeightAtPoint(Utils.VToV3(y), Hc, settings));

            double value = (settings.distance_weight * ((X.Transpose() * Ad * X) + (fd.Transpose() * X)).At(0, 0))
          + (settings.velocity_weight * (X.Transpose() * Av * X).At(0, 0))
          + (settings.acceleration_weight * (X.Transpose() * Aa * X).At(0, 0))
          + (settings.collision_weight * Array.ConvertAll(dist_valgrad, y => y.Item1).Sum())
          + (settings.height_weight * Array.ConvertAll(height_valgrad, y => y.Item1).Sum());

            Vector<double> gradient = (settings.distance_weight * (Gd * X + fgd).Column(0))
            + (settings.velocity_weight * (Gv * X).Column(0))
            + (settings.acceleration_weight * (Ga * X).Column(0))
            + (settings.collision_weight * Vector<double>.Build.DenseOfEnumerable(Array.ConvertAll(dist_valgrad, y => Utils.V3ToV(y.Item2)).SelectMany(y => y)))
            + (settings.height_weight * Vector<double>.Build.DenseOfEnumerable(Array.ConvertAll(height_valgrad, y => Utils.V3ToV(y.Item2)).SelectMany(y => y)));
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

    private static HeightAtPointContainer HeightAtPointAlloc(float MAX_DIST, float MAX_VAL, float desired_height)
    {
        var f = new Func<float, float>(y =>
        {
            return (y - desired_height) * (y - desired_height) * MAX_VAL;
        });

        var df = new Func<float, float>(y =>
        {
            return (y - desired_height) * (2 * MAX_VAL);
        });

        HeightAtPointContainer container;
        container.f = f;
        container.df = df;
        container.MAX_DIST = MAX_DIST;
        container.MAX_VALUE = MAX_VAL;

        return container;
    }

    private static (float, Vector3) HeightAtPoint(Vector3 point, HeightAtPointContainer container, TrajectorySettings settings)
    {

        RaycastHit rch;
        float current_height;
        if(Physics.Raycast(point, Vector3.down, out rch, container.MAX_DIST, settings.mask.value))
        {
            current_height = rch.distance;
        }
        else
        {
            current_height = container.MAX_DIST;
        }

        return (container.f(current_height), Vector3.up * container.df(current_height));
    }

    public static DistanceAtPointContainer DistanceAtPointAlloc(float MAX_DIST, float MAX_VAL, int collider_buffer_size)
    {
        var f = new Func<float, float>(y =>
        {
            if (y < MAX_DIST)
            {
                return (y - MAX_DIST) * (y - MAX_DIST) * MAX_VAL / (MAX_DIST * MAX_DIST);
            }
            else
            {
                return 0;
            }
        });

        var df = new Func<float, float>(y =>
        {
            if (y < MAX_DIST)
            {
                return (y - MAX_DIST) * (2 * MAX_VAL) / (MAX_DIST * MAX_DIST);
            }
            else
            {
                return 0;
            }
        });

        DistanceAtPointContainer container;
        container.f = f;
        container.df = df;
        container.MAX_DIST = MAX_DIST;
        container.MAX_VALUE = MAX_VAL;
        container.cols = new Collider[collider_buffer_size];

        return container;
    }

    public static (float, Vector3) DistanceAtPoint(Vector3 point, DistanceAtPointContainer container, TrajectorySettings settings)
    {

        float val = container.MAX_VALUE;
        Vector3 grad = Vector3.zero;

        int num_cols = Physics.OverlapSphereNonAlloc(point, container.MAX_DIST, container.cols, settings.mask.value);

        for(int i = 0; i < num_cols; i++)
        {
            Collider col = container.cols[i];
            if(col is MeshCollider && !((MeshCollider)col).convex)
            {
                continue;
            }

            Vector3 closest_point = col.ClosestPoint(point);

            float closest_point_dist = (point- closest_point).magnitude;
            if (closest_point_dist < val)
            {
                val = closest_point_dist;
                grad = point - closest_point;
            }
        }
        return (container.f(val), grad * container.df(val));
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
