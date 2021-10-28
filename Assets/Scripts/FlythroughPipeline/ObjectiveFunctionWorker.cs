using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Interpolation;

public class ObjectiveFunctionWorker : ISimpleWorker<Vector<double>, Tuple<double, Vector<double>>>
{
    private TrajectoryInitializationHandler.ObjectiveFunctionContainer ofc;
    private TrajectorySettings settings;
    int progress = 0;
    double result_acc;
    Vector<double> gradient_acc;

    Vector<double> x;
    Matrix<double> X;

    public ObjectiveFunctionWorker(List<Vector<double>> tour, TrajectorySettings settings)
    {
        this.settings = settings;
        ofc = TrajectoryInitializationHandler.ObjectiveFunctionAlloc(tour, settings);
        result_acc = 0;
        gradient_acc = Vector<double>.Build.Dense(this.settings.trajectory_point_count * 3);
    }


    public void Start(Vector<double> x)
    {
        result_acc = 0;
        progress = 0;

        int n = settings.trajectory_point_count * 3;
        this.x = x;
        X = Matrix<double>.Build.Dense(n, 1, x.ToArray());
        for (int i = 0; i < settings.trajectory_point_count; i++)
        {
            x.CopySubVectorTo(ofc.trajectory_point_array[i], 3 * i, 0, 3);
        }

        result_acc += (settings.distance_weight * ((X.Transpose() * ofc.Ad * X) + (ofc.fd.Transpose() * X)).At(0, 0))
          + (settings.velocity_weight * (X.Transpose() * ofc.Av * X).At(0, 0))
          + (settings.acceleration_weight * (X.Transpose() * ofc.Aa * X).At(0, 0));

        gradient_acc = (settings.distance_weight * (ofc.Gd * X + ofc.fgd).Column(0))
            + (settings.velocity_weight * (ofc.Gv * X).Column(0))
            + (settings.acceleration_weight * (ofc.Ga * X).Column(0));
    }

    public Tuple<double, Vector<double>> GetResult()
    {
        return new Tuple<double, Vector<double>>(result_acc, gradient_acc);
    }

    public bool IsDone()
    {
        return progress >= settings.trajectory_point_count;
    }



    public void Work()
    {
        int n = settings.trajectory_point_count * 3;
        progress += settings.physics_computations_per_step;
    }
}
