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


/// <summary>
/// The pipeline handler responsible for optimizing the flythough trajectory
/// 
/// Author: Robin Schmidiger
/// Date: July 2021
/// Version: 0.32
/// </summary>
public class TrajectoryOptimizationHandler : IHandler<(Vector<double>, LBFGS, IObjectiveFunction), IEnumerable<Vector<double>>>
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

    public IEnumerable<Vector<double>> Invoke((Vector<double>, LBFGS, IObjectiveFunction) input)
    {
        var control_points = input.Item1;
        var objective = input.Item3;

        var algorithm = input.Item2;

        foreach(var r in algorithm.FindMinimum(objective, control_points))
        {
            yield return r;
        }
        //var result = algorithm.FindMinimum(objective, control_points);

        //return result.MinimizingPoint;
        //return result;
    }
}
