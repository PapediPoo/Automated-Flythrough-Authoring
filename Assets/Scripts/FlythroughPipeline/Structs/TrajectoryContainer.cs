using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using RSUtils;

public struct TrajectoryContainer
{
    public List<Vector<double>> control_points;
    public List<Vector<double>> tour;
    public Vector<double> trajectory;
    public IObjectiveFunction objective;
    public ISimpleWorker<Vector<double>, Tuple<double, Vector<double>>> objective_worker;
    public LBFGS lbfgs;
}
