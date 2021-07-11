using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using RSUtils;

public struct TrajectoryContainer
{
    public List<Vector<double>> tour;
    public Vector<double> trajectory;
    public IObjectiveFunction objective;
    public LBFGS lbfgs;
}
