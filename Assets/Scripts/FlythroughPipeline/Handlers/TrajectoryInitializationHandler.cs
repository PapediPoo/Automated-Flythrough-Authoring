using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using System.Linq;

public class TrajectoryInitializationHandler : IHandler<(Transform, TrajectorySettings, MapContainer), TrajectoryContainer>
{
    public TrajectoryContainer Invoke((Transform, TrajectorySettings, MapContainer) input)
    {
        Transform object_container = input.Item1;
        TrajectorySettings ts = input.Item2;
        MapContainer mc = input.Item3;
        TrajectoryContainer tc;

        var ls = ObjectContainer.FromObjectContainer(object_container).ConvertAll(x => Utils.V3ToV(x));
        tc.tour = new TourPlannerHandler().Invoke((ls, ts));
        tc.lbfgs = new LBFGS(10, ts.max_iterations);
        tc.trajectory = TrajectoryOptimizationHandler.CPToInitGuess(tc.tour, ts.num_trajectory_points);
        tc.objective = TrajectoryOptimizationHandler.BuildObjectiveFunction(tc.tour, ts, mc);

        return tc;
    }
}
