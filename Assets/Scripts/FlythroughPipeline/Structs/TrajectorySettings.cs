using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct TrajectorySettings
{
    public int trajectory_point_count;
    public int max_iterations;
    public float desired_height;
    public bool backtrack;
    public LayerMask mask;
    public double lbfgs_gradient_termination_threshold;

    [Range(0.01f, 1)]
    public double distance_weight;
    [Range(0, 1)]
    public double velocity_weight;
    [Range(0, 10)]
    public double acceleration_weight;
    [Range(0, 100)]
    public double collision_weight;
    [Range(0, 1)]
    public double height_weight;

    [Range(0, 1)]
    public double collision_max_dist;

}
