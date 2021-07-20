using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct TrajectorySettings
{
    public int num_trajectory_points;
    public int max_iterations;
    public float desired_height;
    public bool backtrack;
    public LayerMask mask;

    [Range(0.01f, 1)]
    public double distance_weight;
    [Range(0, 1)]
    public double velocity_weight;
    [Range(0, 1)]
    public double acceleration_weight;
    [Range(0, 1)]
    public double collision_weight;
    [Range(0, 1)]
    public double height_weight;

}
