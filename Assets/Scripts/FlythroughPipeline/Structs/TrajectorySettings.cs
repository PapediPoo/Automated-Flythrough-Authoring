using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// contains the settings needed for the trajectory initialization and optimization
/// </summary>
[System.Serializable]
public struct TrajectorySettings
{
    public int trajectory_point_count;                      // the number of discrete points that make up the trajectory
    public float desired_height;                            // the height above ground that the trajectory should be at
    public bool backtrack;                                  // if the trajectory should go back to the start once all CPs have been visited
    public LayerMask mask;                                  // the layermask of obstacle detection
    public double lbfgs_gradient_termination_threshold;     // the termination threshold for the optimizer

    [Range(0.01f, 1)]
    public double distance_weight;                          // the weight for the tour distance cost term
    [Range(0, 1)]
    public double velocity_weight;                          // the weight for the velocity cost term 
    [Range(0, 10)]
    public double acceleration_weight;                      // the weight for the acceleration cost term
    [Range(0, 100)]
    public double collision_weight;                         // the weight for the collision cost term
    [Range(0, 1)]
    public double height_weight;                            // the weight for the flying height cost term

    [Range(0, 1)]
    public double collision_max_dist;                       // range at which collisions should be considered. Larger ranges have a bigger impact on performance

}
