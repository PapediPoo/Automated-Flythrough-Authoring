using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// contains the adjustable settings needed for generating control points
/// </summary>

[System.Serializable]
public struct ControlPointSettings
{
    [Range(0f, 1f)]
    public float bitmap_weight;         // determines how much the bitmap is weighted in the CP optimization
    [Range(0f, 1f)]
    public float visibility_weight;     // determines how much the visibility map is weighted in the CP optimization
    [Range(0f, 1f)]
    public float distance_weight;       // determines how much the distance transform map is weighted in the CP optimization
    [Range(0f, 3f)]
    public float cp_distance_weight;    // determines how much the distance to other control points is weighted in the CP optimization
    public int num_control_points;      // determines the number of control points to be instantiated initially
}
