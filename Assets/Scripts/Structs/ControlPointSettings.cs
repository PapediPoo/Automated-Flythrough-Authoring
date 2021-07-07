using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public struct ControlPointSettings
{
    [Range(0f, 1f)]
    public float bitmap_weight;
    [Range(0f, 1f)]
    public float visibility_weight;
    [Range(0f, 1f)]
    public float distance_weight;
    [Range(0f, 1f)]
    public float cp_distance_weight;
    public int num_control_points;
}
