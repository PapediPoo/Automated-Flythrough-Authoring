using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using MathNet.Numerics.LinearAlgebra;

public enum ScanDisplayType
{
    none,
    bitmap,
    distanceTransform,
    visibility,
    score,
    controlpoints,
    tour,
    trajectory
}

/// <summary>
/// Implements a pipeline that generates a flythrough of an area. The area is defined using the bbox of a collider.
/// The different parameters of this component let you adjust how the flythrough is generated.
/// 
/// TODO:
/// - add more comments
/// - Implement trajectory optimization
/// - Port over baseline of trajectory optimization from deprecated script "FloorScanner"
/// - Make visibility metric generation more performant
/// - Add "masking" to visibility metric, So that points that have already been seen are masked out.
/// 
/// Version: 0.3
/// Author: Robin Schmidiger
/// Date: June 2021
/// </summary>

[RequireComponent(typeof(Collider))]
public class FlythroughGenerator : MonoBehaviour
{
    public LayerMask mask;
    public float cell_size = 1f;
    public RSGrid rsgrid;
    public bool[,,] bitmap;
    public float[,,] visibilitymap;
    public float[,,] distancetransformmap;
    public List<Vector<double>> controlpoints;
    public List<Vector<double>> tour;
    public List<Vector<double>> trajectory;
    public Vector3 debug_sphere_position;
    public float debug_sphere_amp = 1f;

    [Header("Control Point Placement Scoring")]
    public float bitmapWeight = 10;
    public float visibilityWeight = 6;
    public float distanceTransformWeight = 7;
    public float minDistanceWeight = 20;
    public int numControlPoints = 10;
    public float visibilityGain = 100f;
    public int distance_transform_iterations = 8;

    [Header("Tour Planning")]
    public bool backtrack = false;

    [Header("Trajectory Optimization")]
    public int num_trajectory_points = 100;
    public double gradient_tolerance = 0.1;
    public double parameter_tolerance = 0.1;
    public double function_progress_tolerance = 0.1;
    public int max_iterations = 250;

    [Range(0, 1)]
    public double distance_weight = 0.5d;
    [Range(0, 1)]
    public double velocity_weight = 0.5d;
    [Range(0, 1)]
    public double acceleration_weight = 0.5d;
    [Range(0, 1)]
    public double collision_weight = 0.5d;

    [Header("debug")]
    [Range(0, .99f)]
    public float _debugHeight = 0.5f;
    public ScanDisplayType _debugType = ScanDisplayType.bitmap;

    /// <summary>
    /// Draws the different maps, control points, etc using Unity gizmos
    /// </summary>
    private void OnDrawGizmosSelected()
    {
        /*
         * Note: you will see this a lot:
         *  if (Mathf.FloorToInt(_debugHeight * rsgrid.GetLengths().At(1)) == (int)i.At(1)) {
                Gizmos.DrawCube(RSUtils.Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(rsgrid.GetCellSize()));
            }
         * 
         * What this does is: if the current height is the debug height, draw the gizmos on the current height
         * TODO: Make it readable
         */

        switch (_debugType)
        {
            /// display bitmap
            case ScanDisplayType.bitmap:
                if (bitmap != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        Gizmos.color = Color.white;
                        if (bitmap[(int)i.At(0), (int)i.At(1), (int)i.At(2)] && Mathf.FloorToInt(_debugHeight * rsgrid.GetLengths().At(1)) == (int)i.At(1))
                        {
                            Gizmos.DrawCube(RSUtils.Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(rsgrid.GetCellSize()));
                        }
                        return -1d;
                    };
                    rsgrid.ForAllIndexed(f);
                }
                break;

            // display visibility map
            case ScanDisplayType.visibility:
                if (visibilitymap != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        float c = visibilitymap[(int)i.At(0), (int)i.At(1), (int)i.At(2)] * visibilityGain;
                        Gizmos.color = new Color(1f - c, c, 0f);
                        if (Mathf.FloorToInt(_debugHeight * rsgrid.GetLengths().At(1)) == (int)i.At(1)) {
                            Gizmos.DrawCube(RSUtils.Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(rsgrid.GetCellSize()));
                        }

                        return -1d;
                    };
                    rsgrid.ForAllIndexed(f);
                }
                break;

            // display distance transform
            case ScanDisplayType.distanceTransform:
                if(distancetransformmap != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        //float c = (float)distancetransformmap.GetValue((int)i.At(0), (int)i.At(1), (int)i.At(2));
                        //Gizmos.color = c == 0f ? Color.white : new Color(1f / c, 1f / c, 1f / c);

                        //Gizmos.DrawSphere(RSUtils.Utils.VToV3(x), (float)(rsgrid.GetCellSize() / 2f));
                        if (Mathf.FloorToInt(_debugHeight * rsgrid.GetLengths().At(1)) == (int)i.At(1))
                        {
                            float c = 1f / (0.01f + rsgrid.InterpolateGet(rsgrid.GetIndex(x), distancetransformmap, 0.01f));
                            Gizmos.color = new Color(c * debug_sphere_amp, 0, 0);
                            Gizmos.DrawCube(RSUtils.Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(rsgrid.GetCellSize()));
                        }

                        return -1d;
                    };
                    rsgrid.ForAllIndexed(f);

                    float c = 1f / (0.01f + rsgrid.InterpolateGet(rsgrid.GetIndex(RSUtils.Utils.V3ToV(debug_sphere_position)), distancetransformmap, 0.01f));
                    Gizmos.color = new Color(c * debug_sphere_amp, 0, 0);
                    Handles.Label(debug_sphere_position + Vector3.up, c.ToString());
                    Gizmos.DrawSphere(debug_sphere_position, .5f);
                }
                break;
            case ScanDisplayType.controlpoints:
                // show the generated control points
                if (controlpoints != null)
                {
                    Gizmos.color = Color.blue;
                    foreach (Vector<double> p in controlpoints)
                    {
                        Gizmos.DrawSphere(RSUtils.Utils.VToV3(p), .5f);
                    }
                }
                break;
            case ScanDisplayType.tour:
                Gizmos.color = Color.red;
                if (tour != null)
                {
                    for (int i = 1; i < tour.Count; i++)
                    {
                        Gizmos.DrawLine(RSUtils.Utils.VToV3(tour[i-1]), RSUtils.Utils.VToV3(tour[i]));
                    }
                    Gizmos.color = Color.white;
                    foreach (var p in tour)
                    {
                        Gizmos.DrawSphere(RSUtils.Utils.VToV3(p), 0.1f);
                    }
                }
                break;
            case ScanDisplayType.trajectory:
                if (trajectory != null)
                {
                    Gizmos.color = Color.blue;
                    for (int i = 1; i < trajectory.Count; i++)
                    {
                        Gizmos.DrawLine(RSUtils.Utils.VToV3(trajectory[i - 1]), RSUtils.Utils.VToV3(trajectory[i]));

                    }
                }
                break;

            // display green tiles at debug height
            default:
                if (rsgrid != null)
                {
                    Gizmos.color = Color.green;
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        //Gizmos.DrawSphere(RSUtils.Utils.VToV3(x), (float)(rsgrid.GetCellSize() / 2f));
                        if (Mathf.FloorToInt(_debugHeight * rsgrid.GetLengths().At(1)) == (int)i.At(1))
                        {
                            Gizmos.DrawCube(RSUtils.Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(rsgrid.GetCellSize()));
                        }
                        return -1d;

                    };

                    rsgrid.ForAllIndexed(f);

                }
                break;
        }
    }
}

[CustomEditor(typeof(FlythroughGenerator))]
public class FlythroughGeneratorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        FlythroughGenerator fg = (FlythroughGenerator)target;

        // Generates all the maps in the area of the grid
        if(GUILayout.Button("Generate Maps"))
        {
            fg.rsgrid = RSGrid.BuildFromCollider(fg.GetComponent<Collider>(), fg.cell_size);
            fg.bitmap = new BitmapHandler().Invoke((fg.rsgrid, fg.mask));
            fg.visibilitymap = new VisibilitymapHandler().Invoke((fg.rsgrid, fg.mask));
            fg.distancetransformmap = new DistanceTransformHandler().Invoke((fg.rsgrid, fg.bitmap, fg.distance_transform_iterations));

        }

        // calculate the control points
        if(GUILayout.Button("Find Control Points"))
        {
            fg.controlpoints = new ControlPointHandler().Invoke((fg.rsgrid, fg.bitmap, fg.bitmapWeight, fg.visibilitymap, fg.visibilityWeight, fg.distancetransformmap, fg.distanceTransformWeight, fg.minDistanceWeight, fg.numControlPoints));

        }

        if(GUILayout.Button("Plan Tour"))
        {
            fg.tour = new TourPlannerHandler().Invoke((fg.controlpoints, fg.backtrack));
        }

        if (GUILayout.Button("Generate Trajectory"))
        {
            var algo = TrajectoryOptimizationHandler.BuildMinimizer(fg.function_progress_tolerance, fg.gradient_tolerance, fg.parameter_tolerance, fg.max_iterations);
            var objective = TrajectoryOptimizationHandler.BuildObjectiveFunction(fg.tour, fg.distance_weight, fg.velocity_weight, fg.acceleration_weight, fg.rsgrid, fg.distancetransformmap, fg.collision_weight, fg.num_trajectory_points);

            fg.trajectory = new TrajectoryOptimizationHandler().Invoke((fg.tour, fg.num_trajectory_points, algo, objective));
        }

        if (GUILayout.Button("Apply To FollowPath"))
        {
            FollowPath fp = FindObjectOfType<FollowPath>();
            fp.controlPoints = new List<Transform>();
            for(int i=0;i<fg.trajectory.Count;i++)
            {
                var p = fg.trajectory[i];
                var obj = new GameObject();
                obj.name = i.ToString();
                obj.transform.position = RSUtils.Utils.VToV3(p);
                obj.transform.parent = fp.transform;
                fp.controlPoints.Add(obj.transform);
            }
        }
    }
}
