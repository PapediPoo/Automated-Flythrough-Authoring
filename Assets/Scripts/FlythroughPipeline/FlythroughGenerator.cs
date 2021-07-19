using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using RSUtils;

public enum ScanDisplayType
{
    none,
    bitmap,
    distanceTransform,
    visibility,
    tour,
    trajectory
}

/// <summary>
/// Implements a pipeline that generates a flythrough of an area. The area is defined using the bbox of a collider.
/// The different parameters of this component let you adjust how the flythrough is generated.
/// 
/// TODO:
/// - Make visibility metric generation more performant
/// - Add "masking" to visibility metric, So that points that have already been seen are masked out.
/// 
/// Version: 0.5
/// Author: Robin Schmidiger
/// Date: July 2021
/// </summary>

[RequireComponent(typeof(Collider))]
public class FlythroughGenerator : MonoBehaviour
{
    [SerializeField]
    public MapSettings map_settings;
    public MapContainer map_container;

    [SerializeField]
    public ControlPointSettings control_point_settings;
    public int control_point_layer = 0;

    [SerializeField]
    public TrajectorySettings trajectory_settings;
    public TrajectoryContainer trajectory_container;

    [SerializeField]
    public Transform controlpoint_container;
    public GameObject controlpoint_prefab;

    [Header("debug")]
    [Range(0, .99f)]
    public float _debugHeight = 0.5f;
    public ScanDisplayType _debugType = ScanDisplayType.bitmap;

    public float refine_every = 0.5f;
    private float refine_counter = 0f;

    void Start()
    {
        //GenerateMaps();
        //FindControlPoints();
        //PlanTour();
        //for(int i = 0; i < trajectory_settings.max_iterations; i++)
        //{
        //    RefineTrajectory();
        //}
        //ApplyPath();  
        //FindObjectOfType<FollowPath>().enabled = true;
    }

    private void Update()
    {
        refine_counter += Time.deltaTime;
        if (refine_counter >= refine_every)
        {
            RefineTrajectory();
            refine_counter -= refine_every;
        }
    }

    public void GenerateMaps()
    {
        map_container = new ScannerHandler().Invoke((GetComponent<Collider>(), map_settings.cell_size, map_settings.mask));
    }

    public void FindControlPoints()
    {
        var cp = new ControlPointHandler().Invoke((map_container, control_point_settings));
        ObjectContainer.ToObjectContainer(cp, controlpoint_container, controlpoint_prefab, true, control_point_layer);
    }

    public void PlanTour()
    {
        trajectory_container = new TrajectoryInitializationHandler().Invoke((controlpoint_container, trajectory_settings, map_container));
    }

    public void ApplyPath()
    {
        var fp = FindObjectOfType<FollowPath>();
        var trajectory = trajectory_container.trajectory;

        fp.controlPoints = new List<Vector3>();
        for (int i = 0; i < trajectory.Count; i += 3)
        {
            fp.controlPoints.Add(new Vector3((float)trajectory[i], (float)trajectory[i + 1], (float)trajectory[i + 2]));
        }
    }

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
                if (map_container.bitmap != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        Gizmos.color = Color.white;
                        if (map_container.bitmap[(int)i.At(0), (int)i.At(1), (int)i.At(2)] && Mathf.FloorToInt(_debugHeight * map_container.rsgrid.GetLengths().At(1)) == (int)i.At(1))
                        {
                            Gizmos.DrawCube(Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(map_container.rsgrid.GetCellSize()));
                        }
                        return -1d;
                    };
                    map_container.rsgrid.ForAllIndexed(f, true);
                }
                break;

            // display visibility map
            case ScanDisplayType.visibility:
                if (map_container.visibility != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        float c = map_container.visibility[(int)i.At(0), (int)i.At(1), (int)i.At(2)] * 3f;
                        Gizmos.color = new Color(1f - c, c, 0f);
                        if (Mathf.FloorToInt(_debugHeight * map_container.rsgrid.GetLengths().At(1)) == (int)i.At(1)) {
                            Gizmos.DrawCube(Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)(map_container.rsgrid.GetCellSize()));
                        }

                        return -1d;
                    };
                    map_container.rsgrid.ForAllIndexed(f, true);
                }
                break;

            // display distance transform
            case ScanDisplayType.distanceTransform:
                if(map_container.distancetransform != null)
                {
                    Func<Vector<double>, Vector<float>, double> f = (x, i) =>
                    {
                        //float c = (float)distancetransformmap.GetValue((int)i.At(0), (int)i.At(1), (int)i.At(2));
                        //Gizmos.color = c == 0f ? Color.white : new Color(1f / c, 1f / c, 1f / c);

                        //Gizmos.DrawSphere(RSUtils.Utils.VToV3(x), (float)(rsgrid.GetCellSize() / 2f));
                        if (Mathf.FloorToInt(_debugHeight * map_container.rsgrid.GetLengths().At(1)) == (int)i.At(1))
                        {
                            float cl = 1f / (0.01f + map_container.rsgrid.GetLerp(map_container.rsgrid.GetIndex(x), map_container.distancetransform, 0.01f));
                            Gizmos.color = new Color(cl * 0.05f, 0, 0);
                            Gizmos.DrawCube(Utils.VToV3(x), new Vector3(1, 0.1f, 1) * (float)map_container.rsgrid.GetCellSize());
                        }

                        return -1d;
                    };
                    map_container.rsgrid.ForAllIndexed(f, true);
                }
                break;
            case ScanDisplayType.tour:
                Gizmos.color = Color.red;
                var tour = trajectory_container.tour;
                if (tour != null)
                {
                    for (int i = 1; i < tour.Count; i++)
                    {
                        Gizmos.DrawLine(RSUtils.Utils.VToV3(tour[i-1]), RSUtils.Utils.VToV3(tour[i]));
                    }
                    Gizmos.color = Color.white;
                    foreach (var p in tour)
                    {
                        Gizmos.DrawSphere(Utils.VToV3(p), 0.1f);
                    }
                }
                break;
            case ScanDisplayType.trajectory:
                var trajectory = trajectory_container.trajectory;
                if (trajectory != null)
                {
                    Gizmos.color = Color.blue;
                    for (int i = 3; i < trajectory.Count; i+=3)
                    {
                        Vector3 p1 = new Vector3((float)trajectory[i - 3], (float)trajectory[i - 2], (float)trajectory[i - 1]);
                        Vector3 p2 = new Vector3((float)trajectory[i], (float)trajectory[i + 1], (float)trajectory[i + 2]);
                        Gizmos.DrawLine(p1, p2);
                    }
                }
                break;

            // display green tiles at debug height
            default:
                break;
        }
    }

    public void RefineTrajectory()
    {
        var toh = new TrajectoryOptimizationHandler();

        if (trajectory_container.trajectory != null)
        {
            trajectory_container.trajectory = toh.Invoke((trajectory_container.trajectory, trajectory_container.lbfgs, trajectory_container.objective));
        }
        UpdateLineRenderer(FindObjectOfType<LineRenderer>());
    }

    private void UpdateLineRenderer(LineRenderer lr)
    {
        var trajectory = trajectory_container.trajectory;


        if (lr != null && trajectory != null)
        {
            lr.positionCount = trajectory.Count / 3;
            for (int i = 0; i < trajectory.Count; i += 3)
            {
                lr.SetPosition(i / 3, new Vector3((float)trajectory[i], (float)trajectory[i + 1], (float)trajectory[i + 2]));
            }
        }
    }

}

[CustomEditor(typeof(FlythroughGenerator))]
public class FlythroughGeneratorEditor : Editor
{
    static int tick_count = 0;

    /// <summary>
    /// Adds the trajectory optimization to the editor-mode update routine
    /// </summary>
    private void OnEnable()
    {
        EditorApplication.update += Update;
    }

    /// <summary>
    /// Removes trajectory optimization from editor update routine
    /// </summary>
    private void OnDisable()
    {
        EditorApplication.update -= Update;
    }

    /// <summary>
    /// Des a single optimization step of the trajectory optimization every few "ticks"
    /// Since i.e. time.deltaTime don't work in edit-mode, the tick counting speed depends on the machine
    /// </summary>
    void Update()
    {
        //if(++tick_count > 100)
        //{
        //    tick_count = 0;
        //    ((FlythroughGenerator)target).RefineTrajectory();

        //    UpdateLineRenderer(FindObjectOfType<LineRenderer>());
        //}
    }

    /// <summary>
    /// adds the points of the generated trajectory to the indicated line renderer
    /// </summary>
    /// <param name="lr">The line renderer component</param>
    private void UpdateLineRenderer(LineRenderer lr)
    {
        FlythroughGenerator fg = (FlythroughGenerator)target;
        var trajectory = fg.trajectory_container.trajectory;


        if (lr != null && trajectory != null)
        {
            lr.positionCount = trajectory.Count / 3;
            for (int i = 0; i < trajectory.Count; i += 3)
            {
                lr.SetPosition(i / 3, new Vector3((float)trajectory[i], (float)trajectory[i + 1], (float)trajectory[i + 2]));
            }
        }
    }


    /// <summary>
    /// Redraws the inspector as usual, but adds additionals buttons for the flythrough generation
    /// </summary>
    public override void OnInspectorGUI()
    {


        base.OnInspectorGUI();
        FlythroughGenerator fg = (FlythroughGenerator)target;

        // Generates all the maps in the area of the grid
        if (GUILayout.Button("Generate Maps"))
        {
            fg.GenerateMaps();  
        }

        // calculate the control points
        if (GUILayout.Button("Find Control Points"))
        {
            fg.FindControlPoints();
        }

        if(GUILayout.Button("Plan Tour"))
        {
            fg.PlanTour();
        }

        if (GUILayout.Button("Apply To FollowPath"))
        {
            fg.ApplyPath();
        }
    }
}
