using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using RSUtils;

/// <summary>
/// Implements a pipeline that generates a flythrough of an area. The area is defined using the bbox of a collider.
/// The different parameters of this component let you adjust how the flythrough is generated.
/// 
/// TODO:
/// - Make visibility metric generation more performant
/// - Add "masking" to visibility metric, So that points that have already been seen are masked out.
/// 
/// Version: 0.15
/// Author: Robin Schmidiger
/// Date: July 2021
/// </summary>

[RequireComponent(typeof(Collider))]
public class FlythroughGenerator : MonoBehaviour
{
    [SerializeField]
    public MapSettings map_settings;
    public MapContainer map_container;
    private ScannerHandler scanner_handler;

    [SerializeField]
    public ControlPointSettings control_point_settings;
    public int control_point_layer = 0;
    private ControlPointHandler control_point_handler;

    [SerializeField]
    public TrajectorySettings trajectory_settings;
    public TrajectoryContainer trajectory_container;
    private TrajectoryInitializationHandler trajectory_initialization_handler;
    private TrajectoryOptimizationHandler trajectory_optimization_handler;

    [SerializeField]
    public Transform controlpoint_container;
    public GameObject controlpoint_prefab;
    [Header("Realtime Optimization")]
    public float refine_every = 0.5f;
    private float refine_counter = 0f;

    private void Start()
    {
        scanner_handler = new ScannerHandler();
        control_point_handler = new ControlPointHandler();
        trajectory_initialization_handler = new TrajectoryInitializationHandler();
        trajectory_optimization_handler = new TrajectoryOptimizationHandler();
    }

    /// <summary>
    /// Does a single step of the trajectory optimization if the trajectory has been initialized. Also updates the line renderer
    /// </summary>
    private void Update()
    {
        refine_counter += Time.deltaTime;
        if (refine_counter >= refine_every)
        {
            RefineTrajectory();
            refine_counter -= refine_every;
            UpdateLineRenderer(FindObjectOfType<LineRenderer>());
        }
    }

    /// <summary>
    /// Generates the maps for finding the control points
    /// </summary>
    public void GenerateMaps()
    {
        map_container = scanner_handler.Invoke((GetComponent<Collider>(), map_settings.cell_size, map_settings.mask));
    }

    /// <summary>
    /// Finds the control points according to the settings based on the generated maps
    /// </summary>
    public void FindControlPoints()
    {
        var cp = control_point_handler.Invoke((map_container, control_point_settings));
        ObjectContainer.ToObjectContainer(cp, controlpoint_container, controlpoint_prefab, true, control_point_layer);
    }

    /// <summary>
    /// Creates an initial guess for the trajectory optimization
    /// </summary>
    public void PlanTour()
    {
        trajectory_container = trajectory_initialization_handler.Invoke((controlpoint_container, trajectory_settings));
    }

    /// <summary>
    /// Copies the optimized trajectory to the followpath component
    /// </summary>
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
    /// Does one step of the trajectory optimization
    /// </summary>
    public void RefineTrajectory()
    {
        if (trajectory_container.trajectory != null)
        {
            trajectory_container.trajectory = trajectory_optimization_handler.Invoke((trajectory_container.trajectory, trajectory_container.lbfgs, trajectory_container.objective));
        }
    }

    /// <summary>
    /// overwrites the specified line renderer with the points of the flythrough trajectory
    /// </summary>
    /// <param name="lr">The line renderer to be used</param>
    public void UpdateLineRenderer(LineRenderer lr)
    {
        if(lr == null)
        {
            return;
        }

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
            fg.UpdateLineRenderer(FindObjectOfType<LineRenderer>());
        }

        if (GUILayout.Button("Optimize Trajectory"))
        {
            for (int i = 0; i < fg.trajectory_settings.max_iterations; i++)
            {
                fg.RefineTrajectory();
            }
            fg.UpdateLineRenderer(FindObjectOfType<LineRenderer>());
        }

        if (GUILayout.Button("Apply To FollowPath"))
        {
            fg.ApplyPath();
        }
    }
}
