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
    private ControlPointHandler control_point_handler;

    [SerializeField]
    public TrajectorySettings trajectory_settings;
    public TrajectoryContainer trajectory_container;
    private TrajectoryInitializationHandler trajectory_initialization_handler;
    private TrajectoryOptimizationHandler trajectory_optimization_handler;

    [Header("Realtime Optimization")]
    public float refine_every = 0.5f;
    private bool modified;

    private void Start()
    {
        scanner_handler = new ScannerHandler();
        control_point_handler = new ControlPointHandler();
        trajectory_initialization_handler = new TrajectoryInitializationHandler(trajectory_settings);
        trajectory_optimization_handler = new TrajectoryOptimizationHandler();

        GenerateMaps();
        FindControlPoints();
        PlanTour();
        StartCoroutine("RefineTrajectory");
    }

    /// <summary>
    /// Generates the maps for finding the control points
    /// </summary>
    public void GenerateMaps()
    {
        map_container = scanner_handler.Invoke((GetComponent<Collider>(), map_settings.cell_size, map_settings.mask));
        print("scanning done");
    }

    /// <summary>
    /// Finds the control points according to the settings based on the generated maps
    /// </summary>
    public void FindControlPoints()
    {
        trajectory_container.control_points = control_point_handler.Invoke((map_container, control_point_settings));
        print("control points done");
    }

    /// <summary>
    /// Creates an initial guess for the trajectory optimization
    /// </summary>
    public void PlanTour()
    {
        //if(crt != null)
        //{
        //    StopCoroutine(crt);
        //}
        trajectory_container = trajectory_initialization_handler.Invoke((trajectory_container.control_points, trajectory_settings));
        trajectory_container.lbfgs.Reset();
        modified = true;
        print("tour planning done");
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
    IEnumerator RefineTrajectory()
    {
        var wfs = new WaitForSeconds(refine_every);
        for (; ; )
        {
            if (trajectory_container.trajectory != null)
            {
                Vector<double> tmp = trajectory_container.trajectory;
                foreach(var t in trajectory_optimization_handler.Invoke((trajectory_container.trajectory, trajectory_container.lbfgs, trajectory_container.objective)))
                {
                    tmp = t;
                    yield return wfs;
                }
                if (modified)
                {
                    modified = false;
                }
                else
                {
                    trajectory_container.trajectory = tmp;
                }
                //trajectory_container.trajectory = trajectory_optimization_handler.Invoke((trajectory_container.trajectory, trajectory_container.lbfgs, trajectory_container.objective));
            }
            yield return wfs;
        }
    }

    private void OnDrawGizmosSelected()
    {
        if(trajectory_container.control_points != null)
        {
            foreach(var cp in trajectory_container.control_points)
            {
                Gizmos.DrawSphere(Utils.VToV3(cp), 1f);
                print(Utils.VToV3(cp));
            }
        }
    }
}
