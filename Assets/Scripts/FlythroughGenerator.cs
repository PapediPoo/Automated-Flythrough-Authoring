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
    tour
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
    public List<GameObject> controlpointobjs;
    public List<Vector<double>> tour;

    [Header("Control Point Placement Scoring")]
    public float bitmapWeight = 10;
    public float visibilityWeight = 6;
    public float distanceTransformWeight = 7;
    public float minDistanceWeight = 20;
    public int numControlPoints = 10;
    public float visibilityGain = 100f;



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
                        float c = (float)distancetransformmap.GetValue((int)i.At(0), (int)i.At(1), (int)i.At(2));
                        Gizmos.color = c == 0f ? Color.white : new Color(1f / c, 1f / c, 1f / c);

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
                        Gizmos.DrawLine(RSUtils.Utils.VToV3(tour[i - 1]), RSUtils.Utils.VToV3(tour[i]));
                    }
                }
                Gizmos.color = Color.white;
                foreach(var p in tour)
                {
                    Gizmos.DrawSphere(RSUtils.Utils.VToV3(p), 0.1f);
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
            fg.distancetransformmap = new DistanceTransformHandler().Invoke((fg.rsgrid, fg.bitmap));

        }

        // calculate the control points
        if(GUILayout.Button("Find Control Points"))
        {
            fg.controlpoints = new ControlPointHandler().Invoke((fg.rsgrid, fg.bitmap, fg.bitmapWeight, fg.visibilitymap, fg.visibilityWeight, fg.distancetransformmap, fg.distanceTransformWeight, fg.minDistanceWeight, fg.numControlPoints));

            fg.controlpointobjs = new List<GameObject>();
            foreach(var p in fg.controlpoints)
            {
                GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj.transform.localScale = Vector3.one * 0.4f;
                obj.transform.parent = fg.transform;
                obj.transform.position = RSUtils.Utils.VToV3(p);

                fg.controlpointobjs.Add(obj);
            }
        }

        if(GUILayout.Button("Plan Tour"))
        {
            fg.tour = new TourPlannerHandler().Invoke(fg.controlpointobjs.ConvertAll(x => RSUtils.Utils.V3ToV(x.transform.position)));
        }

        if(GUILayout.Button("Apply To FollowPath"))
        {
            FollowPath fp = FindObjectOfType<FollowPath>();
            fp.controlPoints = new List<Transform>();
            foreach(var p in fg.tour)
            {
                var obj = new GameObject();
                obj.transform.position = RSUtils.Utils.VToV3(p);
                obj.transform.parent = fp.transform;
                fp.controlPoints.Add(obj.transform);
            }
        }
    }
}
