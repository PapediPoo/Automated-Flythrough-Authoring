using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using MathNet.Numerics.LinearAlgebra;

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

    [Header("Scoring")]
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
                        if (c == 0f)
                        {
                            Gizmos.color = Color.white;
                        }
                        else
                        {
                            c = 1f / c;
                            Gizmos.color = new Color(c, c, c);
                        }
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

        // show the generated control points
        if (controlpoints != null)
        {
            Gizmos.color = Color.blue;
            foreach (Vector<double> p in controlpoints)
            {
                Gizmos.DrawSphere(RSUtils.Utils.VToV3(p), .5f);
            }
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
        }
    }
}
