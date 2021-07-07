using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;
using System;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;

/// <summary>
/// DEPRECATED CODE!
/// Initial implementation of the pipeline up to generating flythrough paths without optimization
/// </summary>
public class FloorScanner : MonoBehaviour
{
    // Start is called before the first frame update

    [Header("Map generation")]
    public Collider col;
    public float size;
    // public Texture2D bitmap;

    //public Mat bitmap;
    public bool[,,] bitmap;
    public float[,,] distanceTransform;
    // public Mat visibility;
    public float[,,] visibility;
    public LayerMask mask;

    [Header("Scoring")]
    public float bitmapWeight = 1000;
    public float visibilityWeight = 150;
    public float distanceTransformWeight = 100;
    public float minDistanceWeight = 100;

    [Header("Optimization")]
    public double gradientTolerance = 1e-2;
    public double parameterTolerance;
    public double functionProgressTolerance;
    public int gradientIterations = 100;


    public int numControlPoints = 10;
    public Vector3 guess;
    public List<Vector<double>> controlPoints = new List<Vector<double>>();
    public Vector3 controlPoint;

    [Header("Debug Info")]
    [Range(0f, .99f)]
    [SerializeField]
    float _debugHeight = 0f;
    [SerializeField]
    ScanDisplayType _debugType = ScanDisplayType.bitmap;
    public float maxScore = 1000f;

    private void OnDrawGizmosSelected()
    {

        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        if (col != null && bitmap != null && size > 0f)
        {
            int y = (int)(_debugHeight * sizes[1]);
            for (int x = 0; x < sizes[0]; x++)
            {
                for (int z = 0; z < sizes[2]; z++)
                {
                    Vector3 pos = col.bounds.center - col.bounds.extents + (new Vector3(x + .5f, y + .5f, z + .5f) * size);
                    float c;

                    switch (_debugType)
                    {
                        case ScanDisplayType.bitmap:
                            //Gizmos.color = ((bool)bitmap.GetValue(x,y,z)) ? Color.white : Color.black;
                            if ((bool)bitmap.GetValue(x, y, z))
                            {
                                Gizmos.color = Color.white;
                            }
                            else
                            {
                                Gizmos.color = Color.black;
                            }
                            break;
                        case ScanDisplayType.distanceTransform:
                            c = (float)distanceTransform.GetValue(x, y, z);
                            if (c == 0f)
                            {
                                Gizmos.color = Color.white;
                            }
                            else
                            {
                                c = 1f / c;
                                Gizmos.color = new Color(c, c, c);
                            }
                            break;
                        case ScanDisplayType.visibility:
                            c = (float)visibility.GetValue(x, y, z);
                            Gizmos.color = new Color(1f - c, c, 0f);
                            break;
                        //case ScanDisplayType.score:
                        //    c = Mathf.InverseLerp(0, maxScore, (float)Score(RSUtils.Utils.V3ToV(pos)));
                        //    Gizmos.color = new Color(c, 1f - c, 0);
                        //    break;
                        default:
                            break;
                    }
                    if (_debugType != ScanDisplayType.none)
                    {
                        Gizmos.DrawCube(pos, new Vector3(1f, 0.1f, 1f) * size);
                    }
                    //Gizmos.DrawSphere(pos, 0.1f);
                }
            }

            if(controlPoint != null)
            {
                Gizmos.color = Color.blue;
                foreach(Vector<double> p in controlPoints)
                {
                    Gizmos.DrawSphere(RSUtils.Utils.VToV3(p), .5f);
                }
            }
        }
        //Gizmos.DrawSphere(controlPoint, 1f);
    }
}

[CustomEditor(typeof(FloorScanner))]
public class FloorScannerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        FloorScanner fs = (FloorScanner)target;
        if (GUILayout.Button("Generate Maps"))
        {

            fs.bitmap = Scanner.BuildBitmap(fs.col, fs.size, fs.mask);
            fs.visibility = Scanner.BuildVisibilityMap(fs.col, fs.size);
            fs.distanceTransform = Scanner.BuildDistanceTransform(fs.col, fs.size, fs.bitmap);
        }

        if(GUILayout.Button("Generate Control Point"))
        {
            //ControlPointHandler cph = new ControlPointHandler(fs.col, fs.size, fs.bitmap, fs.visibility, fs.distanceTransform, fs.bitmapWeight, fs.visibilityWeight, fs.distanceTransformWeight, fs.minDistanceWeight);
            //fs.controlPoints = cph.FindControlPoint(fs.numControlPoints);
        }

        if(GUILayout.Button("Generate Paths"))
        {
            GraphGenerator gg = FindObjectOfType<GraphGenerator>();
            List<Vector3> cps = fs.controlPoints.ConvertAll(x => RSUtils.Utils.VToV3(x) - Vector3.up);
            gg.GenerateConnectivityGraph(cps);
        }

        if(GUILayout.Button("Plan Path"))
        {
            GraphGenerator gg = FindObjectOfType<GraphGenerator>();
            FollowPath fp = FindObjectOfType<FollowPath>();
            List<Transform> ts = gg.PlanPath(gg.qgraph, gg.qcost);
            // fp.SetCP(ts);
        }
    }
}
