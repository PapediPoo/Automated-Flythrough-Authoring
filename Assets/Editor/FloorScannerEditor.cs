using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

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

        if (GUILayout.Button("Generate Control Point"))
        {
            //ControlPointHandler cph = new ControlPointHandler(fs.col, fs.size, fs.bitmap, fs.visibility, fs.distanceTransform, fs.bitmapWeight, fs.visibilityWeight, fs.distanceTransformWeight, fs.minDistanceWeight);
            //fs.controlPoints = cph.FindControlPoint(fs.numControlPoints);
        }

        if (GUILayout.Button("Generate Paths"))
        {
            GraphGenerator gg = FindObjectOfType<GraphGenerator>();
            List<Vector3> cps = fs.controlPoints.ConvertAll(x => RSUtils.Utils.VToV3(x) - Vector3.up);
            gg.GenerateConnectivityGraph(cps);
        }

        if (GUILayout.Button("Plan Path"))
        {
            GraphGenerator gg = FindObjectOfType<GraphGenerator>();
            FollowPath fp = FindObjectOfType<FollowPath>();
            List<Transform> ts = gg.PlanPath(gg.qgraph, gg.qcost);
            // fp.SetCP(ts);
        }
    }
}

