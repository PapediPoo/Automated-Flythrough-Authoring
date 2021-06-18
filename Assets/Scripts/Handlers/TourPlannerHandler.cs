using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using RSUtils;
using UnityEngine.AI;
using System.Linq;
using QuikGraph;
using QuikGraph.Algorithms;

public class TourPlannerHandler : IHandler<(List<Vector<double>>, bool), List<Vector<double>>>
{
    public List<Vector<double>> Invoke((List<Vector<double>>, bool) input)
    {
        List<Vector<double>> coarseCPs = input.Item1;
        bool backtrack = input.Item2;

        var paths = new Dictionary<Edge<int>, NavMeshPath>();
        var qgraph = new UndirectedGraph<int, Edge<int>>();
        var qcost = new Dictionary<Edge<int>, float>();

        qgraph.AddVertexRange(Enumerable.Range(0, coarseCPs.Count));

        for(int i = 0; i < coarseCPs.Count; i++)
        {
            for(int j = 0; j < i; j++)
            {
                Vector3 u = Utils.VToV3(coarseCPs[i]) - Vector3.up;
                Vector3 v = Utils.VToV3(coarseCPs[j]) - Vector3.up;

                NavMeshPath p = new NavMeshPath();
                if(!NavMesh.CalculatePath(u, v, NavMesh.AllAreas, p))
                {
                    Debug.LogWarning("Path generation unsuccessful");
                }
                else
                {
                    float w = 0;
                    for (int k = 1; k < p.corners.Length; k++)
                    {
                        w += (p.corners[k - 1] - p.corners[k]).magnitude;
                    }
                    var e = new Edge<int>(i, j);
                    qgraph.AddEdge(e);
                    qcost.Add(e, w);
                    paths.Add(e, p);
                }


            }
        }

        var mstgraph = new UndirectedGraph<int, Edge<int>>();
        mstgraph.AddVerticesAndEdgeRange(qgraph.MinimumSpanningTreeKruskal(e => qcost[e]));

        var coarsetour = new List<int>();
        var finetour = new List<Vector<double>>();
        var labels = new bool[coarseCPs.Count];

        void DFS(int v)
        {
            labels[v] = true;
            coarsetour.Add(v);
            foreach(int w in mstgraph.AdjacentVertices(v))
            {
                if (!labels[w])
                {
                    DFS(w);
                }
            }
            if (coarsetour.Last() != v && (backtrack || labels.All(x => !x)))
            {
                coarsetour.Add(v);
            }
        }
        DFS(0);

        Vector3 lastpos = Utils.VToV3(coarseCPs[coarsetour[0]]);
        for(int i = 1; i < coarsetour.Count; i++)
        {
            Edge<int> e;
            if (!qgraph.TryGetEdge(coarsetour[i - 1], coarsetour[i], out e)) Debug.Log("not found");
            //Debug.Log(e);
            var corners = paths[e].corners;
            if ((corners.First() - lastpos).magnitude > (corners.Last() - lastpos).magnitude) corners = corners.Reverse().ToArray();
            foreach (Vector3 p in corners.Take(corners.Count() - 1))
            {
                finetour.Add(Utils.V3ToV(p + Vector3.up));
            }
            if(i == coarsetour.Count - 1)
            {
                finetour.Add(Utils.V3ToV(corners.Last() + Vector3.up));
            }

            lastpos = corners.Last();
        }

        return finetour;
        //return coarsetour.ConvertAll(x => coarseCPs[x]);
    }
}
