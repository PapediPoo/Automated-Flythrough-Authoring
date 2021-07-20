using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using RSUtils;
using UnityEngine.AI;
using System.Linq;
using QuikGraph;
using QuikGraph.Algorithms;

/// <summary>
/// Pipeline handler responsible for generating an initial guess for the trajectory
/// 
/// Author: Robin Schmidiger
/// Date: July 2021
/// Version: 0.4
/// </summary>
public class TourPlannerHandler : IHandler<(List<Vector<double>>, TrajectorySettings), List<Vector<double>>>
{
    /// <summary>
    /// Takes a list of control points and a trajectory settings struct and generates a list of points that corresponds to the initial guess of a flythough-trajectory.
    /// Note
    /// </summary>
    /// <param name="input">A tuple consisting of a list of points and a trajectory settings struct
    /// the elements of the point list are 3-dimensional vectors that denote the xyz coordinates of a point. The order of the list does not matter</param>
    /// <returns>An ordered list of 3-dimensional vectors corresponding to a trajectory of points. the points are not equispaced</returns>
    public List<Vector<double>> Invoke((List<Vector<double>>, TrajectorySettings) input)
    {
        // TODO: produce less garbage / do less allocations
        /*
         * The high level idea behind generating the trajectory is the following:
         * 1. Generate a graph (clique) of the control points. the weights are the path-lengths between the ponts
         * 2. Do DFS with backtrack on MST to find an approximation of a travelling salesman tour ==> coarse tour
         * 3. Insert points on NavMeshPath into coarse tour ==> fine tour
         */

        /// Unpack the input
        List<Vector<double>> coarseCPs = input.Item1;
        TrajectorySettings settings = input.Item2;

        // Allocate space for data structures
        var paths = new Dictionary<Edge<int>, NavMeshPath>();   // Holds all pairwise paths between all control points
        var qgraph = new UndirectedGraph<int, Edge<int>>();     // Holds a representation of the graph induced by the control points. nodes correspond to control points
        var qcost = new Dictionary<Edge<int>, float>();         // Assigns a cost to each edge in the graph. the cost is the distance between the nodes

        qgraph.AddVertexRange(Enumerable.Range(0, coarseCPs.Count));    // foreach control point add a node to the graph
        NavMeshHit nmhu;
        NavMeshHit nmhv;

        // Iterates over all pairwise control points
        for(int i = 0; i < coarseCPs.Count; i++)
        {
            for(int j = 0; j < i; j++)
            {
                // finds the closest point to the control point on the navmesh and writes it to nmhu and nmhv
                NavMesh.SamplePosition(Utils.VToV3(coarseCPs[i]), out nmhu, 3f, NavMesh.AllAreas);
                NavMesh.SamplePosition(Utils.VToV3(coarseCPs[j]), out nmhv, 3f, NavMesh.AllAreas);

                // read out positions on navmesh and write them into points u,v
                Vector3 u = nmhu.position;
                Vector3 v = nmhv.position;

                // generate path uv on navmesh
                NavMeshPath p = new NavMeshPath();
                if(!NavMesh.CalculatePath(u, v, NavMesh.AllAreas, p))
                {
                    Debug.LogWarning("Path generation unsuccessful");
                }
                else
                {
                    // compute lenght of path uv by iterating over its segments
                    float w = 0;
                    for (int k = 1; k < p.corners.Length; k++)
                    {
                        w += (p.corners[k - 1] - p.corners[k]).magnitude;
                    }

                    // add the resulting path to qgraph, qcost and paths data structures
                    var e = new Edge<int>(i, j);
                    qgraph.AddEdge(e);
                    qcost.Add(e, w);
                    paths.Add(e, p);
                }
            }
        }

        // Calculate the MST of qgraph
        var mstgraph = new UndirectedGraph<int, Edge<int>>();
        mstgraph.AddVerticesAndEdgeRange(qgraph.MinimumSpanningTreeKruskal(e => qcost[e]));

        // Alloc data structures for travellign salesman approx
        var coarsetour = new List<int>();
        var finetour = new List<Vector<double>>();
        var labels = new bool[coarseCPs.Count];

        // Implementation of a simple DFS with backtracking
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

            /* the DFS stops adding points if either condition is met:
             * 1. the last point is the current point (don't add neighbouring points twice)
             * 2. all points have been visited
             */
            if (coarsetour.Last() != v && (settings.backtrack || labels.All(x => !x)))
            {
                coarsetour.Add(v);
            }
        }

        // run the DFS ==> coarse tour
        DFS(0);

        // insert points on navmeshpath between points on coarse tour ==> fine tour
        Vector3 lastpos = Utils.VToV3(coarseCPs[coarsetour[0]]);
        for(int i = 1; i < coarsetour.Count; i++)
        {
            Edge<int> e;
            if (!qgraph.TryGetEdge(coarsetour[i - 1], coarsetour[i], out e)) Debug.Log("not found");
            var corners = paths[e].corners;
            if ((corners.First() - lastpos).magnitude > (corners.Last() - lastpos).magnitude) corners = corners.Reverse().ToArray();
            foreach (Vector3 p in corners.Take(corners.Count() - 1))
            {
                finetour.Add(Utils.V3ToV(p + (Vector3.up * settings.desired_height)));
            }

            // Add the last point explicitly (fenceposting problem)
            if(i == coarsetour.Count - 1)
            {
                finetour.Add(Utils.V3ToV(corners.Last() + (Vector3.up * settings.desired_height)));
            }

            lastpos = corners.Last();
        }

        return finetour;
    }
}
