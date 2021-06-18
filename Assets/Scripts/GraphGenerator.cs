using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using QuikGraph;
using QuikGraph.Algorithms;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine.AI;
using System.Linq;
using System;


/// <summary>
/// Given a list of control points and using unity's navmesh system it builds a connectivity graph of the environment.
/// This script makes heavy use of the QuickGraph library from most of its functionality.
/// 
/// Version: 0.5
/// Author: Robin Schmidiger
/// Date: May 2021
/// </summary>
[RequireComponent(typeof(NavMeshSurface))]
public class GraphGenerator : MonoBehaviour
{
    public Dictionary<Edge<string>, NavMeshPath> paths;
    public BidirectionalGraph<string, Edge<string>> qgraph;
    public Dictionary<Edge<string>, float> qcost;

    public float CPErrorThreshold = 1f;

    private void Start()
    {
        FloorScanner fs = FindObjectOfType<FloorScanner>();
        List<Vector3> cps = fs.controlPoints.ConvertAll(x => RSUtils.Utils.VToV3(x) - Vector3.up);

        GenerateConnectivityGraph(cps);
    }

    /// <summary>
    /// Given a list of nodes, creates a QuickGraph. The edges correspond to the connectivity/reachablility between the nodes
    /// For each edge a path is generated. The path follows the geometry from the source node to the target node. 
    /// Those paths act as a baseline for the trajectory optimization, as they closely correspond to the fly-paths that should be taken later
    /// 
    /// TODO: 
    /// - Code cleanup
    /// - Generate navmesh agent via script and expose parameters to the user
    /// 
    /// </summary>
    /// <param name="nodePositions">a list of nodes and their global positions</param>
    /// <returns></returns>
    public BidirectionalGraph<string, Edge<string>> GenerateConnectivityGraph(List<Vector3> nodePositions)
    {
        paths = new Dictionary<Edge<string>, NavMeshPath>();        // The paths that correspond to the edges in the graph
        qgraph = new BidirectionalGraph<string, Edge<string>>();    // The actual graph
        qcost = new Dictionary<Edge<string>, float>();              // The costs associated with the edges

        NavMeshSurface nmb = GetComponent<NavMeshSurface>();
        nmb.collectObjects = CollectObjects.Volume;

        // nmb.size = bbox.bounds.size;
        // nmb.center = bbox.bounds.center + bbox.transform.position - transform.position;
        // nmb.agentTypeID = 0;
        // nmb.BuildNavMesh();

        // Foreach node, add a node to the Qgraph
        for (int i = 0; i < nodePositions.Count; i++)
        {
            qgraph.AddVertex(i.ToString());
        }

        // Add the edges and calculate the corresponding paths on the navmesh
        for (int i = 0; i < nodePositions.Count; i++)
        {

            // Only add the edges / generate the paths that haven't previously been generated
            Vector3 u = nodePositions[i];
            IEnumerable vs = nodePositions.TakeWhile(x => x != u);
            for (int j = 0; j < i; j++)
            {
                Vector3 v = nodePositions[j];
                NavMeshPath p = new NavMeshPath();

                if (!NavMesh.CalculatePath(u, v, NavMesh.AllAreas, p))
                {
                    // TODO: Add recovery mechanism if path generation fails
                    //print(u);
                    //print(v);
                    //print("----");
                }

                // Calculate the length of the path and set it as the edge weighte
                float c = 0f;
                for (int k = 1; k < p.corners.Length; k++)
                {
                    c += (p.corners[k - 1] - p.corners[k]).magnitude;
                }

                // paths.Add(p);
                Edge<string> e1 = new Edge<string>(i.ToString(), j.ToString());
                Edge<string> e2 = new Edge<string>(j.ToString(), i.ToString());

                // Add the edges to the qgraph
                qgraph.AddEdge(e1);
                qgraph.AddEdge(e2);
                qcost.Add(e1, c);       // I don't know if i need to add edges in both directions. TODO: Dig in quickgraph documentation
                qcost.Add(e2, c);
            }

        }

        // the following code would remove unnessecary edges. But it is currently pretty buggy.
        //for (int u = 0; u < nodePositions.Count; u++)
        //{
        //    for (int v = 0; v < nodePositions.Count; v++)
        //    {
        //        for (int w = 0; w < nodePositions.Count; w++)
        //        {
        //            Edge<string> uv;
        //            Edge<string> vw;
        //            Edge<string> uw;

        //            if (!qgraph.TryGetEdge(u.ToString(), v.ToString(), out uv)) continue;
        //            if (!qgraph.TryGetEdge(v.ToString(), w.ToString(), out vw)) continue;
        //            if (!qgraph.TryGetEdge(u.ToString(), w.ToString(), out uw)) continue;

        //            try
        //            {
        //                if (u != v && v != w && u != w && Mathf.Abs(qcost[uv] + qcost[vw] - qcost[uw]) < CPErrorThreshold)
        //                {
        //                    //print("removing");
        //                    qgraph.RemoveEdge(uw);
        //                }
        //            }
        //            catch (KeyNotFoundException) { }
        //        }
        //    }
        //}

        // Calculate paths for each edge remaining in the graph
        for (int u = 0; u < nodePositions.Count; u++)
        {
            for (int v = 0; v < nodePositions.Count; v++)
            {
                Edge<string> e;
                if(qgraph.TryGetEdge(u.ToString(), v.ToString(), out e))
                {
                    NavMeshPath p = new NavMeshPath();
                    NavMesh.CalculatePath(nodePositions[u], nodePositions[v], NavMesh.AllAreas, p);
                    paths.Add(e, p);
                    //print("remaining");
                }
            }
        }
        return qgraph;
    }

    /// <summary>
    /// Given a quickgraph, generates a "tour" of the graph where each node is visited at least once.
    /// 
    /// TODO: Do actual travelling salesman implementation of the problem
    /// </summary>
    /// <param name="qgraph">the quickgraph</param>
    /// <param name="qcost">the costs of the edges in the quickgraph</param>
    /// <returns>A list of transforms that correspond to a tour of the graph</returns>
    public List<Transform> PlanPath(BidirectionalGraph<string, Edge<string>> qgraph, Dictionary<Edge<string>, float> qcost)
    {
        List<Edge<string>> cpPath = new List<Edge<string>>();   // The path only consisting of the control points
        List<Transform> fullPath = new List<Transform>();       // The path with control points + intermediate points of the edges connecting the control points (useful for the actual flytrhough as it doesn't go through obstacles)

        bool[] visited = new bool[qgraph.VertexCount];  // Indicates which nodes have already been visited
        int current = 0;                                // Indicates the current node

        // Do pseudo-BFS to visit all nodes
        while (!visited.All(x => x))
        {
            visited[current] = true;
            IEnumerable<Edge<string>> partialCPPath;
            int target = visited.ToList().IndexOf(false);
            qgraph.ShortestPathsDijkstra(x => qcost[x], current.ToString())(target.ToString(), out partialCPPath);
            foreach (Edge<string> e in partialCPPath)
            {
                visited[int.Parse(e.Target)] = true;
                print(e);
                cpPath.Add(e);
                print(cpPath.Count);
            }
            current = target;


            //print(visited.ToString());
            //IEnumerable<Edge<string>> es;
            //qgraph.TryGetOutEdges(current.ToString(), out es);
            //current = int.Parse(es
            //    .Where(e1 => !visited[int.Parse(e1.Target)])
            //    .OrderBy(e2 => qcost[e2])
            //    .First().Target);
        }

        // qgraph.ShortestPathsDijkstra(x => qcost[x], "0")((qgraph.VertexCount - 1).ToString(), out cpPath);

        // helper function that turns the ID string of a control point into a transform
        Func<Vector3, string, Transform> f = (cp, name) =>
        {
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obj.name = name;
            obj.layer = 8;
            Transform t = obj.transform;
            t.position = cp + Vector3.up;
            t.localScale = 0.2f * Vector3.one;
            t.parent = gameObject.transform;
            return t;
        };

        // from the visiting order, generate the path by iterating over all the visited nodes in visitation order.
        fullPath.Add(f(paths[cpPath[0]].corners[0], "start"));
        foreach(Edge<string> e in cpPath)
        {
            
            foreach(Vector3 cp in paths[e].corners.Skip(1))
            {
                fullPath.Add(f(cp, e.ToString() + cp.ToString()));
            }
        }

        return fullPath;
    }

    private void OnDrawGizmosSelected()
    {
        if (paths != null)
        {
            Gizmos.color = Color.green;
            foreach (NavMeshPath path in paths.Values)
            {
                for (int i = 1; i < path.corners.Length; i++)
                {
                    Gizmos.DrawLine(path.corners[i - 1], path.corners[i]);
                }
            }
        }
    }
}
