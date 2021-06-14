using System.Collections;
using System.Linq;
using System;
using System.Collections.Generic;
using UnityEngine;
using QuikGraph;
using QuikGraph.Algorithms;

namespace Graph {

    public class GraphWrapper : MonoBehaviour
    {
        public BidirectionalGraph<string, Edge<string>> qgraph;
        public Func<Edge<string>, double> qcost;
        public FollowPath fp;

        public GameObject start;
        public GameObject end;

        public List<GameObject> nodes = new List<GameObject>();
        
        [SerializeField]
        public List<Vector2Int> edges = new List<Vector2Int>();

        // Start is called before the first frame update
        public void Start()
        {
            qcost = e => (nodes.Find(u => u.name == e.Source).transform.position - nodes.Find(v => v.name == e.Target).transform.position).magnitude;
            GenerateGraph();

            IEnumerable<Edge<string>> lblpath;


            if(qgraph.ShortestPathsDijkstra(qcost, start.name)(end.name, out lblpath))
            {
                List<Transform> path = lblpath.ToList().ConvertAll(e => nodes.Find(o => o.name == e.Source).transform);
                path.Add(nodes.Find(o => o.name == lblpath.ToList()[lblpath.Count() - 1].Target).transform);
                fp.SetCP(path);
            }
        }

        // Update is called once per frame
        void Update()
        {

        }

        private void GenerateGraph()
        {

            qgraph = new BidirectionalGraph<string, Edge<string>>();
            foreach (GameObject n in nodes)
            {
                qgraph.AddVertex(n.name);
            }

            foreach (Vector2Int v2 in edges)
            {
                string u = nodes[v2.x].name;
                string v = nodes[v2.y].name;

                qgraph.AddEdge(new Edge<string>(u, v));
                qgraph.AddEdge(new Edge<string>(v, u));
            }
        }

        public void OnDrawGizmosSelected()
        {
            foreach (GameObject n in nodes)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(n.transform.position, .5f);
            }

            foreach (Vector2Int v2 in edges)
            {
                string u = nodes[v2.x].name;
                string v = nodes[v2.y].name;

                GameObject uobj = nodes.Find(o => o.name == u);
                GameObject vobj = nodes.Find(o => o.name == v);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(uobj.transform.position, vobj.transform.position);
            }
        }
    }
}
