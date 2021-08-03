using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestScript : MonoBehaviour
{

    public float MAX_DISTANCE;
    public Vector3 gradient;
    public float distance;

    public TrajectorySettings settings;
    private TrajectoryOptimizationHandler.DistanceAtPointContainer container;


    // Start is called before the first frame update
    void Start()
    {
        //container = TrajectoryOptimizationHandler.DistanceAtPointAlloc(0.5f, 25f, 10);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        //var valgrad = TrajectoryOptimizationHandler.DistanceAtPoint(transform.position, container, settings);

        //gradient = valgrad.Item2;
        //distance = valgrad.Item1;

        //if(Physics.SphereCast(transform.position, MAX_DISTANCE, dir, out rch, dist))
        //{
        //    gradient = rch.point - transform.position;
        //    print("hit");
        //}
        //else
        //{
        //    gradient = Vector3.zero;
        //}
    }

    private void OnDrawGizmosSelected()
    {
        //Gizmos.DrawLine(transform.position, transform.position + (distance * gradient));
    }
}
