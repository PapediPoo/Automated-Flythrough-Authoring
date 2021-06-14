using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.Diagnostics.Contracts;
using System;

public class BSpline : MonoBehaviour
{
    /*
     * This script moves the gameobject (@code.follower) along a B-spline defined by the control points (@code.points)
     * 
     * @field points: the control points of the spline
     * @field time: the current time position of the follower
     * @field speed: the scaling of the time field compared to real time. The B-spline is uniform, so this does not represent physically acurate speed.
     * @field iterations: set the number of refinement steps for approximating the move speed
     * @field controHeading: controls if the moved object looks in the direction that it is being moved towards.
     * @field follower: the reference to the object that moves along the spline
     * @field pointPositions: list of vector3s that mirror the positions of the control points. 
     * @field lastPositions: the result of the spline evaluation of the last frame. used for heading.
     */

    [Header("Spline Control Points")]
    public List<GameObject> points = new List<GameObject>();
    // public float[] knots = new float[] { 0f, 1f, 2f, 3f, 4f, 5f, 6f, 7f, 8f, 9f, 10f, 11f, 12f };

    [Header("Movement")]
    [Tooltip("The movement speed of the object in [u/s]")]
    public float speed = 0.2f;
    [Tooltip("Moving along a spline with constant speed is difficult. The actual speed is an approximation of the desired speed using iterative refinement. This parameter controls the number of refinement steps per frame.")]
    public int iterations = 5;
    [Tooltip("Should the object look in the direction it is moving?")]
    public bool controlHeading = true;
    private float stepsize;


    [Header("Object to move")]
    public GameObject follower;
    
    private float time = 0f;
    private List<Vector3> pointPositions;
    private Vector3 lastPosition;

    
    void Start()
    {
        pointPositions = RebuildPositionList(points);
        ResetFlythrough();

    }

    /* 
     * Updates the position and rotation of the follower object.
     * The viewing direction is approximated with the position difference between frames + an arbitrary weight for the current forward direction.
     * 
     * The spline "speed" along the spline is not constant, but it should be. There are 2 ways (that I can think of) to fix this
     * - Reparametrize the spline by calculating the arc-length of the spline and factoring it into the speed.
     * - Do some iterative refinement on the spline step size until it matches the desired real-world step size.
     * The second option is easier from an implementation/understandability point of view, so I implemented the second option.
     */
    void Update()
    {
        // Approximate step size
        for(int i = 0; i < iterations; i++)
        {
            if(time + stepsize >= pointPositions.Count)
            {
                ResetFlythrough();
            }

            float evalStepsize = (UnifCubicBSpline(pointPositions, time + stepsize) - lastPosition).magnitude;
            stepsize *= Time.deltaTime * speed / evalStepsize;
        }

        time += stepsize;

        // Update position & rotation
        if (time < points.Count)
        {
            Vector3 nextPosition = UnifCubicBSpline(pointPositions, time);

            follower.transform.position = nextPosition;
            if (controlHeading)
            {
                follower.transform.rotation = Quaternion.LookRotation(nextPosition - lastPosition + follower.transform.forward * .1f, Vector3.up);
            }

            lastPosition = nextPosition;
        }
        else
        {
            ResetFlythrough();

        }

    }

    private void ResetFlythrough()
    {
        time = 0f;
        stepsize = speed * Time.deltaTime;
        lastPosition = UnifCubicBSpline(pointPositions, time);
    }

    /*
     * Given a list of gameobjects, it generates a list of vector3s from their positions
     */
    private List<Vector3> RebuildPositionList(List<GameObject>objects) {
        List<Vector3> result = new List<Vector3>();
        foreach (GameObject p in objects)
        {
            result.Add(p.transform.position);
        }

        return result;
    }

    /*
     * Given a non-null, non-empty list of positions and a time t within the position-span, returns the value of
     * the open uniform cubic B-spline defined by the control points (@code.positions) at (@code.time)
     * Although the method is generic, the type of the positions needs to form a vector space
     * 
     * @ param positions: The list of B-spline control points
     * @ param time: The time at which the spline should be evaluated
     * @ returm: the value of spline at (@code.time)
     */
    private T UnifCubicBSpline<T>(List<T> positions, float time)
    {
        // Preconditions
        Debug.Assert(positions != null);
        Debug.Assert(positions.Count > 0);
        Debug.Assert(time >= 0f && time < positions.Count);

        if (time < 0f) return positions[0];
        if (time >= positions.Count) return positions[positions.Count - 1];

        int degree = 3;
        // B-Splines never reach the first & last control points naturally, but It would be nice if they did. 
        // Adding multiple copies of the first & last CP fixes this.
        dynamic P = new List<T>(positions);  // TODO: don't use dynamic keyword as it introduces issues at runtime.
        for(int i = 0; i < degree; i++)
        {
            P.Insert(0, P[0]);
            P.Add(P[P.Count-1]);
        }

        Matrix4x4 M = new float4x4(
            new float4(-1, 3, -3, 1),
            new float4(3, -6, 3, 0),
            new float4(-3, 0, 3, 0),
            new float4(1, 4, 1, 0)
            );

        int index = (int)time + degree;
        float4 timeV = new float4(Mathf.Pow(time % 1, degree), Mathf.Pow(time % 1, degree-1), time % 1, 1f);
        float4 weights = (M * timeV) / 6f;

        return (P[index-1] * weights.x) + (P[index] * weights.y) + (P[index+1] * weights.z) + (P[index+2] * weights.w);
    }

    // I tried using non-uniform B-splines but a) the code is buggy and b) Uniform B-splines work just as good
    //private float CoxDeBoor(float[] knots, int i, int k, float x)
    //{
    //    if(k == 1)
    //    {
    //        if(knots[i] <= x && x < knots[i + 1])
    //        {
    //            return 1f;
    //        }
    //        else
    //        {
    //            return 0f;
    //        }
    //    }
    //    else
    //    {

    //            float w1;
    //            float w2;
    //            try { 
    //                w1 = (x - knots[i]) / (knots[i + k - 1] - knots[i]); 
    //            }
    //            catch(IndexOutOfRangeException e)
    //            {
    //                w1 = 0f;
    //            }

    //            try
    //            {
    //                w2 = (x - knots[i + 1]) / (knots[i + k] - knots[i + 1]);

    //            }
    //            catch (IndexOutOfRangeException e)
    //            {
    //                w2 = 0f;
    //            }
    //            return (w1 * CoxDeBoor(knots, i, k - 1, x)) + ((1f - w2) * CoxDeBoor(knots, i + 1, k - 1, x));

    //    }
    //}

    //private Vector3 CubicBSpline(Vector3[] positions, float[] knots, float time)
    //{
    //    int i = (int)time;
    //    Vector3 result = Vector3.zero;
    //    for(int j = i - 1; j < i + 3; j++)
    //    {
    //        try
    //        {
    //            float weight = CoxDeBoor(knots, j, 3, time);
    //            print(weight);
    //            result += weight * positions[j];
    //        }
    //        catch (IndexOutOfRangeException e) { }
    //    }
    //    return result;
    //}

    /*
     * Draws a green line that represents the spline. Rebuilds the positions if needed.
     */
    private void OnDrawGizmosSelected()
    {
        float step = .2f;

        if (pointPositions == null || pointPositions.Count == 0)
        {
            pointPositions = RebuildPositionList(points);
        }

        Gizmos.color = Color.green;
        Vector3 last = UnifCubicBSpline(pointPositions, 0f);
        for (float t = step; t <= pointPositions.Count - 1; t += step)
        {
            Vector3 next = UnifCubicBSpline(pointPositions, t);
            Gizmos.DrawLine(last, next);
            last = next;
        }

    }

}
