using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.Interpolation;
using System.Linq;
using System;

/// <summary>
/// Lets the target object follow the specified trajectory with constant speed.a
/// 
/// Author: Robin Schmidiger
/// Date: April 2021
/// Version: 0.8
/// </summary>
public class FollowPath : MonoBehaviour
{
    public List<Vector3> controlPoints = new List<Vector3>();
    public double t;
    public double speed;
    private float stepSize;
    public GameObject target;
    public bool lookForward = true;
    private Vector3 last_pos;
    public float max_rot_speed = 180f;
    private bool ready;

    CubicSpline[] positionCP = new CubicSpline[3];
    Quaternion[] rotationCP;
    Quaternion rotationRef;
    // public float rotationSmooth = 2f;

    // Called once upon program start
    private void Start()
    {
        ready = false;
    }

    /// <summary>
    /// Resets the trajectory to its target position.
    /// Regenerates the spline.
    /// </summary>
    public void Restart()
    {
        if(controlPoints.Count < 5)
        {
            return;
        }

        RefreshSplines();

        target.transform.rotation = Quaternion.LookRotation(controlPoints[1] - controlPoints.First(), Vector3.up);
        last_pos = controlPoints.First();
        t = 0f;
        ready = true;
    }

    /// <summary>
    /// Updates the contorl point list and refreshes the splines
    /// </summary>
    /// <param name="newControlPoints"></param>
    public void SetCP(List<Vector3> newControlPoints)
    {
        controlPoints = newControlPoints;
        stepSize = (float)speed * Time.deltaTime;

        RefreshSplines();
        transform.rotation = Quaternion.LookRotation(controlPoints[1] - controlPoints.First(), Vector3.up);
    }

    /// <summary>
    /// updates the time variable and the position of the target object every frame.
    /// The resulting animation is being looped.
    /// </summary>
    void Update()
    {
        if (!ready)
        {
            return;
        }

        // Do optimization on the stepsize to make the move speed approximatively constant
        stepSize = (float)speed * Time.deltaTime;   // Initial guess for optimization
        var NUM_ITERATIONS = 10;
        for(int i = 0; i < NUM_ITERATIONS; i++)
        {
            Vector3 evalPos = new Vector3((float)positionCP[0].Interpolate(t+stepSize), (float)positionCP[1].Interpolate(t + stepSize), (float)positionCP[2].Interpolate(t + stepSize));
            float evalStepsize = (evalPos-last_pos).magnitude;
            stepSize *= Time.deltaTime * (float)speed / evalStepsize;
        }

        // apply optimized step size
        t += stepSize;

        // Reset loop if end is reached
        if(t >= controlPoints.Count()-1)
        {
            t = 0f;
            transform.rotation = Quaternion.LookRotation(controlPoints[1] - controlPoints.First(), Vector3.up);
        }

        // get next position by interpolation
        Vector3 pos = new Vector3((float)positionCP[0].Interpolate(t), (float)positionCP[1].Interpolate(t), (float)positionCP[2].Interpolate(t));
        Quaternion rot;

        // calculate viewing direction
        if (lookForward)
        {
            var target_rot = QuaternionUtil.SmoothDamp(target.transform.rotation, Quaternion.LookRotation(pos - last_pos, Vector3.up), ref rotationRef, (float)Time.deltaTime / (stepSize));

            rot = Quaternion.RotateTowards(target.transform.rotation, target_rot, Time.deltaTime * max_rot_speed);
            last_pos = pos;
        }
        else
        {
            rot = PSlerp(rotationCP, (float)t);
        }

        // apply positin & rotation to target transform
        target.transform.position = pos;
        target.transform.rotation = rot;
    }

    /// <summary>
    /// Regenerates the splines
    /// Useful if the control points have changed.
    /// </summary>
    private void RefreshSplines()
    {
        if(controlPoints.Count < 5)
        {
            return;
        }

        Vector3[] xyz = controlPoints.ToArray();
        double[] x = Array.ConvertAll(Enumerable.Range(0, controlPoints.Count).ToArray(), item => (double)item);

        // Split up interpolation value term into xyz axis
        double[] yx = Array.ConvertAll((from o in controlPoints select o.x).ToArray(), item => (double)item);
        double[] yy = Array.ConvertAll((from o in controlPoints select o.y).ToArray(), item => (double)item);
        double[] yz = Array.ConvertAll((from o in controlPoints select o.z).ToArray(), item => (double)item);

        // Generate splines for each separate axis
        positionCP[0] = CubicSpline.InterpolateAkima(x, yx);
        positionCP[1] = CubicSpline.InterpolateAkima(x, yy);
        positionCP[2] = CubicSpline.InterpolateAkima(x, yz);

        // update lastpos (to avoid janky movement)
        last_pos = new Vector3((float)positionCP[0].Interpolate(0), (float)positionCP[1].Interpolate(0), (float)positionCP[2].Interpolate(0));

    }

    /// <summary>
    /// Implements smoothed linear interpolation for quaternions
    /// </summary>
    /// <param name="controlPoints">the control point rotations used for slerping</param>
    /// <param name="t">the evaluation point</param>
    /// <returns>the interpolated rotation</returns>
    Quaternion PSlerp(Quaternion[] controlPoints, float t)
    {
        UnityEngine.Assertions.Assert.IsNotNull(controlPoints);
        // UnityEngine.Assertions.Assert.IsTrue(0d <= t && t < controlPoints.Count());
        float ct = Mathf.Clamp(t, 0, controlPoints.Count());

        int i = (int)ct;

        Quaternion q1 = controlPoints[i];
        Quaternion q2 = controlPoints[i + 1];

        return QuaternionUtil.SmoothDamp(target.transform.rotation, q2, ref rotationRef, 1f / (float)speed);
    }
}
