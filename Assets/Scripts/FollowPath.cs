using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.Interpolation;
using System.Linq;
using System;

public class FollowPath : MonoBehaviour
{
    public List<Transform> controlPoints = new List<Transform>();
    public double t;
    public double speed;
    private float stepSize;
    public GameObject target;
    public bool lookForward = true;
    private Vector3 lastPos;

    CubicSpline[] positionCP = new CubicSpline[3];
    Quaternion[] rotationCP;
    Quaternion rotationRef;
    // public float rotationSmooth = 2f;

    // Start is called before the first frame update
    void Start()
    {
        RefreshSplines();

        target.transform.rotation = controlPoints[0].transform.rotation;
        lastPos = controlPoints[0].transform.position;
    }

    public void SetCP(List<Transform> newControlPoints)
    {
        controlPoints = newControlPoints;
        stepSize = (float)speed * Time.deltaTime;

        RefreshSplines();
        transform.rotation = controlPoints[0].transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        /*
         *         // Approximate step size
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
         * 
         */

        //t += Time.deltaTime * speed;
        stepSize = (float)speed * Time.deltaTime;
        for(int i = 0; i < 10; i++)
        {
            Vector3 evalPos = new Vector3((float)positionCP[0].Interpolate(t+stepSize), (float)positionCP[1].Interpolate(t + stepSize), (float)positionCP[2].Interpolate(t + stepSize));
            float evalStepsize = (evalPos-lastPos).magnitude;
            //print(evalStepsize);
            stepSize *= Time.deltaTime * (float)speed / evalStepsize;
        }

        t += stepSize;

        if(t >= controlPoints.Count()-1)
        {
            t = 0f;
            transform.rotation = controlPoints[0].transform.rotation;
        }

        Vector3 pos = new Vector3((float)positionCP[0].Interpolate(t), (float)positionCP[1].Interpolate(t), (float)positionCP[2].Interpolate(t));
        Quaternion rot;

        if (lookForward)
        {
            rot = QuaternionUtil.SmoothDamp(target.transform.rotation, Quaternion.LookRotation(pos - lastPos, Vector3.up), ref rotationRef, (float)Time.deltaTime / (stepSize * 2f));
            lastPos = pos;
        }
        else
        {
            rot = PSlerp(rotationCP, (float)t);
        }

        target.transform.position = pos;
        target.transform.rotation = rot;
    }

    private void RefreshSplines()
    {
        Vector3[] xyz = (from o in controlPoints select o.transform.position).ToArray();
        double[] x = Array.ConvertAll(Enumerable.Range(0, controlPoints.Count).ToArray(), item => (double)item);
        //List<float> dists = Enumerable.Zip(xyz, xyz.Skip(1), (a, b) => (a - b).magnitude).ToList();
        //dists.Insert(0, 0f);
        //dists.Sort();
        //double[] x = Array.ConvertAll<float, double>(dists.ToArray(), item => (double)item);
        double[] yx = Array.ConvertAll((from o in controlPoints select o.transform.position.x).ToArray(), item => (double)item);
        double[] yy = Array.ConvertAll((from o in controlPoints select o.transform.position.y).ToArray(), item => (double)item);
        double[] yz = Array.ConvertAll((from o in controlPoints select o.transform.position.z).ToArray(), item => (double)item);

        rotationCP = (from o in controlPoints select o.transform.rotation).ToArray();

        //positionCP[0] = CubicSpline.InterpolateNaturalSorted(x, yx);
        //positionCP[1] = CubicSpline.InterpolateNaturalSorted(x, yy);
        //positionCP[2] = CubicSpline.InterpolateNaturalSorted(x, yz);

        positionCP[0] = CubicSpline.InterpolateAkima(x, yx);
        positionCP[1] = CubicSpline.InterpolateAkima(x, yy);
        positionCP[2] = CubicSpline.InterpolateAkima(x, yz);

        lastPos = new Vector3((float)positionCP[0].Interpolate(0), (float)positionCP[1].Interpolate(0), (float)positionCP[2].Interpolate(0));

    }

    Quaternion PSlerp(Quaternion[] controlPoints, float t)
    {
        UnityEngine.Assertions.Assert.IsNotNull(controlPoints);
        // UnityEngine.Assertions.Assert.IsTrue(0d <= t && t < controlPoints.Count());
        float ct = Mathf.Clamp(t, 0, controlPoints.Count());

        int i = (int)ct;

        Quaternion q1 = controlPoints[i];
        Quaternion q2 = controlPoints[i + 1];

        // return Quaternion.Slerp(q1, q2, ct - i);
        return QuaternionUtil.SmoothDamp(target.transform.rotation, q2, ref rotationRef, 1f / (float)speed);
    }

    public void OnDrawGizmosSelected()
    {
        if (controlPoints.Count >= 2)
        {
            if (rotationCP == null || positionCP[0] == null)
            {
                RefreshSplines();
            }
            Vector3 lastPos = new Vector3((float)positionCP[0].Interpolate(0f), (float)positionCP[1].Interpolate(0f), (float)positionCP[2].Interpolate(0f));
            Gizmos.color = Color.red;
            for (float i = 0f; i <= controlPoints.Count() - 1; i += .2f)
            {
                Vector3 pos = new Vector3((float)positionCP[0].Interpolate(i), (float)positionCP[1].Interpolate(i), (float)positionCP[2].Interpolate(i));
                Gizmos.DrawLine(lastPos, pos);
                lastPos = pos;
            }

        }
    }
}
