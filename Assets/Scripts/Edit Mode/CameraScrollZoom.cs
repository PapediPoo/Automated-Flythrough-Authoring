using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;

/// <summary>
/// Changes the zoom/distance of a cinemachine freelook camera rig with scrollwheel input
/// </summary>
[RequireComponent(typeof(CinemachineFreeLook))]
public class CameraScrollZoom : MonoBehaviour
{
    public float min_dist = 10;     // minimal distance between cam and its target object
    public float max_dist = 20;     // maximal distance between cam and its target object
    public float zoom_speed = 5;    // determines how sensitive the scrollwheel should be

    private float current_dist;     // stores the current distance of the camera to its target object
    private CinemachineFreeLook cfl;// references the cinemachine camera rig

    // Start is called before the first frame update
    void Start()
    {
        current_dist = max_dist;                        // initialize cam at maximum distance
        cfl = GetComponent<CinemachineFreeLook>();      // get reference to cinemachine camera rig
    }

    // Update is called once per frame
    void Update()
    {

        // change the camera distance by the scrollwheel input
        current_dist = Mathf.Clamp(current_dist - (Input.mouseScrollDelta.y * Time.deltaTime * zoom_speed), min_dist, max_dist);
        var sqrt = Mathf.Sqrt(.5f * Mathf.Pow(current_dist, 2));

        // the freelook cam is set up using three orbits at different heights that are being interpolated between. all of them have to be changed to adjust the overall camera distance
        cfl.m_Orbits[0] = new CinemachineFreeLook.Orbit() { m_Height = current_dist, m_Radius = 1f };
        cfl.m_Orbits[1] = new CinemachineFreeLook.Orbit() { m_Height = sqrt, m_Radius = sqrt };
        cfl.m_Orbits[2] = new CinemachineFreeLook.Orbit() { m_Height = 1f, m_Radius = current_dist };
    }
}
