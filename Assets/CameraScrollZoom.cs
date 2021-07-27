using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;

[RequireComponent(typeof(CinemachineFreeLook))]
public class CameraScrollZoom : MonoBehaviour
{
    public float min_dist = 10;
    public float max_dist = 20;
    public float zoom_speed = 5;
    private float current_dist;
    private CinemachineFreeLook cfl;

    // Start is called before the first frame update
    void Start()
    {
        current_dist = max_dist;
        cfl = GetComponent<CinemachineFreeLook>();
    }

    // Update is called once per frame
    void Update()
    {
        current_dist = Mathf.Clamp(current_dist - (Input.mouseScrollDelta.y * Time.deltaTime * zoom_speed), min_dist, max_dist);
        var sqrt = Mathf.Sqrt(.5f * Mathf.Pow(current_dist, 2));
        cfl.m_Orbits[0] = new CinemachineFreeLook.Orbit() { m_Height = current_dist, m_Radius = 1f };
        cfl.m_Orbits[1] = new CinemachineFreeLook.Orbit() { m_Height = sqrt, m_Radius = sqrt };
        cfl.m_Orbits[2] = new CinemachineFreeLook.Orbit() { m_Height = 1f, m_Radius = current_dist };
        //todo commenting 
    }
}
