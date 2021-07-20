using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Controls a Runtime Handler to only show for the child object that is closest to the mouse.
/// 
/// Author: Robin Schmidiger
/// Date: July 2021
/// Version: 0.1
/// </summary>
public class RuntimeHandlerController : MonoBehaviour
{
    Camera cam;
    public RuntimeHandle.RuntimeTransformHandle rth;    // The runtime transform handler to be updated
    public float max_dist = 100f;   // The maximum distance for an object to be to be considered by the script (in pixels)

    // Start is called before the first frame update
    void Start()
    {
        cam = Camera.main;
    }

    // Update is called once per frame
    void Update()
    {
        float min_dist = Mathf.Infinity;
        Transform final_selection = null;

        // Iterate over all child objects, find closest one, assign the runtime transform handler to it.
        foreach(Transform cp in transform)
        {
            var d = (Input.mousePosition - cam.WorldToScreenPoint(cp.position)).magnitude;
            if (d < min_dist && d < max_dist)
            {
                min_dist = d;
                final_selection = cp;
            }
        }
        if (final_selection != null)
        {
            rth.enabled = true;
            rth.target = final_selection;
        }
        else
        {
            rth.enabled = false;
        }

    }
}
