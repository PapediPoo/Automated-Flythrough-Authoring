using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RuntimeHandlerController : MonoBehaviour
{
    Camera cam;
    public RuntimeHandle.RuntimeTransformHandle rth;
    public float max_dist = 100f;

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
