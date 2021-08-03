using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlPointCreator : MonoBehaviour
{
    [SerializeField]
    private GameObject control_point_prefab;
    private GameObject current_control_point;
    [SerializeField]
    private FlythroughGizmoDrawer fgd;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(current_control_point == null)
        {
            current_control_point = Instantiate(control_point_prefab, transform.position, Quaternion.identity, transform);
            //current_control_point.transform.SetParent(fgd.transform, true);
        }
        else if((current_control_point.transform.position - transform.position).magnitude > 0.05f)
        {
            // fgd.AddCP(current_control_point);
            current_control_point = null;
        }
    }
}
