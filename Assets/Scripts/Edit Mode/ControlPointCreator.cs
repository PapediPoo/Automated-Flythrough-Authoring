using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Spawns a new prefab instance if the last spawned instance is moved away from the spawn position
/// </summary>
public class ControlPointCreator : MonoBehaviour
{
    [SerializeField]
    private GameObject control_point_prefab;    // Prefab to be instantiated
    private GameObject current_control_point;   // Reference to the previously instantiated prefab

    // Update is called once per frame
    void Update()
    {
        // if the reference to the previous instance is empty, instantiate new.
        if(current_control_point == null)
        {
            current_control_point = Instantiate(control_point_prefab, transform.position, Quaternion.identity, transform);
        }
        // if the previous instance is moved, remove reference to it.
        else if((current_control_point.transform.position - transform.position).magnitude > 0.05f)
        {
            current_control_point = null;
        }
    }
}
