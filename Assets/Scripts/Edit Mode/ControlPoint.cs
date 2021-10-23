using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// control point script that handles deletion of control points and updates to the gizmos
/// </summary>
public class ControlPoint : MonoBehaviour
{
    public void AddToCPs()
    {
        var cpd = FindObjectOfType<ControlPointDestroyer>();
        var fgd = FindObjectOfType<FlythroughGizmoDrawer>();

        if (fgd != null)
        {
            // destroy the control point if it is placed in the trash can
            if ((cpd.transform.position - transform.position).magnitude < 0.05f)
            {
                fgd.RemoveCP(gameObject);

            }
            else
            {
                // add the control point to the gizmos
                fgd.AddCP(gameObject);
                transform.localScale = Vector3.one;
            }

            // force the gizmos to update
            fgd.CommitCPChanges();
        }
    }
}
