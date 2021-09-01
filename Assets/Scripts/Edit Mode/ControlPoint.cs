using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlPoint : MonoBehaviour
{
    public void AddToCPs()
    {
        var cpd = FindObjectOfType<ControlPointDestroyer>();
        var fgd = FindObjectOfType<FlythroughGizmoDrawer>();
        var fg = FindObjectOfType<FlythroughGenerator>();

        if (fgd != null)
        {
            if ((cpd.transform.position - transform.position).magnitude < 0.05f)
            {
                fgd.RemoveCP(gameObject);
                // Destroy(gameObject);

            }
            else
            {
                fgd.AddCP(gameObject);
                transform.localScale = Vector3.one;
            }

            fgd.CommitCPChanges();
        }
    }
}
