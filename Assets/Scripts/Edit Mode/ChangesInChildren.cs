using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Triggers a unityEvent if the position of its children changes
/// </summary>
public class ChangesInChildren : MonoBehaviour
{
    public UnityEvent change_event;     // The event to be triggered
    public float check_every = 1.5f;    // frequency of change detection
    private float counter;              // time until next refresh
    public bool disable_on_leftclick = true;    // disables updates if left mouse is clicked (left click = move object)
    private float hash;                 // hash that encodes the previous positional configuration of the children

    // Start is called before the first frame update
    void Start()
    {
        counter = 0f;
        hash = 0f;
    }

    // Update is called once per frame
    void Update()
    {
        counter += Time.deltaTime;
        if(counter >= check_every && !(disable_on_leftclick && Input.GetMouseButton(0)))    // update if counter runs out and left mouse not clicked
        {
            counter = 0;
            float new_hash = 0f;
            foreach(Transform t in transform)
            {
                new_hash += t.position.magnitude;   // compute new haw from child positions
            }

            if(hash != new_hash)                    // compare new and old hash
            {                                       // if they are different
                hash = new_hash;                        // overwrite old hash
                change_event.Invoke();                  // trigger unityEvent
            }
        }
    }
}
