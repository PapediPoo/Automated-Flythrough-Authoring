using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class ChangesInChildren : MonoBehaviour
{
    public UnityEvent change_event;
    public float check_every = 1.5f;
    float counter;
    public bool disable_on_leftclick = true;
    float hash;

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
        if(counter >= check_every && !(disable_on_leftclick && Input.GetMouseButton(0)))
        {
            counter = 0;
            float new_hash = 0f;
            foreach(Transform t in transform)
            {
                new_hash += t.position.magnitude;
            }

            if(hash != new_hash)
            {
                hash = new_hash;
                change_event.Invoke();
            }
        }
    }
}
