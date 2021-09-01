using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetToThisPosition : MonoBehaviour
{
    public void Set(Transform other)
    {
        other.transform.position = transform.position;
        other.transform.rotation = transform.rotation;
    }
}
