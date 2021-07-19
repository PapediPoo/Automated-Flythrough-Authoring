using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Cinemachine;

public class CameraControlOnRightClick : MonoBehaviour
{
    public string init_Xaxis_name;
    public string init_Yaxis_name;
    private CinemachineFreeLook freelook;

    // Start is called before the first frame update
    void Start()
    {
        freelook = GetComponent<CinemachineFreeLook>();

        DisableFreeLook();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonUp(1))
        {
            DisableFreeLook();

        }

        if (Input.GetMouseButtonDown(1))
        {
            EnableFreeLook();

        }
    }

    private void DisableFreeLook()
    {
        freelook.m_XAxis.m_InputAxisName = "";
        freelook.m_YAxis.m_InputAxisName = "";
    }

    private void EnableFreeLook()
    {
        freelook.m_XAxis.m_InputAxisName = init_Xaxis_name;
        freelook.m_YAxis.m_InputAxisName = init_Yaxis_name;
    }
}
