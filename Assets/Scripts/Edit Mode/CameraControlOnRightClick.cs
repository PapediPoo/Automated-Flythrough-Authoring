using UnityEngine;
using Cinemachine;

/// <summary>
/// Utility script that extends a CinemachineFreeLook camera to only respond to input if the right mouse button is being pressed.
/// </summary>
/// 
[RequireComponent(typeof(CinemachineFreeLook))]
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

    /// <summary>
    /// enable and disable the camera movement depending on if the right mouse button is pressed/released.
    /// Check the input every frame.
    /// </summary>
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

    /// <summary>
    /// disables the camera movement of the cinemachine free look camera
    /// </summary>
    private void DisableFreeLook()
    {
        freelook.m_XAxis.m_InputAxisName = "";
        freelook.m_YAxis.m_InputAxisName = "";

        freelook.m_XAxis.m_InputAxisValue = 0f;
        freelook.m_YAxis.m_InputAxisValue = 0f;
    }

    /// <summary>
    /// (re-) enables the camera movement of the cinemachine free look camera
    /// </summary>
    private void EnableFreeLook()
    {
        freelook.m_XAxis.m_InputAxisName = init_Xaxis_name;
        freelook.m_YAxis.m_InputAxisName = init_Yaxis_name;
    }
}
