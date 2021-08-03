using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum ProgramState
{
    EditMode,
    PlayMode
}

/// <summary>
/// Controls the overall state of the program.
/// Depending on the current state of the program it enables/disables specified gameobjects.
/// </summary>
public class ProgramMaster : MonoBehaviour
{
    private ProgramState current_state;

    [SerializeField]
    private GameObject editmode_container;
    [SerializeField]
    private GameObject playmode_container;

    // Start is called before the first frame update
    void Start()
    {
        // Disable all containers
        editmode_container.SetActive(false);
        playmode_container.SetActive(false);

         // Set the default state to play mode.
        ChangeState(ProgramState.EditMode);
    }

    /// <summary>
    /// changes the currently active state of the program master
    /// </summary>
    /// <param name="next_state">The state to be switched to</param>
    public void ChangeState(string next_state)
    {
        switch (next_state)
        {
            case "editmode":
                ChangeState(ProgramState.EditMode);
                break;
            case "playmode":
                ChangeState(ProgramState.PlayMode);
                break;
        }
    }

    /// <summary>
    /// changes the currently active state of the program master
    /// </summary>
    /// <param name="next_state">The state to be switched to</param>
    public void ChangeState(ProgramState next_state)
    {
        print("changed state to: " + next_state.ToString());

        // Disable the container for the last state
        switch (current_state)
        {
            case ProgramState.EditMode:
                editmode_container.SetActive(false);
                break;
            case ProgramState.PlayMode:
                playmode_container.SetActive(false);
                break;
        }

        // update the current state
        current_state = next_state;

        // enable the container for the next state
        switch (current_state)
        {
            case ProgramState.EditMode:
                editmode_container.SetActive(true);
                break;
            case ProgramState.PlayMode:
                playmode_container.SetActive(true);
                break;
        }
    }
}
