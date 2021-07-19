using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum ProgramState
{
    EditMode,
    PlayMode
}

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
        editmode_container.SetActive(false);
        playmode_container.SetActive(false);

        ChangeState(ProgramState.EditMode);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

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

    public void ChangeState(ProgramState next_state)
    {
        print("changed state to: " + next_state.ToString());

        switch (current_state)
        {
            case ProgramState.EditMode:
                editmode_container.SetActive(false);
                break;
            case ProgramState.PlayMode:
                playmode_container.SetActive(false);
                break;
        }

        current_state = next_state;

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
