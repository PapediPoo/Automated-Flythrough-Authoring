using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface ISimpleWorker<IN, OUT>
{
    public void Start(IN input);
    public bool IsDone();
    public void Work();
    public OUT GetResult();
}
