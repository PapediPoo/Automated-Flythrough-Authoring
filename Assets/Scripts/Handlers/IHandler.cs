using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RSUtils
{
    /// <summary>
    /// Interface for the step of a pipeline
    /// </summary>
    /// <typeparam name="I">The types of the input parameters of the pipeline step</typeparam>
    /// <typeparam name="O">The output type of the pipeline step </typeparam>
    public interface IHandler<I, O>
    {
        O Invoke(I input);
    }
}
