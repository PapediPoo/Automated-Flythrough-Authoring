using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using System;
using MathNet.Numerics.LinearAlgebra;

/// <summary>
/// Handler that generates a bitmap inside the grid
/// 
/// Version: 0.4
/// Author: Robin Schmidiger
/// Date: May 2021
/// </summary>
public class BitmapHandler : IHandler<(RSGrid, LayerMask), bool[,,]>
{
    public bool[,,] Invoke((RSGrid, LayerMask) input)
    {
        RSGrid rsgrid = input.Item1;
        LayerMask mask = input.Item2;
        Func<Vector<double>, bool> f = x =>
        {
            Vector3 pos = RSUtils.Utils.VToV3(x);
            return Physics.CheckSphere(pos, (float)rsgrid.GetCellSize() / 2f, mask.value);
        };

        return rsgrid.ForAll(f);
    }
}
