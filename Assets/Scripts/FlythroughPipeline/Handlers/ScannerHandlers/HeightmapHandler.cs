using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RSUtils;
using System;
using MathNet.Numerics.LinearAlgebra;

public class HeightmapHandler : IHandler<(RSGrid, LayerMask), float[,,]>
{
    public float[,,] Invoke((RSGrid, LayerMask) input)
    {
        LayerMask mask = input.Item2;
        RSGrid rsgrid = input.Item1;
        float MAX_DIST = 10f;

        RaycastHit rch;
        Func<Vector<double>, float> f = x =>
        {
            Vector3 p = Utils.VToV3(x);
            if (Physics.Raycast(p, Vector3.down, out rch, MAX_DIST, mask.value))
            {
                // Debug.Log(rch.distance);
                return rch.distance;
            }
            else
            {
                return MAX_DIST;
            }
        };

        return rsgrid.ForAll(f);
    }
}

