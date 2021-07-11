using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RSUtils;
using System;
using MathNet.Numerics.LinearAlgebra;

using UnityEngine;

public class ScannerHandler : IHandler<(Collider, float, LayerMask), MapContainer>
{
    private BitmapHandler bitmap_h;
    private VisibilitymapHandler visibility_h;
    private DistanceTransformHandler distancetransform_h;
    private HeightmapHandler heightmap_h;

    public ScannerHandler()
    {
        bitmap_h = new BitmapHandler();
        visibility_h = new VisibilitymapHandler();
        distancetransform_h = new DistanceTransformHandler();
        heightmap_h = new HeightmapHandler();
    }

    public MapContainer Invoke((Collider, float, LayerMask) input)
    {
        var collider = input.Item1;
        var cell_size = input.Item2;
        var mask = input.Item3;

        MapContainer container;
        container.rsgrid = RSGrid.BuildFromCollider(collider, cell_size);
        container.bitmap = bitmap_h.Invoke((container.rsgrid, mask));
        container.visibility = visibility_h.Invoke((container.rsgrid, mask));
        container.distancetransform = distancetransform_h.Invoke((container.rsgrid, container.bitmap, (int)(5 / cell_size) + 1));
        container.heightmap = heightmap_h.Invoke((container.rsgrid, mask));

        return container;
    }
}