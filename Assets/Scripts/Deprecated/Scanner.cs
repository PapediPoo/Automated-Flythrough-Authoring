using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Scanner
{
    public static bool[,,] BuildBitmap(Collider col, float size, LayerMask mask)
    {
        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        //bitmap = new Mat(sizes, MatType.CV_32FC1, 0f);
        bool[,,] bitmap = new bool[sizes[0], sizes[1], sizes[2]];

        if (col != null && size > 0f)
        {
            for (int x = 0; x < sizes[0]; x++)
            {
                for (int y = 0; y < sizes[1]; y++)
                {
                    for (int z = 0; z < sizes[2]; z++)
                    {
                        Vector3 pos = col.bounds.center - col.bounds.extents + (size * new Vector3(x, y, z));
                        if (Physics.CheckSphere(pos, size / 2f, mask.value))
                        {
                            bitmap.SetValue(true, x, y, z);
                        }
                        else
                        {
                            bitmap.SetValue(false, x, y, z);

                        }
                    }
                }
            }


            Debug.Log("bitmap done");
        }
        return bitmap;
    }
    public static float[,,] BuildVisibilityMap(Collider col, float size)
    {
        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        // visibility = new Mat(sizes, MatType.CV_32FC1, 1f);
        float[,,] visibility = new float[sizes[0], sizes[1], sizes[2]];
        for (int x1 = 0; x1 < sizes[0]; x1++)
        {
            for (int y1 = 0; y1 < sizes[1]; y1++)
            {
                for (int z1 = 0; z1 < sizes[2]; z1++)
                {
                    for (int x2 = 0; x2 < sizes[0]; x2++)
                    {
                        for (int y2 = 0; y2 < sizes[1]; y2++)
                        {
                            for (int z2 = 0; z2 < sizes[2]; z2++)
                            {
                                Vector3 pos1 = col.bounds.center - col.bounds.extents + (size * new Vector3(x1, y1, z1));
                                Vector3 pos2 = col.bounds.center - col.bounds.extents + (size * new Vector3(x2, y2, z2));
                                if (!Physics.Raycast(pos1, pos2 - pos1, (pos2 - pos1).magnitude))
                                {
                                    // float v = visibility.At<float>(x1, y1, z1);
                                    // visibility.Set<float>(x1, y1, z1, v + 1);
                                    visibility[x1, y1, z1]++;
                                }
                            }
                        }
                    }
                }
            }
        }

        float minVal = visibility.Cast<float>().Min();
        float maxVal = visibility.Cast<float>().Max();

        for (int x = 0; x < sizes[0]; x++)
        {
            for (int y = 0; y < sizes[1]; y++)
            {
                for (int z = 0; z < sizes[2]; z++)
                {
                    float v = ((float)visibility.GetValue(x, y, z) - minVal) / (maxVal - minVal);
                    visibility.SetValue(v, x, y, z);
                }
            }
        }

        Debug.Log("visibility done");
        return visibility;
    }

    public static float[,,] BuildDistanceTransform(Collider col, float size, bool[,,] bitmap)
    {

        int[] sizes = new int[] { (int)(col.bounds.size.x / size), (int)(col.bounds.size.y / size), (int)(col.bounds.size.z / size) };

        float[,,] distanceTransform = new float[sizes[0], sizes[1], sizes[2]];



        for (int x = 0; x < sizes[0]; x++)
        {
            for (int y = 0; y < sizes[1]; y++)
            {
                for (int z = 0; z < sizes[2]; z++)
                {
                    if ((bool)bitmap.GetValue(x, y, z))
                    {
                        distanceTransform.SetValue(0f, x, y, z);
                    }
                    else
                    {
                        distanceTransform.SetValue(Mathf.Infinity, x, y, z);
                    }
                }
            }
        }

        for (int iter = 0; iter < 10; iter++)
        {
            for (int x = 0; x < sizes[0]; x++)
            {
                for (int y = 0; y < sizes[1]; y++)
                {
                    for (int z = 0; z < sizes[2]; z++)
                    {
                        float locmin = Mathf.Infinity;
                        for (int xo = -1; xo <= 1; xo++)
                        {

                            for (int zo = -1; zo <= 1; zo++)
                            {
                                try
                                {
                                    Vector3 o = new Vector3(xo, 0, zo) * size;
                                    locmin = Mathf.Min(locmin, (float)distanceTransform.GetValue(x + xo, y, z + zo) + o.magnitude);
                                }
                                catch (System.IndexOutOfRangeException)
                                {

                                }
                            }
                        }
                        distanceTransform.SetValue(locmin, x, y, z);
                    }
                }
            }
        }


        Debug.Log("distance transform done");
        return distanceTransform;
    }
}
