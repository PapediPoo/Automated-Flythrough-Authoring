using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using RuntimeHandle;
using System;
using TMPro;
using UnityEngine.UI;

[ExecuteAlways]
public class FlythroughGizmoDrawer : MonoBehaviour
{
    private enum FlythroughGizmoMode
    {
        none,
        bitmap,
        distance_transform,
        visibility,
        control_points,
        tour,
        trajectory
    }

    [SerializeField]
    private FlythroughGizmoMode mode = FlythroughGizmoMode.trajectory;

    [SerializeField]
    private FlythroughGenerator fg;

    [SerializeField]
    private Material gizmo_material;

    //[SerializeField]
    //private List<Material> crosssection_materials = new List<Material>();
    //[SerializeField]
    //private float crosssection_offset = 0.5f;

    private GameObject[,] map_tiles;
    private List<Renderer> control_points;
    private LineRenderer line_renderer;
    private GameObject rth;
    private GameObject rhs;



    public float render_height = 1f;

    public void ChangeRenderHeight(Slider s)
    {
        render_height = s.value;
    }

    public void ChangeMode(TMP_Dropdown dd)
    {
        var new_mode = dd.value;
        switch (new_mode)
        {
            case 0:
                mode = FlythroughGizmoMode.none;
                break;
            case 1:
                mode = FlythroughGizmoMode.trajectory;
                break;
            case 2:
                mode = FlythroughGizmoMode.tour;
                break;
            case 3:
                mode = FlythroughGizmoMode.control_points;
                break;
            case 4:
                mode = FlythroughGizmoMode.bitmap;
                break;
            case 5:
                mode = FlythroughGizmoMode.visibility;
                break;
            case 6:
                mode = FlythroughGizmoMode.distance_transform;
                break;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        mode = FlythroughGizmoMode.trajectory;
        fg = FindObjectOfType<FlythroughGenerator>();

        line_renderer = GetComponentInChildren<LineRenderer>();
        control_points = new List<Renderer>();
        rth = FindObjectOfType<RuntimeTransformHandle>().gameObject;
        rhs = FindObjectOfType<Slider>().gameObject;

        //rhs.SetActive(false);
        // InitializeTiles();
    }

    void InitializeTiles()
    {
        if(control_points.Count == 0)
        {
            foreach(var obj in GameObject.FindGameObjectsWithTag("Control Point"))
            {
                var r = obj.GetComponent<Renderer>();
                r.enabled = false;
                control_points.Add(r);

            }
        }

        if (fg != null && fg.map_container.rsgrid != null)
        {
            var rsgrid = fg.map_container.rsgrid;
            var xz = ((int)rsgrid.lengths.At(0), (int)rsgrid.lengths.At(2));

            map_tiles = new GameObject[xz.Item1, xz.Item2];
            for (int i = 0; i < xz.Item1; i++)
            {
                for (int j = 0; j < xz.Item2; j++)
                {
                    var obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    map_tiles[i, j] = obj;

                    var renderer = obj.GetComponent<Renderer>();
                    renderer.material = gizmo_material;

                    obj.transform.localScale = new Vector3((float)rsgrid.cell_size, 0.1f, (float)rsgrid.cell_size);
                    obj.transform.parent = transform;

                    Destroy(obj.GetComponent<BoxCollider>());

                    obj.SetActive(false);
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        //foreach(Material m in crosssection_materials)
        //{
        //    print(render_height + crosssection_offset);
        //    m.SetFloat("Vector1_24f0fe65288441ad9422b8f2b4765bdb", render_height + crosssection_offset);
        //}

        if (fg != null)
        {
            if(fg.map_container.rsgrid != null && map_tiles == null)
            {
                InitializeTiles();
            }

            if (fg.map_container.rsgrid != null && map_tiles != null)
            {
                switch (mode)
                {
                    case FlythroughGizmoMode.bitmap:
                        DrawBitmap();
                        break;
                    case FlythroughGizmoMode.visibility:
                        DrawVisibility();
                        break;
                    case FlythroughGizmoMode.distance_transform:
                        DrawDistanceTransform();
                        break;
                    case FlythroughGizmoMode.trajectory:
                        DrawTrajectory();
                        break;
                    case FlythroughGizmoMode.tour:
                        DrawTour();
                        break;
                    case FlythroughGizmoMode.control_points:
                        DrawControlPoints();
                        break;
                    default:
                        DrawNone();
                        break;
                }

            }
        }
    }

    private T GetAt<T>(T[,,] map, Vector<float> index)
    {
        return map[(int)index.At(0), (int)index.At(1), (int)index.At(2)];
    }

    private void DrawBitmap()
    {
        var bitmap = fg.map_container.bitmap;
        line_renderer.enabled = false;
        foreach(var r in control_points)
        {
            r.enabled = false;
        }
        rth.SetActive(false);
        rhs.SetActive(true);

        if(bitmap == null || map_tiles == null)
        {
            return;
        }

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height) {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.position = pos;
                tile.GetComponent<Renderer>().material.color = Color.white;
                if (GetAt<bool>(bitmap, i))
                {
                    tile.SetActive(true);
                }
                else
                {
                    tile.SetActive(false);
                }
            }
            return 0f;
        });

        fg.map_container.rsgrid.ForAllIndexed(f, true);
    
    }

    private void DrawVisibility()
    {
        var visibility = fg.map_container.visibility;
        var bitmap = fg.map_container.bitmap;
        line_renderer.enabled = false;
        foreach (var r in control_points)
        {
            r.enabled = false;
        }
        rth.SetActive(false);
        rhs.SetActive(true);


        if (visibility == null || map_tiles == null)
        {
            return;
        }

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height)
            {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.position = pos;
                //tile.SetActive(true);
                if (GetAt<bool>(bitmap, i))
                {
                    tile.SetActive(false);
                }
                else
                {
                    tile.SetActive(true);
                }

                float GAIN = 5f;
                float v = GetAt(visibility, i) * GAIN;
                tile.GetComponent<Renderer>().material.color = new Color(1f - v, v, 0f);

            }
            return 0f;
        });

        fg.map_container.rsgrid.ForAllIndexed(f, true);

    }

    private void DrawDistanceTransform()
    {
        var distance_transform = fg.map_container.distancetransform;
        var bitmap = fg.map_container.bitmap;
        line_renderer.enabled = false;
        foreach (var r in control_points)
        {
            r.enabled = false;
        }
        rth.SetActive(false);
        rhs.SetActive(true);


        if (distance_transform == null || map_tiles == null)
        {
            return;
        }

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height)
            {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.position = pos;
                tile.SetActive(true);
                float GAIN = 3f;
                float v = Mathf.Clamp01(GetAt(distance_transform, i) / GAIN);
                tile.GetComponent<Renderer>().material.color = new Color(1f - v, v, 0f);

            }
            return 0f;
        });

        fg.map_container.rsgrid.ForAllIndexed(f, true);

    }

    private void DrawNone()
    {
        foreach (var t in map_tiles)
        {
            t.SetActive(false);
        }
        foreach (var r in control_points)
        {
            r.enabled = false;
        }
        line_renderer.enabled = false;
        rth.SetActive(false);
        rhs.SetActive(false);

    }

    private void DrawTour()
    {
        var tour = fg.trajectory_container.tour;

        foreach (var t in map_tiles)
        {
            t.SetActive(false);
        }
        foreach (var r in control_points)
        {
            r.enabled = true;
        }
        line_renderer.enabled = true;
        rth.SetActive(true);
        rhs.SetActive(false);


        if (tour == null || line_renderer == null)
        {
            return;
        }

        line_renderer.positionCount = tour.Count;
        for(int i = 0; i < tour.Count; i++)
        {
            line_renderer.SetPosition(i, RSUtils.Utils.VToV3(tour[i]));
        }
    }

    private void DrawControlPoints()
    {
        foreach (var t in map_tiles)
        {
            t.SetActive(false);
        }
        foreach (var r in control_points)
        {
            r.enabled = true;
        }
        line_renderer.enabled = false;
        rth.SetActive(true);
        rhs.SetActive(false);

    }

    private void DrawTrajectory()
    {
        var trajectory = fg.trajectory_container.trajectory;

        foreach(var t in map_tiles)
        {
            t.SetActive(false);
        }
        foreach (var r in control_points)
        {
            r.enabled = true;
        }
        rth.SetActive(true);
        rhs.SetActive(false);

        if (trajectory == null || line_renderer == null)
        {
            return;
        }

        Vector3 pos;

        line_renderer.enabled = true;
        line_renderer.positionCount = trajectory.Count / 3;
        for(int i = 0; i < trajectory.Count; i+=3)
        {
            pos = new Vector3((float)trajectory[i], (float)trajectory[i+1], (float)trajectory[i+2]);
            line_renderer.SetPosition(i / 3, pos);
        }
        
    }
}
