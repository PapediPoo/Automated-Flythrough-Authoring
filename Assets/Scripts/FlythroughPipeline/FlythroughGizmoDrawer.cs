using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using RuntimeHandle;
using System;
using TMPro;
using System.Linq;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit.UI;

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
    [SerializeField]
    private GameObject control_point_prefab;
    [SerializeField]
    public Vector3 control_point_init_size = Vector3.one;

    public Transform relative_to = null;

    private bool ready;
    private float checksum;

    private GameObject[,] map_tiles;
    private List<GameObject> control_points;
    private LineRenderer line_renderer;
    [SerializeField]
    private GameObject rhs;

    public float render_height = 1f;

    public List<Material> cross_section_materials = new List<Material>();

    public void ChangeRenderHeight(PinchSlider slider)
    {
        render_height = Mathf.Lerp(-3f, 5f, slider.SliderValue);
    }

    public void AddCP(GameObject cp)
    {
        if (!control_points.Contains(cp))
        {
            control_points.Add(cp);
            cp.transform.SetParent(relative_to);
        }
    }

    public void RemoveCP(GameObject cp)
    {
        if (control_points.Contains(cp))
        {
            control_points.Remove(cp);
            Destroy(cp);
        }
    }

    public void ChangeRenderHeight(Slider s)
    {
        render_height = s.value;
    }
    public void ChangeMode(InteractableToggleCollection itc)
    {
        switch (itc.CurrentIndex)
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
        ready = false;
        checksum = 0f;
        mode = FlythroughGizmoMode.trajectory;
        fg = FindObjectOfType<FlythroughGenerator>();

        line_renderer = GetComponentInChildren<LineRenderer>();
    }

    void Initialize()
    {
        var cps = fg.trajectory_container.control_points;
        var rsgrid = fg.map_container.rsgrid;
        var xz = ((int)rsgrid.lengths.At(0), (int)rsgrid.lengths.At(2));

        map_tiles = new GameObject[xz.Item1, xz.Item2];
        control_points = new List<GameObject>();

        for (int i = 0; i < xz.Item1; i++)
        {
            for (int j = 0; j < xz.Item2; j++)
            {
                var obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                map_tiles[i, j] = obj;

                var renderer = obj.GetComponent<Renderer>();
                renderer.material = gizmo_material;

                obj.transform.parent = relative_to;
                obj.transform.localScale = new Vector3((float)rsgrid.cell_size, 0.1f, (float)rsgrid.cell_size);

                Destroy(obj.GetComponent<BoxCollider>());

                obj.SetActive(false);
            }
        }

        foreach(var v in cps)
        {
            Vector3 v3 = RSUtils.Utils.VToV3(v);
            //var obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            var obj = Instantiate(control_point_prefab);
            control_points.Add(obj);

            //var renderer = obj.GetComponent<Renderer>();
            //renderer.material = gizmo_material;
            //renderer.material.color = Color.white;

            obj.transform.parent = relative_to;
            //obj.transform.localScale = Vector3.one * 0.5f;
            obj.transform.localPosition = v3;
            obj.transform.localScale = control_point_init_size;
        }
    }

    private void DrawMode(bool draw_tiles, bool draw_cps, bool draw_line)
    {
        foreach(GameObject t in map_tiles)
        {
            t.SetActive(draw_tiles);
        }
        foreach(var cp in control_points)
        {
            cp.SetActive(draw_cps);
        }
        line_renderer.enabled = draw_line;

        // if(rhs != null) rhs.SetActive(draw_tiles);
    }

    public void CommitCPChanges()
    {
        var sum = Enumerable.Aggregate(control_points, 0f, (x, y) => x + y.transform.localPosition.magnitude);
        if(sum != checksum)
        {
            while(control_points.Count > fg.trajectory_container.control_points.Count)
            {
                fg.trajectory_container.control_points.Add(Vector<double>.Build.Dense(3));
            }

            while(control_points.Count < fg.trajectory_container.control_points.Count)
            {
                fg.trajectory_container.control_points.RemoveAt(0);
            }

            checksum = sum;
            for(int i=0;i<control_points.Count;i++)
            {
                fg.trajectory_container.control_points[i] = RSUtils.Utils.V3ToV(control_points[i].transform.localPosition, fg.trajectory_container.control_points[i]);
            }
            fg.PlanTour();
        }
    }
    // Update is called once per frame
    void Update()
    {
        foreach(Material m in cross_section_materials)
        {
            m.SetFloat("Vector1_24f0fe65288441ad9422b8f2b4765bdb", Mathf.Lerp(13, 22, Mathf.InverseLerp(-3, 5, render_height) + 0.05f));
        }

        if (ready)
        {
            // CommitCPChanges();
            line_renderer.startWidth = transform.lossyScale.x * .1f;

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
        else if(fg != null && fg.trajectory_container.trajectory != null)
        {
            ready = true;
            Initialize();
        }
    }

    private T GetAt<T>(T[,,] map, Vector<float> index)
    {
        return map[(int)index.At(0), (int)index.At(1), (int)index.At(2)];
    }
    private void DrawBitmap()
    {
        var bitmap = fg.map_container.bitmap;
        DrawMode(true, false, false);

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height) {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.localPosition = pos;
                tile.GetComponent<Renderer>().material.color = Color.white;
                tile.SetActive(GetAt(bitmap, i));
            }
            return 0f;
        });

        fg.map_container.rsgrid.ForAllIndexed(f, true);
    
    }

    private void DrawVisibility()
    {
        var visibility = fg.map_container.visibility;
        var bitmap = fg.map_container.bitmap;
        DrawMode(true, false, false);

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height)
            {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.localPosition = pos;
                tile.SetActive(!GetAt(bitmap, i));
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
        DrawMode(true, false, false);

        var f = new Func<Vector<double>, Vector<float>, double>((x, i) =>
        {
            if (x.At(1) < render_height && x.At(1) + fg.map_container.rsgrid.cell_size > render_height)
            {
                var tile = map_tiles[(int)i.At(0), (int)i.At(2)];
                var pos = new Vector3((float)x.At(0), render_height - 0.5f, (float)x.At(2));
                tile.transform.localPosition = pos;
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
        DrawMode(false, false, false);
    }

    private void DrawTour()
    {
        var tour = fg.trajectory_container.tour;
        DrawMode(false, true, true);

        line_renderer.positionCount = tour.Count;
        for(int i = 0; i < tour.Count; i++)
        {
            if(relative_to != null)
            {
                line_renderer.SetPosition(i, (Vector3)(relative_to.localToWorldMatrix * RSUtils.Utils.VToV3(tour[i])) + relative_to.transform.position);
            }
            else
            {
                line_renderer.SetPosition(i, RSUtils.Utils.VToV3(tour[i]));
            }
        }
    }

    private void DrawControlPoints()
    {
        DrawMode(false, true, false);
    }
    private void DrawTrajectory()
    {
        var trajectory = fg.trajectory_container.trajectory;
        DrawMode(false, true, true);

        Vector3 pos;

        line_renderer.positionCount = trajectory.Count / 3;
        for(int i = 0; i < trajectory.Count; i+=3)
        {
            pos = new Vector3((float)trajectory[i], (float)trajectory[i+1], (float)trajectory[i+2]);
            if (relative_to != null)
            {
                line_renderer.SetPosition(i / 3, (Vector3)(relative_to.localToWorldMatrix * pos) + relative_to.transform.position);
            }
            else
            {
                line_renderer.SetPosition(i / 3, pos);

            }
        }
        
    }
}
