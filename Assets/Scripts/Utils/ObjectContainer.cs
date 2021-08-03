using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace RSUtils
{
    /// <summary>
    /// Wrapper class that takes an enumerable of points and turns it into a list of gameobjects with an in-scene representation
    /// Useful if a list of positions has to be exposed to the user.
    /// 
    /// Author: Robin Schmidiger
    /// Date: July 2021
    /// Version: 0.3
    /// </summary>
    public class ObjectContainer : Object
    {
        /// <summary>
        /// Turns an IEnumerable of vector3s into a list of gameobjects.
        /// the object's transform.position corresponds to the vector3s.
        /// </summary>
        /// <param name="vector3s">the IEnumerable of the vector3s to be instantiated</param>
        /// <param name="target">the transform of the target object that acts as a container</param>
        /// <param name="clear">if true, deletes all children of the target before instantiating gameobjects for the vector3s</param>
        public static void ToObjectContainer(IEnumerable<Vector3> vector3s, Transform target, bool clear = true)
        {
            if (clear)
            {
                foreach (Transform child in target)
                {
                    DestroyImmediate(child.gameObject);
                }
            }

            foreach (Vector3 v in vector3s)
            {
                GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj.name = v.ToString();
                obj.transform.localScale = Vector3.one * 0.2f;
                obj.transform.parent = target;
                obj.transform.localPosition = v;
            }
        }
        /// <summary>
        /// Turns a trajectory vector into a list of gameobjects.
        /// The object's transform.position correspons to 3 neighbouring components in the trajectory vector
        /// </summary>
        /// <param name="trajectory">The trajectory vector to be instantiated</param>
        /// <param name="target">The transform of the target object that </param>
        /// <param name="clear"></param>
        public static void ToObjectContainer(Vector<double> trajectory, Transform target, bool clear = true, int layer = 0)
        {
            if (clear)
            {
                foreach (Transform child in target)
                {
                    DestroyImmediate(child.gameObject);
                }
            }

            for (int i = 0; i < trajectory.Count; i += 3)
            {
                GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                obj.name = (i/3).ToString();
                obj.transform.localScale = Vector3.one * 0.2f;
                obj.transform.parent = target;
                obj.transform.localPosition = new Vector3((float)trajectory[i], (float)trajectory[i + 1], (float)trajectory[i + 2]);
                obj.layer = layer;
            }
        }

        /// <summary>
        /// Turns a list of positional vectors into a list of gameobjects
        /// </summary>
        /// <param name="vectors">the reference list of positional vectors</param>
        /// <param name="target">the target transform that functions as the list parent</param>
        /// <param name="prefab">The prefab to be instantiated, null = primitive object</param>
        /// <param name="clear">Whether the target's children should be cleared first</param>
        /// <param name="layer">the layer to instantiate the gameobjects on</param>
        public static void ToObjectContainer(List<Vector<double>> vectors, Transform target, GameObject prefab=null, bool clear = true, int layer = 0)
        {
            if (clear)
            {
                foreach (Transform child in target)
                {
                    DestroyImmediate(child.gameObject);
                }
            }

            foreach (Vector<double> v in vectors)
            {
                if (prefab == null)
                {
                    Vector3 v3 = Utils.VToV3(v);
                    GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    obj.name = v3.ToString();
                    obj.transform.localScale = Vector3.one * 0.2f;
                    obj.transform.parent = target;
                    obj.transform.localPosition = v3;
                    obj.layer = layer;
                }
                else
                {
                    Vector3 v3 = Utils.VToV3(v);
                    GameObject obj = Instantiate(prefab, v3, Quaternion.identity, target);
                    obj.name = v3.ToString();
                    obj.layer = layer;
                }
            }
        }

        /// <summary>
        /// Given a target transform returns a list of the transform.position of the children
        /// Useful for exposing a list of vector3s to the user
        /// 
        /// </summary>
        /// <param name="target">the transform whose children should be returned</param>
        /// <param name="vector3s">avoid allocation of a new list by providing one. if null, a new list is created</param>
        /// <returns></returns>
        public static List<Vector3> FromObjectContainer(Transform target, List<Vector3> vector3s = null)
        {
            if (vector3s == null)
            {
                vector3s = new List<Vector3>();
            }
            else
            {
                vector3s.Clear();
            }

            foreach (Transform child in target)
            {
                vector3s.Add(child.localPosition);
            }

            return vector3s;
        }
    }
}
