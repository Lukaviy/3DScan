using System.Collections;
using System.Collections.Generic;
using UnityEngine;

struct RayId
{
    public int testId;
    public RayIndex index;
}

namespace Visual
{
    public class Ray : MonoBehaviour
    {
        private UnityEngine.Ray m_ray;

        public void SetRay(UnityEngine.Ray ray)
        {
            transform.position = ray.origin;
            transform.LookAt(ray.origin + ray.direction);
        }

        public void SetLength(float length)
        {
            transform.localScale = new Vector3(1, 1, length);
        }
    }
}

