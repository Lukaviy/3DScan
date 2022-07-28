using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct RayId
{
    public bool Equals(RayId other)
    {
        return testId == other.testId && index.rayId == other.index.rayId && index.camId == other.index.camId;
    }

    public override bool Equals(object obj)
    {
        return obj is RayId other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(testId, index);
    }

    public int testId;
    public RayIndex index;

    public RayId(int testId, RayIndex index)
    {
        this.testId = testId;
        this.index = index;
    }

    public static bool operator !=(RayId a, RayId b)
    {
        return a.testId != b.testId || a.index.rayId != b.index.rayId || a.index.camId != b.index.camId;
    }

    public static bool operator ==(RayId a, RayId b)
    {
        return a.testId == b.testId && a.index.rayId == b.index.rayId && a.index.camId == b.index.camId;
    }
}

namespace Visual
{
    public class Ray : MonoBehaviour
    {
        private UnityEngine.Ray m_ray;
        private float m_length = 1;
        private float m_radius = 1;

        private bool m_selected;

        public bool Selected { get; set; }

        public Material SelectedMaterial { get; set; }

        public Material NormalMaterial { get; set; }

        public RayId rayId;

        void Update()
        {
            if (Selected != m_selected)
            {
                GetComponent<Renderer>().material = Selected ? SelectedMaterial : NormalMaterial;
                m_selected = Selected;
            }
        }

        public void SetRay(UnityEngine.Ray ray)
        {
            transform.position = ray.origin;
            transform.LookAt(ray.origin + ray.direction);
        }

        public void SetLength(float length)
        {
            m_length = length;
            transform.localScale = new Vector3(m_radius * m_length, m_radius * m_length, m_length);
        }

        public void SetRadius(float radius)
        {
            m_radius = radius;
            transform.localScale = new Vector3(m_radius * m_length, m_radius * m_length, m_length);
        }
    }
}

