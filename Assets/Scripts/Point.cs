using System;
using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using UnityEngine;
public struct GroupId
{
    public bool Equals(GroupId other)
    {
        return groupIdInTest == other.groupIdInTest && testId == other.testId;
    }

    public override bool Equals(object obj)
    {
        return obj is GroupId other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(groupIdInTest, testId);
    }

    public GroupId(int testId, int groupId)
    {
        this.testId = testId;
        this.groupIdInTest = groupId;
    }

    public int groupIdInTest;
    public int testId;

    public static bool operator !=(GroupId a, GroupId b)
    {
        return a.testId != b.testId || a.groupIdInTest != b.groupIdInTest;
    }

    public static bool operator ==(GroupId a, GroupId b)
    {
        return a.testId == b.testId && a.groupIdInTest == b.groupIdInTest;
    }
}

namespace Visual
{
    public enum ColorizationType
    {
        None,
        IntersectionCount,
        Score
    }

    public class Point : MonoBehaviour
    {
        public GroupId groupId;

        private bool m_selectedSkin;
        private bool m_dirtyVisual;

        private float m_size;

        public Material selectedMaterial;
        public Material normalMaterial;

        private ColorizationType m_colorizationType = ColorizationType.None;

        public FoundPoint FoundPoint { get; set; }

        public int CamerasCount { get; set; }

        public float MaxScore { get; set; }

        public ColorizationType ColorizationType
        {
            get => m_colorizationType;
            set
            {
                if (m_colorizationType == value)
                {
                    return;
                }

                m_colorizationType = value;
                m_dirtyVisual = true;
            }
        }

        public Action<GroupId> onClick;
        public bool Selected { get; set; }

        private Renderer m_renderer;

        public float Size { get; set; }

        void Start()
        {
            m_renderer = GetComponent<Renderer>();
            m_dirtyVisual = true;
        }

        void Update()
        {
            if (Selected && !m_selectedSkin)
            {
                m_renderer.material = selectedMaterial;
                m_selectedSkin = true;
            }

            if (!Selected && m_selectedSkin)
            {
                m_renderer.material = normalMaterial;
                m_selectedSkin = false;
                m_dirtyVisual = true;
            }

            if (m_size != Size)
            {
                m_size = Size;
                transform.localScale = new Vector3(m_size, m_size, m_size);
            }

            if (m_dirtyVisual)
            {
                if (m_colorizationType == ColorizationType.IntersectionCount)
                {
                    m_renderer.material.color = Color.HSVToRGB(1 - FoundPoint.pointIds.Length / (1.0f * CamerasCount), 1, 1);
                }
                else if (m_colorizationType == ColorizationType.Score)
                {
                    m_renderer.material.color = Color.HSVToRGB(FoundPoint.score / (MaxScore * 2), 1, 1);
                }
                else
                {
                    m_renderer.material.color = Color.white;
                }

                m_dirtyVisual = false;
            }
        }

        void OnMouseDown()
        {
            onClick.Invoke(groupId);
        }
    }
}
