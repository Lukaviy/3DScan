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
    public class Point : MonoBehaviour
    {
        public GroupId groupId;

        private bool m_selectedSkin;

        public Material selectedMaterial;
        public Material normalMaterial;

        public Action<GroupId> onClick;
        public bool Selected { get; set; }

        private Renderer m_renderer;

        void Start()
        {
            m_renderer = GetComponent<Renderer>();
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
            }
        }

        void OnMouseDown()
        {
            onClick.Invoke(groupId);
        }
    }
}
