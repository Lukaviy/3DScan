using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Visual
{
    public class CameraVisual : MonoBehaviour
    {
        public GameObject imagePlane;

        private bool m_loadingState;

        private bool m_drawFrustum;
        private float m_frustumSize;
        private bool m_drawImagePlane;
        private bool m_invertYImagePlane;

        public Texture2D loadingTexture;

        public Material frustumMaterial;

        private Texture2D m_objectImage;
        private Texture2D m_pointsImage;

        private Matrix4x4 m_extrinsicMatrix;
        private Matrix4x4 m_intrinsicMatrix;
        private Vector2 m_resolution;

        public GroupId? GroupId { get; private set; }

        public bool InvertYImagePlane
        {
            get => m_invertYImagePlane;
            set
            {
                m_invertYImagePlane = value;
                m_dirtyVisual = true;
            }
        }

        private MathematicaRayLoader.ImageType m_imageType;

        public MathematicaRayLoader.ImageType ImageType
        {
            get => m_imageType;
            set
            {
                m_imageType = value;
                m_dirtyVisual = true;
            }
        }

        private bool m_dirtyVisual;

        public bool DrawFrustum
        {
            get => m_drawFrustum;
            set {
                if (m_drawFrustum == value)
                {
                    return;
                }
                m_drawFrustum = value;
                m_dirtyVisual = true;
            }
        }

        public bool DrawImagePlane
        {
            get => m_drawImagePlane;
            set {
                if (m_drawImagePlane == value)
                {
                    return;
                }
                m_drawImagePlane = value;
                m_dirtyVisual = true;
            }
        }

        private GameObject m_frustumHolder;

        public Matrix4x4 ExtrinsicMatrix
        {
            get => m_extrinsicMatrix;
            set
            {
                m_extrinsicMatrix = value;
                m_dirtyVisual = true;
            }
        }

        public Matrix4x4 IntrinsicMatrix
        {
            get => m_intrinsicMatrix;
            set
            {
                m_intrinsicMatrix = value;
                m_dirtyVisual = true;
            }
        }

        public float FrustumSize
        {
            get => m_frustumSize;
            set
            {
                if (m_frustumSize == value)
                {
                    return;
                }
                m_frustumSize = value;
                m_dirtyVisual = true;
            }
        }

        public Vector2 Resolution
        {
            get => m_resolution;
            set
            {
                m_resolution = value;
                m_dirtyVisual = true;
            }
        }

        public void ClearImages()
        {
            m_objectImage = null;
            m_pointsImage = null;
            
            m_dirtyVisual = true;
            GroupId = null;
        }
        
        public void SetImages(Texture2D objectImage, Texture2D pointsImage, GroupId groupId)
        {
            if (GroupId is { } id && id.testId == groupId.testId)
            {
                return;
            }

            m_objectImage = objectImage;
            m_pointsImage = pointsImage;

            m_dirtyVisual = true;
            GroupId = groupId;
        }

        public void SetImages(byte[] objectImage, byte[] pointsImage, GroupId groupId)
        {
            if (GroupId is { } id && id.testId == groupId.testId)
            {
                return;
            }

            m_objectImage ??= new Texture2D(2, 2);
            m_objectImage.LoadImage(objectImage);

            m_pointsImage ??= new Texture2D(2, 2);
            m_pointsImage.LoadImage(pointsImage);

            m_dirtyVisual = true;
            GroupId = groupId;
        }

        public void SetLoadingState(bool state)
        {
            m_dirtyVisual = m_loadingState != state;
            m_loadingState = state;
        }

        private Rect GetEyeSpaceImageRect()
        {
            var topLeft = IntrinsicMatrix * new Vector4(0, 0, 1, 1);
            var bottomRight = IntrinsicMatrix * new Vector4(m_resolution.x, m_resolution.y, 1, 1);
            return new Rect(topLeft, bottomRight - topLeft);
        }

        private Vector3[] GenerateFrustumPoints()
        {
            return new[]
            {
                new Vector2(0, 0),
                new Vector2(0, m_resolution.y),
                new Vector2(m_resolution.x, m_resolution.y),
                new Vector2(m_resolution.x, 0)
            }.Select(x => (IntrinsicMatrix * new Vector4(x.x, x.y, 1, 1)) * m_frustumSize).Select(x => (Vector3)(m_extrinsicMatrix * new Vector4(x.x, x.y, x.z, 1))).ToArray();
        }

        void UpdateFrustum()
        {
            transform.position = ExtrinsicMatrix.GetPosition();
            transform.rotation = ExtrinsicMatrix.rotation;

            if (m_frustumHolder is not null)
            {
                Destroy(m_frustumHolder);
            }

            if (m_drawFrustum)
            {
                m_frustumHolder = new GameObject { transform = { parent = transform } };

                var frustum = GenerateFrustumPoints();
                if (!m_drawImagePlane)
                {
                    var t = new GameObject { transform = { parent = m_frustumHolder.transform } };

                    var lineRenderer = t.AddComponent<LineRenderer>();
                    lineRenderer.material = frustumMaterial;
                    lineRenderer.positionCount = frustum.Length;
                    lineRenderer.SetPositions(frustum);
                    lineRenderer.startWidth = 0.01f;
                    lineRenderer.numCornerVertices = 5;
                    lineRenderer.loop = true;
                }

                foreach (var point in frustum)
                {
                    var t = new GameObject { transform = { parent = m_frustumHolder.transform } };

                    var lineRenderer = t.AddComponent<LineRenderer>();
                    lineRenderer.SetPositions(new[] { m_extrinsicMatrix.GetPosition(), point });
                    lineRenderer.startWidth = 0.01f;
                    lineRenderer.material = frustumMaterial;
                }
            }

            imagePlane.SetActive(m_drawImagePlane);
            if (m_drawImagePlane)
            {
                var rect = GetEyeSpaceImageRect();
                imagePlane.transform.localPosition = new Vector3(rect.center.x, rect.center.y, 1) * m_frustumSize;
                imagePlane.transform.localScale = (rect.size * m_frustumSize) * new Vector3(1, m_invertYImagePlane ? -1 : 1, 1);
            }
        }

        void UpdateImagePlane()
        {
            imagePlane.GetComponent<Renderer>().material.mainTexture =
                 m_loadingState ? loadingTexture : m_imageType == MathematicaRayLoader.ImageType.PointImage ? m_pointsImage : m_objectImage;
        }

        // Start is called before the first frame update
        void Start()
        {
            m_dirtyVisual = true;
        }

        // Update is called once per frame
        void Update()
        {
            if (m_dirtyVisual)
            {
                UpdateFrustum();
                UpdateImagePlane();
                m_dirtyVisual = false;
            }
        }

        void OnPostRender()
        {

        }
    }
}

