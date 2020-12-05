using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


public class Cast : MonoBehaviour
{
    static Vector3[] GetChessboard(int width, int height, float randomNoiseMultiplier, float scale)
    {
        var res = new Vector3[width * height];

        for (var i = 0; i < width; i++)
        {
            for (var j = 0; j < height; j++)
            {
                var v = new Vector3(i - width / 2.0f, j - height / 2.0f);
                var noiseX = Mathf.PerlinNoise(v.x * scale, v.y * scale);
                var noiseY = Mathf.PerlinNoise(v.y * scale, v.x * scale);
                var noiseV = new Vector3(noiseX / 2 - 1, noiseY / 2 - 1) * randomNoiseMultiplier;
                res[width * i + j] = v + noiseV;
            }
        }

        return res;
    }

    public Transform[] cameras;
    public Transform laser;

    public int Width;
    public int Height;
    public float Scale;
    public float Distance;
    public float randomNoiseMultiplier = 0;
    public float randomScale = 1;
    public float labelOffset = 0;
    public bool drawPointLabels;
    public bool drawLaserRays;
    public bool drawCameraRays;
    public bool drawOriginPoints;
    public bool drawFoundPoints;

    Vector3[] laserPoints;
    List<Vector3> intersectedLaserPoints = new List<Vector3>();
    List<int>[] cameraRays;

    FoundPoint[] foundPoints = null;

    // Start is called before the first frame update
    void Start()
    {
        cameraRays = new List<int>[cameras.Length];
    }

    List<IndexedRay>[] GetIndexedRays(List<Vector3> intersectedLaserPoints, List<int>[] cameraRays)
    {
        var res = new List<IndexedRay>[cameraRays.Length];

        for (var camId = 0; camId < cameraRays.Length; camId++)
        {
            res[camId] = new List<IndexedRay>();
            foreach (var rayId in cameraRays[camId])
            {
                var direction = (cameras[camId].position - intersectedLaserPoints[rayId]).normalized;
                res[camId].Add(new IndexedRay { ray = new Ray(cameras[camId].position, direction), pointIndex = new RayIndex(camId, rayId) });
            }
        }

        return res;
    }

    // Update is called once per frame
    void Update()
    {
        laserPoints = GetChessboard(Width, Height, randomNoiseMultiplier, randomScale);

        intersectedLaserPoints = new List<Vector3>();
        for (var i = 0; i < cameraRays.Length; i++)
        {
            cameraRays[i] = new List<int>();
        }
        var pointIndex = 0;
        foreach (var point in laserPoints)
        {
            var direction = (transform.TransformVector(point * Scale + Vector3.forward * Distance) - transform.position).normalized;
            if (Physics.Raycast(new Ray(transform.position, direction), out var info))
            {
                var intersectedPoint = transform.position + direction * info.distance;
                intersectedLaserPoints.Add(intersectedPoint);

                for (var i = 0; i < cameras.Length; i++)
                {
                    var pointCamDirection = (cameras[i].position - intersectedPoint).normalized;
                    if (!Physics.Raycast(new Ray(intersectedPoint - direction, pointCamDirection)))
                    {
                        cameraRays[i].Add(pointIndex);
                    }
                }
                ++pointIndex;
            }
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            foundPoints = FindPoints.Find(GetIndexedRays(intersectedLaserPoints, cameraRays), 30.0f);
        }

        if (Input.GetKeyDown(KeyCode.A))
        {
            foundPoints = FindPoints.FindOld(GetIndexedRays(intersectedLaserPoints, cameraRays), 30.0f);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        foreach (var point in intersectedLaserPoints)
        {
            if (drawLaserRays)
            {
                Gizmos.DrawLine(transform.position, point);
            }
            if (drawOriginPoints)
            {
                Gizmos.DrawSphere(point, 20.0f);
            }
        }

        if (drawCameraRays)
        {
            Gizmos.color = Color.yellow;
            for (var i = 0; i < cameras.Length; i++)
            {
                foreach (var j in cameraRays[i])
                {
                    Gizmos.DrawLine(cameras[i].position, intersectedLaserPoints[j]);
                }
            }
        }

        Gizmos.color = Color.red;

        if (foundPoints != null && drawFoundPoints)
        {
            foreach (var point in foundPoints)
            {
                Gizmos.DrawSphere(point.point, 20.0f);

                if (drawPointLabels)
                {
                    var text = "[";
                    for (var i = 0; i < point.pointIds.Length; ++i)
                    {
                        if (i > 0)
                        {
                            text += ",";
                        }
                        text += $"({point.pointIds[i].camId},{point.pointIds[i].rayId})";
                    }
                    text += "]";
                    UnityEditor.Handles.color = Color.white;
                    UnityEditor.Handles.Label(point.point + new Vector3(labelOffset, labelOffset, labelOffset), text);
                }
            }
        }
    }
}
