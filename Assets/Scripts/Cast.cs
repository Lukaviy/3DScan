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

    public enum FoundPointsDrawType
    {
        SingleColor,
        FoundOrder,
        Score,
        IntersectionsCount
    }

    public Transform[] cameras;
    public Transform laser;

    public int Width;
    public int Height;
    public float Scale;
    public float Distance;
    public float ChessboadRandomNoiseMultiplier = 0;
    public float ChessboardRandomScale = 1;
    public float RaysNoise = 0;
    public float Treshold = 30.0f;
    public int kNearestNeighbors = 4;
    public int kMinNearestNeighbors = 2;
    public float labelOffset = 0;
    public float sphereRadius = 20.0f;
    public FoundPointsDrawType foundPointsDrawType = FoundPointsDrawType.SingleColor;
    public bool drawPointLabels;
    public bool drawLaserRays;
    public bool drawNoisedLaserRays;
    public bool drawImportedLaserRays;
    public bool drawCameraRays;
    public bool drawOriginPoints;
    public bool drawFoundPoints;
    public bool drawCameraOpticLines;
    public bool drawPointIndex;
    public bool drawRaysParticipatedInPoints;
    public bool drawSizeAsScore;
    public bool drawScoreLabel;
    public int pointIndex = -1;
    public int drawImportedRaysWithIntersectionsCountMin = 2;
    public int drawImportedRaysWithIntersectionsCountMax = 4;
    public float importedRayLength = 2.0f;
    public string mathematicaString;
    public List<string> nearestNeighbors;

    Vector3[] laserPoints;
    List<Vector3> intersectedLaserPoints = new List<Vector3>();
    List<int>[] cameraRays;
    List<IndexedRay>[] importedCameraRays;
    List<IndexedRay>[] noisedRays;

    FoundPoint[] foundPoints = null;

    // Start is called before the first frame update
    void Start()
    {
        cameraRays = new List<int>[cameras.Length];
    }

    List<IndexedRay>[] GetIndexedRays(List<Vector3> intersectedLaserPoints, List<int>[] cameraRays, float noise)
    {
        Random.InitState(0);
        var res = new List<IndexedRay>[cameraRays.Length];

        for (var camId = 0; camId < cameraRays.Length; camId++)
        {
            res[camId] = new List<IndexedRay>();
            foreach (var rayId in cameraRays[camId])
            {
                var direction = (cameras[camId].position - intersectedLaserPoints[rayId] + new Vector3(Random.Range(-noise/2, noise/2), Random.Range(-noise / 2, noise / 2), Random.Range(-noise / 2, noise / 2))).normalized;
                //var noiseDistance = Random.Range(0, noise);
                //var noiseAngle = Random.Range(0, 2 * Mathf.PI);
                res[camId].Add(new IndexedRay { ray = new Ray(cameras[camId].position, direction), pointIndex = new RayIndex(camId, rayId) });
            }
        }

        return res;
    }

    // Update is called once per frame
    void Update()
    {
        laserPoints = GetChessboard(Width, Height, ChessboadRandomNoiseMultiplier, ChessboardRandomScale);

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

        noisedRays = GetIndexedRays(intersectedLaserPoints, cameraRays, RaysNoise);

        if (Input.GetKeyDown(KeyCode.N))
        {
            foundPoints = FindPoints.FindNew(noisedRays, Treshold);
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            foundPoints = FindPoints.Find(noisedRays, Treshold);
        }

        if (Input.GetKeyDown(KeyCode.A))
        {
            foundPoints = FindPoints.FindOld(noisedRays, Treshold);
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            importedCameraRays = MathematicaRayLoader.LoadCamVectors(@"C:\projects\python_side\test", 4);
            foundPoints = FindPoints.FindNew(importedCameraRays, Treshold);
        }

        if (Input.GetKeyDown(KeyCode.O))
        {
            importedCameraRays = MathematicaRayLoader.LoadCamVectors(@"C:\projects\python_side\test", 4);
            var importedCameraPoints = MathematicaRayLoader.LoadCamPoints(@"C:\projects\python_side\test", 4);
            foundPoints = FindPoints.FindBasedOnKNearestNeighbors(importedCameraRays, Treshold, importedCameraPoints, kNearestNeighbors, kMinNearestNeighbors);
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
                Gizmos.DrawSphere(point, sphereRadius);
            }
        }

        if (drawNoisedLaserRays)
        {
            Gizmos.color = Color.blue;
            foreach (var camera in noisedRays)
            {
                foreach (var ray in camera) {
                    Gizmos.DrawLine(ray.ray.origin, ray.ray.origin - ray.ray.direction * 4000.0f); 
                }
            }
        }

        if (drawImportedLaserRays)
        {
            Gizmos.color = Color.blue;
            foreach (var camera in importedCameraRays)
            {
                foreach (var ray in camera)
                {
                    Gizmos.DrawLine(ray.ray.origin, ray.ray.origin - ray.ray.direction * importedRayLength);
                }
            }
        }

        if (drawRaysParticipatedInPoints)
        {
            Gizmos.color = Color.blue;

            for (var i = 0; i < foundPoints.Length; i++)
            {
                if (pointIndex != -1 && i != pointIndex)
                {
                    continue;
                }

                var foundPoint = foundPoints[i];
                if (foundPoint.pointIds.Length >= drawImportedRaysWithIntersectionsCountMin &&
                    foundPoint.pointIds.Length <= drawImportedRaysWithIntersectionsCountMax)
                {
                    foreach (var foundPointPointId in foundPoint.pointIds)
                    {
                        var ray = importedCameraRays[foundPointPointId.camId][foundPointPointId.rayId];
                        Gizmos.DrawLine(ray.ray.origin, ray.ray.origin - ray.ray.direction * importedRayLength);
                    }
                }
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

        List<string> pointStrings = new List<string>();

        if (foundPoints != null && drawFoundPoints)
        {
            var delta = 1.0f / foundPoints.Length / 2;
            var H = 0.0f;

            var maxScore = 0.0f;

            if (foundPointsDrawType == FoundPointsDrawType.Score)
            {
                maxScore = foundPoints.Max(x => x.score);
            }

            for (var pointId = 0; pointId < foundPoints.Length; pointId++)
            {
                if (pointIndex != -1 && pointId != pointIndex)
                {
                    continue;
                }

                if (pointIndex != -1)
                { 
                    nearestNeighbors = foundPoints[pointId].nearestNeighborsPointId.Select(x => x != null ? "{" + string.Join(", ", x) + "}" : "").ToList();
                }

                pointStrings.Add("{" + string.Join(", ", foundPoints[pointId].pointIds.Select(x =>
                    $"<| \"camId\"->{x.camId}, \"pointId\"->{pointId}, \"rayId\"->{x.rayId}, \"nearestNeighborsPointId\"->{{{string.Join(", ",foundPoints[pointId].nearestNeighborsPointId[x.camId])}}}, \"nearestNeighborsRayId\"->{{{string.Join(", ", foundPoints[pointId].nearestNeighborsRayId[x.camId])}}}, \"intersectedNeighborsRayId\"->{{{string.Join(", ", foundPoints[pointId].intersectedNeighborsRayId[x.camId])}}}|>")) + "}");

                var point = foundPoints[pointId];

                if (foundPointsDrawType == FoundPointsDrawType.FoundOrder)
                {
                    Gizmos.color = Color.HSVToRGB(H, 1, 1);
                    H += delta;
                }
                if (foundPointsDrawType == FoundPointsDrawType.Score)
                {
                    Gizmos.color = Color.HSVToRGB(point.score / (maxScore * 2), 1, 1);
                }
                if (foundPointsDrawType == FoundPointsDrawType.IntersectionsCount)
                {
                    Gizmos.color = Color.HSVToRGB(1 - point.pointIds.Length / (1.0f * importedCameraRays.Length), 1, 1);
                }
                Gizmos.DrawSphere(point.point, drawSizeAsScore ? point.score * sphereRadius : sphereRadius);

                if (drawPointIndex)
                {
                    UnityEditor.Handles.color = Color.white;
                    UnityEditor.Handles.Label(point.point + new Vector3(labelOffset, labelOffset, labelOffset), $"{pointId}");
                }

                if (drawScoreLabel)
                {
                    UnityEditor.Handles.color = Color.white;
                    UnityEditor.Handles.Label(point.point + new Vector3(labelOffset, labelOffset, labelOffset), $"{point.score}");
                }

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

        mathematicaString = "{" + string.Join(", ", pointStrings) + "}";

        if (drawCameraOpticLines)
        {
            Gizmos.color = Color.yellow;
            foreach (var camera in cameras)
            {
                Gizmos.DrawRay(new Ray(camera.position, camera.forward));
            }
        }
    }
}
