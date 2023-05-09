using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Structure;
using UnityEngine;
using UnityEngine.Rendering.Universal;
using Visual;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Random = System.Random;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;
using Vector4 = UnityEngine.Vector4;

public struct SimulationResult
{
    public List<IndexedRay>[] rays;
    public List<Vector3> originalPoints;
    public List<Vector2>[] camPoints;
}

public struct SimulationResultData
{
    public MathematicaRayLoader.CamPosData[] cameras;
    public SimulationResult[] result;

    public RawExperimentData ToRawExperimentData()
    {
        return new RawExperimentData
        {
            cameras = cameras,
            tests = result.Select(x => new MathematicaRayLoader.TestResult { camPoints = x.camPoints, rays = x.rays }).ToArray()
        };
    }
}

public class SceneToScan : MonoBehaviour
{
    public CameraEmulator[] cameras;
    public Laser laser;
    public ChessBoard marker;

    public string cameraIntrinsicsPath;

    public float noise = 1.0f;

    static float RandomGaussian(float mean, float sigma) {
        float u, v, S;

        do {
            u = 2.0f * UnityEngine.Random.value - 1.0f;
            v = 2.0f * UnityEngine.Random.value - 1.0f;
            S = u * u + v * v;
        }
        while (S >= 1.0f);

        // Standard Normal Distribution
        float std = u * Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);

        // Normal Distribution centered between the min and max value
        // and clamped following the "three-sigma rule"
        return std * sigma + mean;
    }

    static CameraData LoadCameraData(string cameraIntrinsicsPath, int camId)
    {
        var resolution = MathematicaRayLoader.LoadCamResolution(cameraIntrinsicsPath, camId);
        var intrinsicMatrix = MathematicaRayLoader.LoadCamCalibMatrix(cameraIntrinsicsPath, camId);

        return new CameraData{ cameraId = camId, intrinsicMatrix = intrinsicMatrix, resolution = resolution };
    }

    static CameraData[] LoadCamerasData(string cameraIntrinsicsPath, int[] camIds)
    {
        return camIds.Select(camId => LoadCameraData(cameraIntrinsicsPath, camId)).ToArray();
    }

    static Vector2 ProjectPointOnCamera(Vector3 point, Matrix4x4 intrinsicMatrix, Matrix4x4 inverseExtrinsicCamera)
    {
        var res = intrinsicMatrix * inverseExtrinsicCamera * new Vector4(point.x, point.y, point.z, 1.0f);
        return new Vector2(res.x, res.y) / res.z;
    }

    static Vector2? ProjectPointCameraIfVisible(Vector3 point, Matrix4x4 intrinsicMatrix, Vector2 cameraResolution, Matrix4x4 inverseExtrinsicCamera, Vector3 cameraPos)
    {
        var layerMask = LayerMask.GetMask("UI");
        var direction = cameraPos - point;
        if (Physics.Raycast(cameraPos, direction, direction.magnitude, ~layerMask))
        {
            return null;
        }

        var res = ProjectPointOnCamera(point, intrinsicMatrix, inverseExtrinsicCamera);

        if (res.x < 0 || res.x > cameraResolution.x || res.y < 0 || res.y > cameraResolution.y)
        {
            return null;
        }

        return res;
    }

    static Vector3 GetWorldSpaceDirection(Vector2 pointOnCamImage, Matrix4x4 inverseIntrinsicMatrix, Matrix4x4 extrinsicMatrix)
    {
        var origin = extrinsicMatrix * new Vector4(0, 0, 0, 1);
        var destination = extrinsicMatrix * inverseIntrinsicMatrix * new Vector4(pointOnCamImage.x, pointOnCamImage.y, 1, 1);
        return destination - origin;
    }

    static Vector2 AddNoise(Vector2 point, float noise)
    {
        var xNoise = RandomGaussian(0, noise);
        var yNoise = RandomGaussian(0, noise);

        return new Vector2(point.x + xNoise, point.y + yNoise);
    }

    Matrix4x4[] CalibrateCameras(CameraData[] cameraData, float noise)
    {
        var imagePoints = new List<Vector2>[cameraData.Length];

        for (var camId = 0; camId < cameraData.Length; camId++)
        {
            imagePoints[camId] = marker.Points.Select(x => AddNoise(ProjectPointCameraIfVisible(
                x, 
                cameraData[camId].intrinsicMatrix,
                cameraData[camId].resolution, 
                cameras[camId].transform.worldToLocalMatrix,
                cameras[camId].transform.position).Value, noise)
            ).ToList();
        }

        return Math.CalibrateCameras(cameraData, marker.Points, imagePoints).Select(x => x.Value).ToArray();
    }

    public SimulationResult GenerateTestResult(CameraData[] cameraData, Matrix4x4[] cameraExtrinsics, float noise)
    {
        var points = new List<Vector3>();

        var layerMask = LayerMask.GetMask("UI");

        for (var x = 0; x < laser.xResolution; x++)
        {
            for (var y = 0; y < laser.yResolution; y++)
            {
                var localPoint = new Vector3(
                    ((float)(laser.xResolution - x) / laser.xResolution - 0.5f) * laser.xFOV, 
                    1,
                    ((float)(laser.yResolution - y) / laser.yResolution - 0.5f) * laser.yFOV
                    );

                var pos = laser.transform.position;
                var dir = laser.transform.TransformVector(localPoint).normalized;
                var hit = Physics.Raycast(pos, dir, out var hitInfo, Mathf.Infinity, ~layerMask);

                if (hit)
                {
                    points.Add(pos + dir * hitInfo.distance);
                }
            }
        }

        var rays = new List<IndexedRay>[cameraExtrinsics.Length];
        var camPoints = new List<Vector2>[cameraExtrinsics.Length];

        for (var camId = 0; camId < cameraData.Length; camId++)
        {
            rays[camId] = new List<IndexedRay>();
            camPoints[camId] = new List<Vector2>();

            var camPos = cameraExtrinsics[camId] * new Vector4(0, 0, 0, 1);
            var inverseExtrinsicPos = cameraExtrinsics[camId].inverse;
            var extrinsicPos = cameraExtrinsics[camId];

            var inverseIntrinsicMatrix = cameraData[camId].intrinsicMatrix.inverse;

            for (var pointId = 0; pointId < points.Count; pointId++)
            {
                var point = points[pointId];

                var camPointOpt = ProjectPointCameraIfVisible(point, cameraData[camId].intrinsicMatrix,
                    cameraData[camId].resolution, inverseExtrinsicPos, camPos);

                if (camPointOpt is not { } camPoint)
                {
                    continue;
                }

                var noisedCamPoint = AddNoise(camPoint, noise);

                var noisedRayDirection = GetWorldSpaceDirection(noisedCamPoint, inverseIntrinsicMatrix,
                    extrinsicPos);

                rays[camId].Add(new IndexedRay
                    { ray = new UnityEngine.Ray(camPos, noisedRayDirection), pointIndex = new RayIndex(camId, pointId) });
                camPoints[camId].Add(noisedCamPoint);
            }
        }

        return new SimulationResult { camPoints = camPoints, rays = rays, originalPoints = points };
    }


    public SimulationResultData CreateRawExperimentDataGroundTruthCamPositions(float noise) {
        var cameraData = LoadCamerasData(cameraIntrinsicsPath, cameras.Select(x => x.cameraId).ToArray());

        var test = GenerateTestResult(cameraData, cameras.Select(x => x.transform.localToWorldMatrix).ToArray(), noise);

        return new SimulationResultData {
            cameras = cameraData.Select((x, i) => new MathematicaRayLoader.CamPosData
            {
                inverseCalibMatrix = x.intrinsicMatrix.inverse,
                inverseTransformMatrix = cameras[i].transform.localToWorldMatrix
            }).ToArray(),
            result = new[] { test }
        };
    }

    public SimulationResultData CreateRawExperimentData(float noise) {
        var cameraData = LoadCamerasData(cameraIntrinsicsPath, cameras.Select(x => x.cameraId).ToArray());

        var inverseExtrinsicMatrices = CalibrateCameras(cameraData, noise).Select(x => x.inverse).ToArray();

        var test = GenerateTestResult(cameraData, inverseExtrinsicMatrices, noise);

        return new SimulationResultData {
            cameras = cameraData.Select((x, i) => new MathematicaRayLoader.CamPosData {
                inverseCalibMatrix = x.intrinsicMatrix.inverse,
                inverseTransformMatrix = inverseExtrinsicMatrices[i]
            }).ToArray(),
            result = new[] { test }
        };
    }
}
