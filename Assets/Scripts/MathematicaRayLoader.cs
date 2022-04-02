using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public static class MathematicaRayLoader
{
    private static List<Vector3> LoadVectors(string path)
    {
        var res = new List<Vector3>();

        string[] lines = System.IO.File.ReadAllLines(path);
        foreach (var line in lines)
        {
            var r = Regex.Match(line, @"(?<x>[-\d.]+),(?<y>[-\d.]+),(?<z>[-\d.]+)");

            var x = float.Parse(r.Groups["x"].Value, CultureInfo.InvariantCulture);
            var y = float.Parse(r.Groups["y"].Value, CultureInfo.InvariantCulture);
            var z = float.Parse(r.Groups["z"].Value, CultureInfo.InvariantCulture);

            res.Add(new Vector3(x, y, z));
        }

        return res;
    }

    public static List<IndexedRay>[] LoadCamVectors(string path, int count)
    {
        var res = new List<IndexedRay>[count];

        for (int i = 0; i < count; i++)
        {
            res[i] = new List<IndexedRay>();

            var rayStarts = LoadVectors($"{path}\\ray_start{i}.csv");
            var rayEnds = LoadVectors($"{path}\\ray_end{i}.csv");

            for (int j = 0; j < rayStarts.Count; j++)
            {
                res[i].Add(new IndexedRay{ ray = new Ray(rayStarts[j], (rayStarts[j] - rayEnds[j]).normalized), pointIndex = new RayIndex(i, j)});
            }
        }

        return res;
    }
    private static List<Vector2> LoadPoints(string path)
    {
        var res = new List<Vector2>();

        string[] lines = System.IO.File.ReadAllLines(path);

        foreach (var line in lines)
        {
            var r = Regex.Match(line, @"(?<x>[-\d.e+]+),(?<y>[-\d.e+]+)");

            var x = float.Parse(r.Groups["x"].Value, CultureInfo.InvariantCulture);
            var y = float.Parse(r.Groups["y"].Value, CultureInfo.InvariantCulture);

            res.Add(new Vector2(x, y));
        }

        return res;
    }

    public static List<Vector2>[] LoadCamPoints(string path, int count)
    {
        var res = new List<Vector2>[count];

        for (int i = 0; i < count; i++)
        {
            res[i] = LoadPoints($"{path}\\points{i}.csv");
        }

        return res;
    }

    public struct CamPosData
    {
        public Matrix4x4 inverseTransformMatrix;
        public Matrix4x4 inverseCalibMatrix;
    }

    public struct TestResult
    {
        public List<IndexedRay>[] rays;
        public List<Vector2>[] camPoints;
    }

    public static Vector3 LoadTranslationVector(string path)
    {
        string[] lines = System.IO.File.ReadAllLines(path);

        var x = float.Parse(lines[0], CultureInfo.InvariantCulture);
        var y = float.Parse(lines[1], CultureInfo.InvariantCulture);
        var z = float.Parse(lines[2], CultureInfo.InvariantCulture);

        return new Vector3(x, y, z);
    }

    public static Matrix4x4 LoadMatrix(string path)
    {
        string[] lines = System.IO.File.ReadAllLines(path);

        var r = new Regex(@"(?<x>[-\d.e+]+),(?<y>[-\d.e+]+),(?<z>[-\d.e+]+)");

        var r0 = r.Match(lines[0]);

        var m00 = float.Parse(r0.Groups["x"].Value, CultureInfo.InvariantCulture);
        var m01 = float.Parse(r0.Groups["y"].Value, CultureInfo.InvariantCulture);
        var m02 = float.Parse(r0.Groups["z"].Value, CultureInfo.InvariantCulture);

        var r1 = r.Match(lines[1]);

        var m10 = float.Parse(r1.Groups["x"].Value, CultureInfo.InvariantCulture);
        var m11 = float.Parse(r1.Groups["y"].Value, CultureInfo.InvariantCulture);
        var m12 = float.Parse(r1.Groups["z"].Value, CultureInfo.InvariantCulture);

        var r2 = r.Match(lines[2]);

        var m20 = float.Parse(r2.Groups["x"].Value, CultureInfo.InvariantCulture);
        var m21 = float.Parse(r2.Groups["y"].Value, CultureInfo.InvariantCulture);
        var m22 = float.Parse(r2.Groups["z"].Value, CultureInfo.InvariantCulture);

        return new Matrix4x4(
            new Vector4(m00, m01, m02, 0),
            new Vector4(m10, m11, m12, 0),
            new Vector4(m20, m21, m22, 0),
            new Vector4(0, 0, 0, 1)
        ).transpose;
    }

    public static Matrix4x4 LoadTransformMatrix(string path, int cam, float scale)
    {
        return Matrix4x4.Translate(LoadTranslationVector($"{path}/cam{cam}vec.csv") * scale) *
               LoadMatrix($"{path}/cam{cam}mat.csv");
    }

    public static Matrix4x4 LoadTransformMatrixFromAxesVectors(string path, int cam)
    {
        var vectors = LoadVectors($"{path}/cam{cam}axes.csv");

        var origin = vectors[0];
        var x = vectors[1] - vectors[0];
        var y = vectors[2] - vectors[0];
        var z = vectors[3] - vectors[0];

        return Matrix4x4.LookAt(origin, origin + z, y);
    }

    public static float LoadScale(string path)
    {
        string[] lines = System.IO.File.ReadAllLines(path);

        return float.Parse(lines[0], CultureInfo.InvariantCulture);
    }

    public static Matrix4x4[] LoadInverseCamCalibMatrices(string path, int[] camsIndices)
    {
        Matrix4x4[] res = new Matrix4x4[camsIndices.Max() + 1];

        for (int cam_index_id = 0; cam_index_id < camsIndices.Length; cam_index_id++)
        {
            var calibMatrix = LoadMatrix($"{path}/cam{camsIndices[cam_index_id]}calibmtx.csv");
            var inverseCalibMatrix = calibMatrix.inverse;

            res[cam_index_id] = inverseCalibMatrix;
        }

        return res;
    }

    public static CamPosData[] LoadCamPosDataFromMatrices(string path, int[] camsIndices)
    {
        CamPosData[] res = new CamPosData[camsIndices.Max() + 1];

        var scale = LoadScale($"{path}/scale.csv");

        var calibMatrices = LoadInverseCamCalibMatrices(path, camsIndices);

        for (int cam_index_id = 0; cam_index_id < camsIndices.Length; cam_index_id++)
        {
            var transformMatrix = LoadTransformMatrix(path, camsIndices[cam_index_id], scale);
            var inverseTransformMatrix = transformMatrix.inverse;
            var inverseCalibMatrix = calibMatrices[cam_index_id];

            res[cam_index_id] = new CamPosData
                { inverseCalibMatrix = inverseCalibMatrix, inverseTransformMatrix = inverseTransformMatrix };
        }

        return res;
    }

    public static CamPosData[] LoadCamPosDataFromAxes(string path, int[] camsIndices)
    {
        CamPosData[] res = new CamPosData[camsIndices.Max() + 1];

        var calibMatrices = LoadInverseCamCalibMatrices(path, camsIndices);

        for (int cam_index_id = 0; cam_index_id < camsIndices.Length; cam_index_id++)
        {
            res[cam_index_id] = new CamPosData
            {
                inverseCalibMatrix = calibMatrices[cam_index_id], 
                inverseTransformMatrix = LoadTransformMatrixFromAxesVectors(path, cam_index_id)
            };
        }

        return res;
    }

    public static CamPosData[] LoadCamPosData(string path, int[] camsIndices)
    {
        return System.IO.File.Exists($"{path}/scale.csv")
            ? LoadCamPosDataFromMatrices(path, camsIndices)
            : LoadCamPosDataFromAxes(path, camsIndices);
    }

    public static TestResult LoadTestResult(string path, CamPosData[] calibMatrices, int[] camsIndices)
    {
        List<IndexedRay>[] res_rays = new List<IndexedRay>[camsIndices.Max() + 1];
        List<Vector2>[] res_camPoints = new List<Vector2>[camsIndices.Max() + 1];

        var scale = LoadScale($"{path}/scale.csv");

        for (int cam_index_id = 0; cam_index_id < camsIndices.Length; cam_index_id++)
        {
            var inverseTransformMatrix = calibMatrices[cam_index_id].inverseTransformMatrix;
            var inverseCalibMatrix = calibMatrices[cam_index_id].inverseCalibMatrix;

            var points = LoadPoints($"{path}/cam{camsIndices[cam_index_id]}Pnts.csv");

            var rays = points.Select(point =>
                inverseCalibMatrix * new Vector4(point.x, point.y, 1, 1)
            ).Select((pointInEyeSpace, i) =>
            {
                var origin = inverseTransformMatrix * new Vector4(0, 0, 0, 1);
                var destination = inverseTransformMatrix * pointInEyeSpace;
                var direction = destination - origin;
                return new IndexedRay
                {
                    ray = new Ray(origin, direction),
                    pointIndex = new RayIndex(cam_index_id, i)
                };
            }).ToList();

            res_rays[cam_index_id] = rays;

            res_camPoints[cam_index_id] = points;
        }

        return new TestResult { camPoints = res_camPoints, rays = res_rays };
    }

    public static TestResult LoadTestResult(string path, int[] camsIndices)
    {
        var calibMatrices = LoadCamPosData(path, camsIndices);
        return LoadTestResult(path, calibMatrices, camsIndices);
    }

    public static List<TestResult> LoadTestsResults(string path, string camPosesPath, int testsCount, int[] camsIndices)
    {
        var result = new List<TestResult>();
        var camPoses = LoadCamPosData(camPosesPath, camsIndices);

        for (var test_id = 0; test_id < testsCount; test_id++)
        {
            result.Add(LoadTestResult($"{path}/test_{test_id}", camPoses, camsIndices));
        }

        return result;
    }

    public static List<TestResult> LoadTestsResults(string path, int testsCount, int[] camsIndices)
    {
        var result = new List<TestResult>();

        for (var test_id = 0; test_id < testsCount; test_id++)
        {
            result.Add(LoadTestResult($"{path}/test_{test_id}", camsIndices));
        }

        return result;
    }

    public struct LoadTestsProgress
    {
        public int loaded;
        public int count;
    }

    public struct ExperimentData
    {
        public List<TestResult> tests;
        public CamPosData[] cameras;
    }

    public static ExperimentData LoadTestsResults(string path, int[] camsIndices, CancellationToken token, IProgress<LoadTestsProgress> progress)
    {
        var r = new Regex(@"test_(\d+)");
        var testsCount = Directory.GetDirectories(path).Select(x => { var t = r.Match(x);
            return t.Success ? int.Parse(t.Groups[1].Value) : -1;
        }).Max() + 1;

        var camPoses = LoadCamPosData($"{path}/test_0", camsIndices);

        var result = new ExperimentData{ tests = new List<TestResult>(), cameras = camPoses };

        for (var test_id = 0; test_id < testsCount; test_id++)
        {
            token.ThrowIfCancellationRequested();
            result.tests.Add(LoadTestResult($"{path}/test_{test_id}", camPoses, camsIndices));
            progress.Report(new LoadTestsProgress{ loaded = test_id, count = testsCount });
        }

        return result;
    }

    public static void SavePoints(string path, List<Vector3> points)
    {
        System.IO.File.WriteAllLines(path, points.Select(point => $"{point.x.ToString(CultureInfo.InvariantCulture)}, {point.y.ToString(CultureInfo.InvariantCulture)}, {point.z.ToString(CultureInfo.InvariantCulture)}").ToArray());
    }

    public static void SavePoints(string path, List<FoundPoint> points)
    {
        System.IO.File.WriteAllLines(path, points.Select(point => $"{point.point.x.ToString(CultureInfo.InvariantCulture)}, {point.point.y.ToString(CultureInfo.InvariantCulture)}, {point.point.z.ToString(CultureInfo.InvariantCulture)}, {point.score.ToString(CultureInfo.InvariantCulture)}").ToArray());
    }

    public static void SaveIntersectionGroups(string path, List<FindPoints.IntersectionGroup> groups)
    {
        System.IO.File.WriteAllLines(path, groups.Select(group => $"{String.Join(", ", group.camRayIndices.Select(x => x.ToString(CultureInfo.InvariantCulture)))}, {group.score.ToString(CultureInfo.InvariantCulture)}").ToArray());
    }

    public static void SavePointsWithFullInfo(string path, FoundPoint[] points)
    {
        System.IO.File.WriteAllLines(path, points.Select(point => $"{point.point.x.ToString(CultureInfo.InvariantCulture)}, {point.point.y.ToString(CultureInfo.InvariantCulture)}, {point.point.z.ToString(CultureInfo.InvariantCulture)}, {point.score.ToString(CultureInfo.InvariantCulture)}, {String.Join(", ", point.pointIds.Select(y => $"{y.camId}, {y.rayId}"))}").ToArray());
    }
}
