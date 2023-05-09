using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Web.DynamicData;
using System.Xml;
using CameraState;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using ImGuiNET;
using JetBrains.Annotations;
using ScanerState;
using UImGui;
using UnityEngine;
using UnityEngine.Rendering.Universal;
using Visual;
using static UnityEngine.UI.GridLayoutGroup;
using CameraData = ScanerState.CameraData;
using Color = UnityEngine.Color;

namespace ScanerState {
    struct ImageTypeRaw{}

    struct ImageTypeUndistorted{}

    struct ImageColorGray{}
    struct ImageColorBgr{}

    struct TypedImage<T, TR> where T : struct where TR : struct
    {
        public Mat val;

        public TypedImage(Mat val)
        {
            this.val = val;
        }
    }

    class CamMatrices 
    {
        public TypedImage<ImageTypeUndistorted, ImageColorBgr> undistortedBgr;
        public TypedImage<ImageTypeUndistorted, ImageColorGray> undistortedGray;
        public Mat transform;

        public CamMatrices()
        {
            undistortedBgr = new TypedImage<ImageTypeUndistorted, ImageColorBgr>(new Mat());
            undistortedGray = new TypedImage<ImageTypeUndistorted, ImageColorGray>(new Mat());
            transform = new Mat();
        }
    }

    class CameraData
    {
        public CamMatrices matCache;
        public Matrix4x4? transform;
        public Texture2D texture;
        public TypedImage<ImageTypeUndistorted, ImageColorBgr> image;

        public CameraData()
        {
            matCache = new CamMatrices();
            image = new TypedImage<ImageTypeUndistorted, ImageColorBgr>(new Mat());
        }
    }

    interface IState {}

    class CameraViewSettings
    {
        public float viewScale;
    }

    struct CamerasViewSettings
    {
        public int?[,] grid;
        public readonly Dictionary<int, CameraViewSettings> viewSettings;

        public CamerasViewSettings(int width, int height)
        {
            this.grid = new int?[width, height];
            this.viewSettings = new();
        }
    }

    class StartScanning : IState
    {
        public string path = String.Empty;
    }

    class PositionCalibration : IState
    {
        public Dictionary<int, CameraData> activeCameras;
        public readonly string path;
        public CamerasViewSettings camerasViewSettings;

        public Vector3[] objectPoints;
        public Vector2Int chessBoardSize;
        public float elementSize;

        public PositionCalibration(string path)
        {
            this.activeCameras = new();
            this.path = path;
            this.camerasViewSettings = new CamerasViewSettings(1, 1);
            chessBoardSize = new Vector2Int(3, 3);
            objectPoints = new Vector3[]
            {
                new(0, 0), new(0, 1), new(0, 2),
                new(1, 0), new(1, 1), new(1, 2),
                new(2, 0), new(2, 1), new(3, 2),
            };
        }
    }
}

public class Scanner : MonoBehaviour
{
    private ScanerState.IState m_state;

    public Scanner()
    {

    }

    void DrawCameraImages(List<CameraInfo> cameras)
    {

    }

    static void UndistortImage<T>(TypedImage<ImageTypeRaw, T> raw, TypedImage<ImageTypeUndistorted, T> undistorted,
        Matrix4x4 camMatrix, float[] distCoeffs) where T : struct
    {
        CvInvoke.Undistort(raw.val, undistorted.val, Math.Matrix4x4To3x3Array(camMatrix), new VectorOfFloat(distCoeffs));
    }

    static void GrayImage<T>(TypedImage<T, ImageColorBgr> bgr, TypedImage<T, ImageColorGray> gray) where T : struct
    {
        CvInvoke.CvtColor(bgr.val, gray.val, ColorConversion.Bgr2Gray);
    }

    static void CopyImageTo<T, TR>(TypedImage<T, TR> i1, TypedImage<T, TR> i2) where T : struct where TR : struct {
        i1.val.CopyTo(i2.val);
    }

    [CanBeNull]
    static Vector2[] GetChessboardPoints(Vector2Int size, TypedImage<ImageTypeUndistorted, ImageColorGray> image)
    {
        var points = new VectorOfPointF();

        var res = CvInvoke.FindChessboardCorners(image.val, new Size(size.x, size.y), points);

        if (!res)
        {
            return null;
        }

        CvInvoke.CornerSubPix(image.val, points, new(11, 11), new(-1, -1), new(30, 0.001));

        return points.ToArray().Select(x => new Vector2(x.X, x.Y)).ToArray();
    }

    static Vector3[] GetChessBoardWorldPoints(Vector2Int chessBoardSize, float elementSize)
    {
        var res = new Vector3[chessBoardSize.x * chessBoardSize.y];

        for (var i = 0; i < chessBoardSize.x; i++)
        {
            for (var j = 0; j < chessBoardSize.y; j++)
            {
                res[chessBoardSize.x * j + i] = new Vector3(i, j) * elementSize;
            }
        }

        return res;
    }

    struct CamPosResult
    {
        public Matrix4x4 position;
        public Vector2[] points;
    }

    static CamPosResult? GetCameraPosition(TypedImage<ImageTypeUndistorted, ImageColorBgr> image, Matrix4x4 intrinsicMatrix, float[] distCoeffs, ref CamMatrices camMatrices, Vector3[] objectPoints, Vector2Int chessBoardSize) {
        GrayImage(image, camMatrices.undistortedGray);

        var chessBoardPoints = GetChessboardPoints(chessBoardSize, camMatrices.undistortedGray);

        if (chessBoardPoints is null)
        {
            return null;
        }

        var res = Math.CalibrateCamera(intrinsicMatrix, objectPoints, chessBoardPoints);

        if (res is null)
        {
            return null;
        }

        return new CamPosResult
        {
            position = res.Value,
            points = chessBoardPoints
        };
    }

    static void ProjectChessboardPoints(TypedImage<ImageTypeUndistorted, ImageColorBgr> image, Vector2Int patterSize, Vector3[] objectPoints, Matrix4x4 position, Matrix4x4 intrinsicMatrix) {
        Math.Matrix4x4ToRvecAndTvec(position, out var rvec, out var tvec);

        var corners = CvInvoke.ProjectPoints(Math.Vector3ToMCvPoint3D32Fs(objectPoints), rvec, tvec, Math.Matrix4x4To3x3Array(intrinsicMatrix), new VectorOfFloat());

        CvInvoke.DrawChessboardCorners(image.val, new Size(patterSize.x, patterSize.y), new VectorOfPointF(corners), true);
    }

    void DrawPositionCalibrator(List<CameraInfo> cameras, NamedCameraTexture[] images, PositionCalibration calibration)
    {
        int[] size = { calibration.camerasViewSettings.grid.GetLength(0), calibration.camerasViewSettings.grid.GetLength(1) };
        if (ImGui.InputInt2("Grid Size", ref size[0])) {
            if (size[0] > 0 && size[1] > 0) {
                var newGrid = new int?[size[0], size[1]];

                for (var x = 0; x < size[0] && x < calibration.camerasViewSettings.grid.GetLength(0); x++) {
                    for (var y = 0; y < size[0] && y < calibration.camerasViewSettings.grid.GetLength(1); y++) {
                        newGrid[x, y] = calibration.camerasViewSettings.grid[x, y];
                    }
                }

                calibration.camerasViewSettings.grid = newGrid;
            }
        }

        int[] chessBoardSize = { calibration.chessBoardSize.x, calibration.chessBoardSize.y };
        if (ImGui.InputInt2("Chessboard Size", ref chessBoardSize[0]))
        {
            if (chessBoardSize.All(x => x >= 3)) {
                calibration.chessBoardSize = new Vector2Int(chessBoardSize[0], chessBoardSize[1]);

                calibration.objectPoints = GetChessBoardWorldPoints(calibration.chessBoardSize, calibration.elementSize);
            }
        }

        if (ImGui.InputFloat("Element size in meters", ref calibration.elementSize)) 
        {
            calibration.objectPoints = GetChessBoardWorldPoints(calibration.chessBoardSize, calibration.elementSize);
        }

        void DrawCameraView(CameraViewSettings viewSettings, CameraInfo cameraInfo, Texture2D texture) {
            var captureState = CameraImageGetter.GetCameraState(cameraInfo.deviceId.GetValueOrDefault(-1));

            switch (captureState) {
                case null or Stopped: {
                    ImGui.TextUnformatted(cameraInfo.name);
                    ImGui.TextUnformatted("Currently not recording");
                }
                break;
                case Error error: {
                    ImGui.TextUnformatted(cameraInfo.name);
                    ImGui.TextColored(Color.red, "Error:");
                    ImGui.TextUnformatted(error.message);
                }
                break;
                case Capturing: {
                    ImGui.Image(UImGuiUtility.GetTextureId(texture),
                        new Vector2(texture.width, texture.height) / 2, new Vector2(0, 1), new Vector2(1, 0));

                    ImGui.TextUnformatted(cameraInfo.name);
                }
                break;
            }
        }

        void DrawGridElement(CameraInfo cameraInfo, int uniqueId)
        {
            var viewSetting = calibration.camerasViewSettings.viewSettings.GetValueOrDefault(uniqueId);

            var texture = images.FirstOrDefault(t => t.id == cameraInfo.deviceId);

            if (texture == null) {
                return;
            }

            if (cameraInfo.selectedResolution is null) {
                ImGui.TextColored(Color.yellow, "Please select resolution");
                return;
            }

            var resolution = cameraInfo.availableResolutions[cameraInfo.selectedResolution.Value];

            var camData = calibration.activeCameras.GetValueOrDefault(uniqueId);

            if (camData is null)
            {
                camData = new ScanerState.CameraData();
                calibration.activeCameras.Add(uniqueId, camData);
            }

            var calibrationInfo =
                cameraInfo.calibrationInfos.FirstOrDefault(t =>
                    t.resolution == resolution);

            if (calibrationInfo is null) {
                ImGui.TextColored(Color.red, "No available calibration info for selected resolution");
                return;
            }

            if (texture.image is null) 
            {
                ImGui.TextColored(Color.yellow, "Texture is not available");
                return;
            }

            UndistortImage(new TypedImage<ImageTypeRaw, ImageColorBgr>(texture.image), camData.matCache.undistortedBgr, calibrationInfo.intrinsicMatrix, calibrationInfo.distortionCoeffs);
            var camPosResult = GetCameraPosition(camData.matCache.undistortedBgr, calibrationInfo.intrinsicMatrix, calibrationInfo.distortionCoeffs, ref camData.matCache, calibration.objectPoints, calibration.chessBoardSize);

            switch (camPosResult) {
                case null:
                    ImGui.TextColored(Color.red, "Chessboard is not detected");
                    DrawCameraView(viewSetting, cameraInfo, texture.texture);
                    break;
                case { } position:
                    ImGui.TextColored(Color.green, "Chessboard detected");

                    CopyImageTo(camData.matCache.undistortedBgr, camData.image);

                    ProjectChessboardPoints(camData.image, calibration.chessBoardSize, calibration.objectPoints, position.position, calibrationInfo.intrinsicMatrix);

                    CameraImageGetter.GetTexture(camData.image.val, ref camData.texture);

                    DrawCameraView(viewSetting, cameraInfo, camData.texture);

                    break;
            }
        }

        var grid = calibration.camerasViewSettings.grid;

        void DrawUniqueIdCombo(ref int? currentUniqueId) {
            var id = currentUniqueId;
            var cameraInfo = cameras.FirstOrDefault(v => v.uniqueId == id);

            var previewStr = cameraInfo != null ? $"{cameraInfo.name}: {id}" : "N/A";

            if (ImGui.BeginCombo("Choose Cam", previewStr)) {
                foreach (var info in cameras) {
                    if (ImGui.Selectable($"{info.name}: {info.uniqueId}")) {
                        currentUniqueId = info.uniqueId;
                    }
                }

                if (ImGui.Selectable("Reset")) {
                    currentUniqueId = null;
                }
                ImGui.EndCombo();
            }
        }

        if (ImGui.BeginTable("Views", grid.GetLength(0))) {
            for (var y = 0; y < grid.GetLength(1); y++) {
                ImGui.PushID(y);
                ImGui.TableNextRow();
                for (var x = 0; x < grid.GetLength(0); x++) {
                    ImGui.PushID(x);
                    ImGui.TableNextColumn();

                    if (grid[x, y] is { } uniqueId) {
                        var cameraInfo = cameras.FirstOrDefault(v => v.uniqueId == uniqueId);

                        if (cameraInfo is not null)
                        {
                            DrawGridElement(cameraInfo, uniqueId);
                        }
                        else
                        {
                            grid[x, y] = null;
                        }
                    }

                    DrawUniqueIdCombo(ref grid[x, y]);
                    ImGui.PopID();
                }
                ImGui.PopID();
            }

            ImGui.EndTable();
        }
    }

    public void Draw(List<CameraInfo> cameras, NamedCameraTexture[] images)
    {
        switch (m_state)
        {
            case null:
            {
                m_state = new StartScanning();
            } break;
            case StartScanning start:
            {
                ImGui.TextUnformatted("Scanning not started");
                ImGui.InputText("Path", ref start.path, 100);
                if (start.path.Length != 0 && !Directory.Exists(start.path))
                {
                    ImGui.TextUnformatted("Path does not exist. Create?");
                    if (ImGui.Button("Create path"))
                    {
                        Directory.CreateDirectory(start.path);
                    }
                }
                if (ImGui.Button("Start Position Calibration"))
                {
                    m_state = new PositionCalibration(start.path);
                }
            } break;
            case PositionCalibration calibration:
            {
                DrawPositionCalibrator(cameras, images, calibration);
            } break;
        }
    }

    private ScanerState.IState GetCurrentState => m_state;
}
