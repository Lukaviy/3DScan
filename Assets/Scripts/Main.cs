using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Web.UI.WebControls;
using ImGuiNET;
using ImGuizmoNET;
using UnityEngine;
using Newtonsoft.Json;
using UImGui;
using Visual;
using Newtonsoft.Json.UnityConverters.Math;

class ExperimentInfo
{
    public string Name { get; set; }
    public string Path { get; set; }
    public bool AutoLoad { get; set; }
}

[System.Serializable]
class CalibrationInfo
{
    public CalibrationInfo(DateTime calibrationTime, string name, Vector2 resolution, Matrix4x4 intrinsicMatrix, float[] distortionCoeffs)
    {
        this.calibrationTime = calibrationTime;
        this.name = name;
        this.resolution = resolution;
        this.intrinsicMatrix = intrinsicMatrix;
        this.distortionCoeffs = distortionCoeffs;
        this.guid = Guid.NewGuid();
    }

    public DateTime calibrationTime;
    public string name;
    public Guid guid;

    public Vector2 resolution;
    public Matrix4x4 intrinsicMatrix;
    public float[] distortionCoeffs;
}

[System.Serializable]
class CameraInfo
{
    public int uniqueId;
    public int? deviceId;
    public string name;

    public List<Vector2> availableResolutions;

    public List<CalibrationInfo> calibrationInfos;

    public int? selectedResolution;
    public Guid? selectedCalibrationInfo;
    public int exposure;
}

class Settings
{
    public Settings()
    {
        Experiments = new List<ExperimentInfo>();
    }

    public List<ExperimentInfo> Experiments { get; set; }

    public string CamerasInfoPath { get; set; } = string.Empty;

    public int? SelectedExperiment { get; set; }
    public bool ExperimentsWindowIsOpen { get; set; }
    public bool TestsWindowIsOpen { get; set; }

    [NonSerialized]
    public bool isDirty;
}

static class Loader
{
    public static Settings LoadExperimentsInfo(string path)
    {
        var text = File.ReadAllText(path);
        return JsonConvert.DeserializeObject<Settings>(text);
    }

    public static Settings LoadExperimentsInfo()
    {
        var path = "Experiments.json";
        return !File.Exists(path) ? new Settings() : LoadExperimentsInfo(path);
    }

    public static void SaveExperimentsInfo(string path, Settings info)
    {
        File.WriteAllText(path, JsonConvert.SerializeObject(info));
    }

    public static void SaveExperimentsInfo(Settings info)
    {
        SaveExperimentsInfo("Experiments.json", info);
    }
}

public class RawExperimentDataState
{
}

public class RawExperimentData : RawExperimentDataState
{
    public MathematicaRayLoader.CamPosData[] cameras;
    public MathematicaRayLoader.TestResult[] tests;
}

class RawExperimentDataLoadError : RawExperimentDataState
{
    public Exception exception;
}

class CalculateExperimentDataState
{
}

class CalculateExperimentDataReadyState : CalculateExperimentDataState
{
    public float treshold;
    public FindPoints.Event[] events;
    public FoundPoint[] points;
    public Dictionary<int, FindPoints.Event[]> groupEvents;
    public Dictionary<int, int> groupIdToPointIndex;
    public Dictionary<RayIndex, int> rayIndexToGroupId;
}

class CalculateExperimentDataInProgressState : CalculateExperimentDataState
{
    public TaskToken<CalculateExperimentDataReadyState> task;
    public CancellationTokenSource calculateDataCancellationToken;
}

class ExperimentData
{
    public RawExperimentDataState rawData;
    public CalculateExperimentDataState[] calculatedData;
}

class CameraInfosState {}

class CameraInfosLoadingState : CameraInfosState
{
    public TaskToken<List<CameraInfo>> taskToken;
}

class CameraInfosLoadErrorState : CameraInfosState 
{
    public string error;
}

class CameraInfosLoadedState : CameraInfosState 
{
    public List<CameraInfo> cameraInfos;
    public bool isDirty;
}

class State
{
}

class InitialState : State
{
}

class LoadExperimentsErrorState : State
{
    public string error;
}

class PhysicalCameraWindowState
{
    public bool isOpen = true;

    public class ImportCameraInfoState {

        public string cameraInfoPath = string.Empty;
        public string importError;
    }

    public ImportCameraInfoState importCameraInfoState = new ();

    public class ImportCalibInfoWindowState {
        public string currentImportPath = string.Empty;
        public string currentName = string.Empty;
        public bool calibMatrixFileExists;
        public bool distCoeffsFileExists;
        public bool fileExists;
        public bool userRequestedCustomName;
        public string loadErrorMessage;
        public Vector2 chosenResolution;
    }

    public ImportCalibInfoWindowState importCalibInfo = new ();

    public class AddPhysicalCameraState
    {
        public string currentName = string.Empty;
        public int uniqueId;
        public int[] resolution = new int[2];
    }

    public AddPhysicalCameraState addPhysicalCamera = new ();

    [System.Serializable]
    public struct PhysicalCameraCalibrationData
    {
        public float[,] intrinsicMatrix;
        public float[] distCoeffs;
    }
}

class ExperimentsWindowState
{
    private bool m_isDirty;
    private bool m_open = true;

    public bool Open
    {
        get => m_open;
        set
        {
            m_open = value;
            m_isDirty = true;
        }
    }
    public string currentPath = String.Empty;
    public string currentName = String.Empty;
    public bool directoryExists;
    public bool userRequestedCustomName;

    public bool IsDirty => m_isDirty;

    public void MarkClean()
    {
        m_isDirty = false;
    }
}

class LoadTestTaskData
{
    public TaskToken<MathematicaRayLoader.ExperimentData> loadTestResultTask;
    public MathematicaRayLoader.LoadTestsProgress loadTestResultProgress;
    public CancellationTokenSource loadTestResultCancellationToken;
}

struct CameraSelectionState
{
    public bool drawImagePlane;
    public float frustumSizeMultiplier;
    public bool drawFrustum;
}

class ExperimentDataState
{
    public ExperimentInfo info;
    public ExperimentData data;
    public CameraSelectionState[] cameraSelectionStates;
    public HashSet<int> selectedTests;
    public RayId? selectedRay;
    public LoadTestTaskData loadTestTask;
    public GroupId? selectedGroup;

    public ExperimentDataState(RawExperimentData rawData, string name)
    {
        info = new ExperimentInfo { Name = name };
        data = new ExperimentData
            { calculatedData = new CalculateExperimentDataState[rawData.tests.Length], rawData = rawData };
        cameraSelectionStates = new CameraSelectionState[rawData.cameras.Length];
        selectedTests = new HashSet<int>();
    }

    public ExperimentDataState(ExperimentInfo expInfo)
    {
        info = expInfo;
        data = new ExperimentData();
        selectedTests = new HashSet<int>();
    }
}

struct FilterData
{
    public bool EnableFilterByScore;
    public float MinScore;
    public float MaxScore;

    public bool EnableFilterByRayCount;
    public int MinRayCount;
    public int MaxRayCount;
}

[System.Serializable]
public struct SimulatedScene
{
    public GameObject scene;
    public string name;
}


[System.Serializable]
class LoadedExperimentsInfoSate : State
{
    public LoadedExperimentsInfoSate(Settings info)
    {
        m_settings = info;
        m_experiments = info.Experiments.Select(x => new ExperimentDataState(x)).ToList();
        ExperimentsWindowState = new ExperimentsWindowState{  };
    }

    private Settings m_settings;
    private bool m_settingsIsDirty;
    private List<ExperimentDataState> m_experiments;

    public int ExperimentsCount => m_experiments.Count;

    public float RayLength { get; set; } = 4;

    public float FrustumSize { get; set; } = 1;

    public float PointSize { get; set; } = 0.01f;

    public float CameraImageSizeScale { get; set; } = 0.1f;

    public bool DrawRays { get; set; }

    public bool LinkFrustumSize { get; set; }

    public ColorizationType ColorizationType { get; set; }

    public FilterData FilterData { get; set; }

    public ExperimentDataState GetExperimentDataState(int index)
    {
        return m_experiments[index];
    }
    public void AddExperiment(ExperimentInfo info)
    {
        m_experiments.Add(new ExperimentDataState(info));
        m_settingsIsDirty = true;
    }

    public void AddExperiment(RawExperimentData data, string name) {
        m_experiments.Add(new ExperimentDataState(data, name));
        m_settingsIsDirty = true;
    }

    public void RemoveExperiment(int index)
    {
        m_experiments.RemoveAt(index);
        m_settingsIsDirty = true;
        if (SelectedExperiment == index)
        {
            SelectedExperiment = null;
        }

        if (SelectedExperiment > index)
        {
            SelectedExperiment--;
        }
    }

    public int? SelectedExperiment
    {
        get => m_settings.SelectedExperiment;
        set {
            m_settings.SelectedExperiment = value;
            m_settingsIsDirty = true;
        }
    }

    public int? ExperimentDeleteRequested { get; set; }
    public ExperimentsWindowState ExperimentsWindowState { get; set; }

    public float Treshold { get; set; } = 0.01f;

    public bool TestsWindowsIsOpen { get; set; } = true;
    public bool PointInfoWindowsIsOpen { get; set; }

    public bool CameraImagesIsOpen { get; set; }

    public bool DrawOnlySelectedPoint { get; set; }

    public bool IsSettingsDirty => m_settingsIsDirty || m_settings.isDirty;

    public CameraInfosState CameraInfos;

    public PhysicalCameraWindowState PhysicalCameraWindowState { get; set; } = new ();

    public void AddPhysicalCameraInfo(CameraInfo cameraInfo)
    {
        if (CameraInfos is not CameraInfosLoadedState loadedState)
        {
            throw new Exception("Cameras info is not loaded");
        }

        if (loadedState.cameraInfos.Any(x => x.uniqueId == cameraInfo.uniqueId))
        {
            throw new Exception("Duplicate unique id");
        }

        loadedState.cameraInfos.Add(cameraInfo);
        loadedState.isDirty = true;
    }

    public void MarkSettingsClean()
    {
        m_settingsIsDirty = false;
    }

    public void MarkSettingsDirty()
    {
        m_settingsIsDirty = true;
    }

    public Settings Settings
    {
        get {
            m_settings.Experiments = m_experiments.Select(x => x.info).ToList();
            m_settings.ExperimentsWindowIsOpen = ExperimentsWindowState.Open;
            return m_settings;
        }
    }
}

class CameraImageState {}


class CameraImageRawLoadedState : CameraImageState
{
    public byte[] objectImage;
    public byte[] pointsImage;
}

class CameraImageReadyState : CameraImageState
{
    public Texture2D objectImage;
    public Texture2D pointsImage;
}

class CameraImageLoadState : CameraImageState
{
    public TaskToken<CameraImageRawLoadedState> token;
    public CancellationTokenSource cancellationToken;
}

class SimulatedScenesWindowState
{
    public int? selectedScene;
    public float noise = 1.0f;
    public SceneToScan sceneToScan;
}

class SimulatedSceneVisualCache
{
    public GameObject scene;
}

class PhysicalCameraCache
{
    public NamedCameraTexture[] cameraTextures;
}

class VisualCache
{
    public int? selectedExperiment;
    public Dictionary<GroupId, Visual.Point> pointObjects = new();
    public Dictionary<RayId, Visual.Ray> rayObjects = new();
    public List<CameraVisual> cameraObjects = new ();
    public GameObject pointPrefab;
    public GameObject rayPrefab;
    public GameObject cameraPrefab;
    public GroupId? selectedGroupId;
    public RayId? selectedRayId;
    public CameraImageState[] cameraImages;
    public bool drawRays;
    public float rayLength;
    public float pointSize;
    public bool drawOnlySelectedPoint;
    public ColorizationType colorizationType;
    public FilterData filterData;
}

public class Main : MonoBehaviour
{
    private State m_state = new InitialState();
    private TaskScheduler m_taskScheduler = new(8);
    private VisualCache m_visualCache;
    private SimulatedSceneVisualCache m_simulatedSceneVisualCache = new ();
    private SimulatedScenesWindowState m_simulatedScenesWindowState = new ();
    private PhysicalCameraCache m_physicalCameraCache = new();

    public GameObject PointPrefab;
    public GameObject RayPrefab;
    public GameObject CameraPrefab;

    public List<SimulatedScene> Scenes;

    void Start()
    {
        m_visualCache = new VisualCache { pointPrefab = PointPrefab, rayPrefab = RayPrefab, cameraPrefab = CameraPrefab };
    }

    // Start is called before the first frame update
    void OnEnable()
    {
        UImGuiUtility.Layout += OnLayout;
    }

    // Update is called once per frame
    void OnDisable()
    {
        UImGuiUtility.Layout -= OnLayout;
    }

    static void DrawVisual(LoadedExperimentsInfoSate state, VisualCache cache, TaskScheduler scheduler)
    {
        if (state.SelectedExperiment is null || state.SelectedExperiment.Value != cache.selectedExperiment)
        {
            foreach (var pointObject in cache.pointObjects)
            {
                Destroy(pointObject.Value.gameObject);
            }

            cache.pointObjects.Clear();

            foreach (var rayObject in cache.rayObjects)
            {
                Destroy(rayObject.Value.gameObject);
            }

            cache.rayObjects.Clear();

            foreach (var cameraObject in cache.cameraObjects)
            {
                Destroy(cameraObject.gameObject);
            }

            cache.cameraObjects.Clear();

            cache.cameraImages = null;
        }

        cache.selectedExperiment = state.SelectedExperiment;

        if (state.SelectedExperiment is null)
        {
            return;
        }

        var experiment = state.GetExperimentDataState(state.SelectedExperiment.Value);

        var pointsToDelete = cache.pointObjects.Where(x => !experiment.selectedTests.Contains(x.Key.testId)).ToArray();
        foreach (var pointObject in pointsToDelete)
        {
            Destroy(pointObject.Value.gameObject);
            cache.pointObjects.Remove(pointObject.Key);
        }

        var raysToDelete = cache.rayObjects.Where(x => !experiment.selectedTests.Contains(x.Key.testId)).ToArray();
        foreach (var rayObject in raysToDelete)
        {
            Destroy(rayObject.Value.gameObject);
            cache.rayObjects.Remove(rayObject.Key);
        }

        if (cache.selectedGroupId != experiment.selectedGroup)
        {
            foreach (var ray in cache.rayObjects.Values)
            {
                Destroy(ray.gameObject);
            }

            cache.rayObjects.Clear();
        }

        if (experiment.data.rawData is not RawExperimentData rawData)
        {
            return;
        }

        foreach (var testId in experiment.selectedTests)
        {
            if (experiment.data.calculatedData[testId] is CalculateExperimentDataReadyState readyState)
            {
                foreach (var point in readyState.points)
                {
                    var groupId = new GroupId(testId, point.groupIndex);
                    if (!cache.pointObjects.TryGetValue(groupId, out var existedPointObject))
                    {
                        var pointObject = Instantiate(cache.pointPrefab, point.point, Quaternion.identity);
                        var pointComponent = pointObject.GetComponent<Visual.Point>();
                        pointComponent.groupId.groupIdInTest = point.groupIndex;
                        pointComponent.groupId.testId = testId;
                        pointComponent.Size = state.PointSize;
                        pointComponent.FoundPoint = point;
                        pointComponent.CamerasCount = rawData.cameras.Length;
                        pointComponent.MaxScore = state.Treshold;
                        pointComponent.ColorizationType = state.ColorizationType;
                        pointComponent.onClick = (x) =>
                        {
                            experiment.selectedGroup = groupId;
                        };
                        cache.pointObjects.Add(groupId, pointComponent);
                    }
                    else
                    {
                        existedPointObject.Selected = experiment.selectedGroup != null &&
                                                      existedPointObject.groupId == experiment.selectedGroup.Value;

                        if (existedPointObject.Selected && experiment.selectedGroup != cache.selectedGroupId)
                        {
                            foreach (var rayIndex in readyState.points[readyState.groupIdToPointIndex[experiment.selectedGroup.Value.groupIdInTest]].pointIds)
                            {
                                var rayObject = Instantiate(cache.rayPrefab);
                                rayObject.SetActive(state.DrawRays);
                                var rayComponent = rayObject.GetComponent<Visual.Ray>();

                                rayComponent.SetRay(
                                    rawData.tests[testId].rays[rayIndex.camId][rayIndex.rayId].ray);

                                rayComponent.SetLength(state.RayLength);
                                rayComponent.SetRadius(readyState.treshold);

                                cache.rayObjects.Add(new RayId(testId, rayIndex), rayComponent);
                            }
                        }
                    }
                }
            }
        }

        if (cache.cameraObjects.Count == 0)
        {
            for (var camId = 0; camId < rawData.cameras.Length; camId++)
            {
                var rawDataCamera = rawData.cameras[camId];
                var cameraObject = Instantiate(cache.cameraPrefab);

                var cameraVisual = cameraObject.GetComponent<CameraVisual>();
                cameraVisual.DrawFrustum = experiment.cameraSelectionStates[camId].drawFrustum;
                cameraVisual.DrawImagePlane = experiment.cameraSelectionStates[camId].drawImagePlane;
                cameraVisual.ExtrinsicMatrix = rawDataCamera.inverseTransformMatrix;
                cameraVisual.IntrinsicMatrix = rawDataCamera.inverseCalibMatrix;
                cameraVisual.FrustumSize = state.FrustumSize;
                cameraVisual.InvertYImagePlane = true;
                cameraVisual.ImageType = MathematicaRayLoader.ImageType.PointImage;
                cameraVisual.Resolution = new Vector2(1280, 960);

                cache.cameraObjects.Add(cameraVisual);
            }
        }

        cache.cameraImages ??= new CameraImageState[rawData.cameras.Length];

        if (cache.selectedGroupId is {} csg && (experiment.selectedGroup is null || experiment.selectedGroup is {} esg && esg.testId != csg.testId))
        {
            for (var camId = 0; camId < rawData.cameras.Length; camId++)
            {
                if (cache.cameraImages[camId] is CameraImageLoadState loadState)
                {
                    loadState.cancellationToken.Cancel();
                }

                cache.cameraImages[camId] = null;
                cache.cameraObjects[camId].ClearImages();
            }
        }

        for (var camId = 0; camId < rawData.cameras.Length; camId++)
        {
            switch (cache.cameraImages[camId])
            {
                case CameraImageLoadState loadState:
                {
                    if (loadState.token.IsCompleted)
                    {
                        cache.cameraImages[camId] = loadState.token.Result;
                    }

                    break;
                }
            }
        }

        if (experiment.selectedGroup is { } selectedGroupId)
        {
            for (var camId = 0; camId < rawData.cameras.Length; camId++)
            {
                if (cache.cameraObjects[camId].GroupId is { } id && id.testId == selectedGroupId.testId)
                {
                    continue;
                }

                switch (cache.cameraImages[camId])
                {
                    case null:
                    {
                        var cancellationToken = new CancellationTokenSource();

                        var path = experiment.info.Path;
                        var testId = selectedGroupId.testId;
                        var i = camId;

                        if (path is null)
                        {
                            break;
                        }

                        var token = scheduler.EnqueueTask(() =>
                        {
                            var pointImage = MathematicaRayLoader.LoadTextureRaw(path, testId, i,
                                MathematicaRayLoader.ImageType.PointImage);

                            var objectImage = MathematicaRayLoader.LoadTextureRaw(path, testId, i,
                                MathematicaRayLoader.ImageType.ObjectImage);

                            return new CameraImageRawLoadedState
                            {
                                objectImage = objectImage,
                                pointsImage = pointImage
                            };
                        }, cancellationToken.Token);

                        cache.cameraImages[camId] = new CameraImageLoadState
                        {
                            cancellationToken = cancellationToken,
                            token = token
                        };

                        break;
                    }
                    case CameraImageLoadState loadState:
                    {
                        cache.cameraObjects[camId].SetLoadingState(!loadState.token.IsCompleted);

                        break;
                    }
                    case CameraImageReadyState readyState:
                    {
                        cache.cameraObjects[camId].SetImages(readyState.objectImage, readyState.pointsImage, selectedGroupId);
                        cache.cameraObjects[camId].SetLoadingState(false);

                        break;
                    }
                }
            }
        }

        for (var camId = 0; camId < experiment.cameraSelectionStates.Length; camId++)
        {
            var cameraObject = cache.cameraObjects[camId];
            cameraObject.DrawFrustum = experiment.cameraSelectionStates[camId].drawFrustum;
            cameraObject.FrustumSize = state.LinkFrustumSize ? state.FrustumSize : state.FrustumSize * experiment.cameraSelectionStates[camId].frustumSizeMultiplier;
            cameraObject.DrawImagePlane = experiment.cameraSelectionStates[camId].drawImagePlane;
        }

        if (state.DrawRays != cache.drawRays)
        {
            foreach (var rayObjectsValue in cache.rayObjects.Values)
            {
                rayObjectsValue.gameObject.SetActive(state.DrawRays);
            }
        }
        cache.drawRays = state.DrawRays;

        if (state.RayLength != cache.rayLength)
        {
            foreach (var rayObjectsValue in cache.rayObjects.Values)
            {
                rayObjectsValue.SetLength(state.RayLength);
            }
        }
        cache.rayLength = state.RayLength;

        if (state.PointSize != cache.pointSize)
        {
            foreach (var pointObject in cache.pointObjects.Values)
            {
                pointObject.Size = state.PointSize;
            }
        }
        cache.pointSize = state.PointSize;

        if (state.ColorizationType != cache.colorizationType)
        {
            foreach (var pointObject in cache.pointObjects.Values)
            {
                pointObject.ColorizationType = state.ColorizationType;
            }
        }
        cache.colorizationType = state.ColorizationType;

        
        if (state.FilterData.MinRayCount != cache.filterData.MinRayCount || state.FilterData.MinScore != cache.filterData.MinScore ||
            state.FilterData.MaxRayCount != cache.filterData.MaxRayCount || state.FilterData.MaxScore != cache.filterData.MaxScore)
        {
            foreach (var pointObject in cache.pointObjects.Values)
            {
                pointObject.gameObject.SetActive(
                    (!state.FilterData.EnableFilterByRayCount || pointObject.FoundPoint.pointIds.Length >= state.FilterData.MinRayCount && pointObject.FoundPoint.pointIds.Length <= state.FilterData.MaxRayCount) &&
                    (!state.FilterData.EnableFilterByScore || pointObject.FoundPoint.score >= state.FilterData.MinScore && pointObject.FoundPoint.score <= state.FilterData.MaxScore)
                );
            }
        }
        cache.filterData = state.FilterData;

        if (state.DrawOnlySelectedPoint != cache.drawOnlySelectedPoint)
        {
            foreach (var pointObject in cache.pointObjects.Values)
            {
                pointObject.gameObject.SetActive(!state.DrawOnlySelectedPoint || pointObject.groupId == experiment.selectedGroup);
            }
        }
        cache.drawOnlySelectedPoint = state.DrawOnlySelectedPoint;

        cache.selectedRayId = experiment.selectedRay;
        cache.selectedGroupId = experiment.selectedGroup;
    }

    Dictionary<int, FindPoints.Event[]> GetPointInfo(FindPoints.Event[] events)
    {
        var dict = new Dictionary<int, List<FindPoints.Event>>();

        void Add(int i, FindPoints.Event e)
        {
            if (!dict.TryGetValue(i, out var l))
            {
                l = new List<FindPoints.Event>();
                dict.Add(i, l);
            }

            l.Add(e);
        }

        void Import(int i, int j)
        {
            if (dict[j][0] is FindPoints.GroupMergeEvent e)
            {
                Import(i, e.group1Id);
                Import(i, e.group2Id);
            }

            if (!dict.TryGetValue(i, out var l))
            {
                l = new List<FindPoints.Event>();
                dict.Add(i, l);
            }

            l.AddRange(dict[j]);
        }

        foreach (var @event in events)
        {
            switch (@event)
            {
                case FindPoints.RayIntersectionEvent e: Add(e.groupId, e); break;
                case FindPoints.GroupMergeEvent e:
                {
                    Import(e.groupId, e.group1Id);
                    Import(e.groupId, e.group2Id);
                    Add(e.groupId, e);
                    Add(e.group1Id, e); 
                    Add(e.group2Id, e);

                    break;
                }
                case FindPoints.RayDeleteEvent e: Add(e.groupId, e); break;
                case FindPoints.GroupDeletedDueToRepeatEvent e: Add(e.groupId, e); break;
                case FindPoints.GroupRegisteredAsMax e: Add(e.groupId, e); break;
                case FindPoints.GroupIsUncotradicted e: Add(e.groupId, e); break;
                case FindPoints.GroupContradictionFound e: 
                    foreach (var groupId in e.groups)
                    {
                        Add(groupId, e);
                    } break;
                case FindPoints.GroupSelectedAsMostCompact e: Add(e.groupId, e); break;
            }
        }

        return dict.ToDictionary(keyVal => keyVal.Key, keyVal => keyVal.Value.ToArray());
    }

    private LoadTestTaskData CreateLoadTestTaskData(string path)
    {
        var loadTestTask = new LoadTestTaskData();

        var cancellationTokenSource = new CancellationTokenSource();
        var progress = new Progress<MathematicaRayLoader.LoadTestsProgress>((p) =>
        {
            loadTestTask.loadTestResultProgress = p;
        });
        var task = m_taskScheduler.EnqueueTask(() =>
        {
            return MathematicaRayLoader.LoadTestsResults(
                path,
                new[] { 0, 1, 2, 3 },
                cancellationTokenSource.Token,
                progress
            );
        }, cancellationTokenSource.Token);

        loadTestTask.loadTestResultTask = task;
        loadTestTask.loadTestResultCancellationToken = cancellationTokenSource;

        return loadTestTask;
    }

    private CalculateExperimentDataInProgressState CreateCalculateDataTask(MathematicaRayLoader.TestResult test, float treshold)
    {
        var cancellationTokenSource = new CancellationTokenSource();
        var task = m_taskScheduler.EnqueueTask(() =>
        {
            var events = new List<FindPoints.Event>();
            var points = FindPoints.FindBasedOnGroups(test.rays, treshold, events);
            var eventsArray = events.ToArray();
            var groupsInfo = GetPointInfo(eventsArray);
            var groupIdToPointIndex = new Dictionary<int, int>();
            var rayIndexToGroupId = new Dictionary<RayIndex, int>();

            for (var i = 0; i < points.Length; i++)
            {
                groupIdToPointIndex.Add(points[i].groupIndex, i);

                foreach (var rayIndex in points[i].pointIds)
                {
                    rayIndexToGroupId[rayIndex] = points[i].groupIndex;
                }
            }

            return new CalculateExperimentDataReadyState
            {
                treshold = treshold,
                events = events.ToArray(),
                points = points,
                groupIdToPointIndex = groupIdToPointIndex,
                rayIndexToGroupId = rayIndexToGroupId,
                groupEvents = groupsInfo
            };
        }, cancellationTokenSource.Token);

        return new CalculateExperimentDataInProgressState
        {
            task = task,
            calculateDataCancellationToken = cancellationTokenSource
        };
    }

    bool DrawExperimentsWindow(LoadedExperimentsInfoSate loadedState)
    {
        var experimentsWindowOpen = true;
        if (ImGui.Begin("Experiments"))
        {
            if (ImGui.Button("Add"))
            {
                ImGui.OpenPopup("Add experiment");
            }

            bool addExperimentOpen = true;
            if (ImGui.BeginPopupModal("Add experiment", ref addExperimentOpen, ImGuiWindowFlags.AlwaysAutoResize))
            {
                var windowState = loadedState.ExperimentsWindowState;

                if (ImGui.InputText("Path", ref windowState.currentPath, 100))
                {
                    if (!windowState.userRequestedCustomName)
                    {
                        windowState.currentName = Path.GetFileName(windowState.currentPath);
                    }

                    windowState.directoryExists = Directory.Exists(windowState.currentPath);
                }

                if (ImGui.InputText("Name", ref loadedState.ExperimentsWindowState.currentName, 100))
                {
                    loadedState.ExperimentsWindowState.userRequestedCustomName = true;
                }

                if (windowState.currentPath.Length != 0 && !windowState.directoryExists)
                {
                    ImGui.TextColored(Color.red, "Directory is not exist");
                }

                if (windowState.currentPath.Length == 0)
                {
                    ImGui.TextColored(Color.red, "Provide some path");
                }

                if (windowState.currentName.Length == 0)
                {
                    ImGui.TextColored(Color.red, "Provide some name");
                }

                if (windowState.directoryExists)
                {
                    if (ImGui.Button("Load"))
                    {
                        var experimentsInfo = new ExperimentInfo
                        {
                            Path = loadedState.ExperimentsWindowState.currentPath,
                            Name = Path.GetFileName(loadedState.ExperimentsWindowState.currentPath)
                        };
                        loadedState.AddExperiment(experimentsInfo);
                        ImGui.CloseCurrentPopup();
                    }

                    ImGui.SameLine();
                }
                if (ImGui.Button("Cancel"))
                {
                    ImGui.CloseCurrentPopup();
                }

                ImGui.EndPopup();
            }

            for (var experimentId = 0; experimentId < loadedState.ExperimentsCount; experimentId++)
            {
                ImGui.Separator();
                ImGui.PushID(experimentId);

                var experiment = loadedState.GetExperimentDataState(experimentId);

                ImGui.TextColored(loadedState.SelectedExperiment == experimentId ? Color.white : Color.gray, experiment.info.Name);
                var experimentIsSelected = loadedState.SelectedExperiment == experimentId;

                ImGui.SameLine();
                ImGui.Checkbox("Selected", ref experimentIsSelected);

                if (experimentIsSelected)
                {
                    loadedState.SelectedExperiment = experimentId;
                }

                ImGui.SameLine();
                if (ImGui.Button("Delete"))
                {
                    loadedState.ExperimentDeleteRequested = experimentId;
                    ImGui.OpenPopup("Delete experiment?");
                }

                bool deleteExperimentOpen = true;
                if (ImGui.BeginPopupModal("Delete experiment?", ref deleteExperimentOpen, ImGuiWindowFlags.AlwaysAutoResize)) {
                    ImGui.Text("Do you really want to delete this experiment? ");
                    ImGui.Text($"{loadedState.GetExperimentDataState(loadedState.ExperimentDeleteRequested.Value).info.Name}");
                    if (ImGui.Button("Delete")) {
                        loadedState.RemoveExperiment(loadedState.ExperimentDeleteRequested.Value);
                        loadedState.ExperimentDeleteRequested = null;
                        ImGui.CloseCurrentPopup();
                    }
                    ImGui.SameLine();
                    if (ImGui.Button("Cancel")) {
                        loadedState.ExperimentDeleteRequested = null;
                        ImGui.CloseCurrentPopup();
                    }

                    ImGui.EndPopup();
                }

                ImGui.SameLine();
                var autoload = experiment.info.AutoLoad;
                if (ImGui.Checkbox("Autoload", ref autoload))
                {
                    experiment.info.AutoLoad = autoload;
                    loadedState.MarkSettingsDirty();
                }

                switch (experiment.data.rawData)
                {
                    case null:
                        {
                            if (experiment.loadTestTask == null)
                            {
                                if (experiment.info.AutoLoad)
                                {
                                    experiment.loadTestTask = CreateLoadTestTaskData(experiment.info.Path);
                                }
                                else
                                {
                                    ImGui.Text("Raw data status: ");
                                    ImGui.SameLine();
                                    ImGui.TextDisabled("Unavailable");
                                    ImGui.SameLine();
                                    if (ImGui.Button("Load"))
                                    {
                                        experiment.loadTestTask = CreateLoadTestTaskData(experiment.info.Path);
                                    }
                                }
                            }
                            else
                            {
                                if (!experiment.loadTestTask.loadTestResultTask.IsCompleted)
                                {
                                    ImGui.Text("Raw data status: ");
                                    ImGui.SameLine();
                                    ImGui.TextColored(new Color(1.0f, 242.0f / 255.0f, 128.0f / 255.0f),
                                        $"Loading... {experiment.loadTestTask.loadTestResultProgress.loaded}/{experiment.loadTestTask.loadTestResultProgress.count}");

                                    ImGui.ProgressBar(experiment.loadTestTask.loadTestResultProgress.loaded * 1.0f /
                                                      experiment.loadTestTask.loadTestResultProgress.count, new Vector2(0, 0), "");

                                    ImGui.SameLine();

                                    if (ImGui.Button("Cancel"))
                                    {
                                        experiment.loadTestTask.loadTestResultCancellationToken.Cancel();
                                        experiment.loadTestTask = null;
                                    }
                                }
                            }
                            break;
                        }
                    case RawExperimentDataLoadError err:
                        {
                            ImGui.Text("Raw data status: ");
                            ImGui.SameLine();
                            ImGui.TextColored(Color.red, "Error");
                            ImGui.SameLine();
                            if (ImGui.Button("Reload"))
                            {
                                experiment.loadTestTask = CreateLoadTestTaskData(experiment.info.Path);
                                experiment.data.rawData = null;
                            }

                            ImGui.Text(err.exception.ToString());
                            break;
                        }
                    case RawExperimentData raw:
                        {
                            ImGui.Text("Raw data status: ");
                            ImGui.SameLine();
                            ImGui.TextColored(Color.green, "Loaded");
                            ImGui.SameLine();
                            if (ImGui.Button("Reload"))
                            {
                                experiment.loadTestTask = CreateLoadTestTaskData(experiment.info.Path);
                                experiment.data.rawData = null;
                            }

                            ImGui.Text("Calculation data status: ");
                            var calculatedTestsCount = 0;
                            var tasksInProgressCount = 0;
                            foreach (var calcDataState in experiment.data.calculatedData)
                            {
                                switch (calcDataState)
                                {
                                    case CalculateExperimentDataReadyState: calculatedTestsCount++; break;
                                    case CalculateExperimentDataInProgressState progressState:
                                        {
                                            tasksInProgressCount++;
                                            break;
                                        }
                                }
                            }

                            if (calculatedTestsCount == raw.tests.Length)
                            {
                                ImGui.SameLine();
                                ImGui.TextColored(Color.green, "Fully completed");
                            }
                            else
                            {
                                if (tasksInProgressCount > 0)
                                {
                                    ImGui.SameLine();
                                    ImGui.TextColored(Color.yellow, $"In progress {calculatedTestsCount}/{calculatedTestsCount + tasksInProgressCount}");
                                    ImGui.SameLine();
                                    if (ImGui.Button("Stop"))
                                    {
                                        for (var i = 0; i < experiment.data.calculatedData.Length; i++)
                                        {
                                            if (experiment.data.calculatedData[i] is CalculateExperimentDataInProgressState progressState)
                                            {
                                                progressState.calculateDataCancellationToken.Cancel();
                                            }
                                            experiment.data.calculatedData[i] = null;
                                        }
                                    }
                                    ImGui.ProgressBar((float)calculatedTestsCount / (calculatedTestsCount + tasksInProgressCount));
                                }
                                else
                                {
                                    if (calculatedTestsCount > 0)
                                    {
                                        ImGui.SameLine();
                                        ImGui.TextColored(Color.green,
                                            $"Partially completed {calculatedTestsCount}/{raw.tests.Length}");
                                        ImGui.SameLine();
                                        if (ImGui.Button("Continue"))
                                        {
                                            for (var i = 0; i < experiment.data.calculatedData.Length; i++)
                                            {
                                                if (experiment.data.calculatedData[i] == null)
                                                {
                                                    experiment.data.calculatedData[i] =
                                                        CreateCalculateDataTask(raw.tests[i], loadedState.Treshold);
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        ImGui.SameLine();
                                        ImGui.TextColored(Color.gray, "Unavailable");
                                        ImGui.SameLine();
                                        if (ImGui.Button("Start"))
                                        {
                                            for (var i = 0; i < experiment.data.calculatedData.Length; i++)
                                            {
                                                experiment.data.calculatedData[i] =
                                                    CreateCalculateDataTask(raw.tests[i], loadedState.Treshold);
                                            }
                                        }
                                    }
                                }
                            }


                            break;
                        }
                }

                ImGui.PopID();
            }

            ImGui.Separator();
        }
        ImGui.End();
        return experimentsWindowOpen;
    }

    void DrawPhysicalCameraImages(CameraInfosLoadedState loadedState, PhysicalCameraCache cameraCache)
    {
        if (ImGui.Begin("Cameras Stream"))
        {
            if (cameraCache.cameraTextures == null ||
                cameraCache.cameraTextures.Length != loadedState.cameraInfos.Count)
            {
                cameraCache.cameraTextures = new NamedCameraTexture[loadedState.cameraInfos.Count];
            }

            for (var i = 0; i < loadedState.cameraInfos.Count; i++)
            {
                cameraCache.cameraTextures[i] ??= new NamedCameraTexture { };
            }

            CameraImageGetter.GetTextures(cameraCache.cameraTextures);

            for (var id = 0; id < loadedState.cameraInfos.Count; id++)
            {
                var cameraInfo = loadedState.cameraInfos[id];
                ImGui.PushID(id);
                ImGui.BeginGroup();
                var state = CameraImageGetter.GetCameraState(cameraInfo.uniqueId);

                var deviceId = cameraInfo.deviceId.GetValueOrDefault(0);

                void startCapture () {

                    if (ImGui.InputInt("Device id", ref deviceId)) {
                        cameraInfo.deviceId = deviceId;
                        loadedState.isDirty = true;
                    }

                    if (ImGui.Button("Start capture")) {
                        cameraInfo.deviceId = deviceId;
                        loadedState.isDirty = true;
                        cameraCache.cameraTextures[id].id = cameraInfo.deviceId;
                        CameraImageGetter.StartCameraCapture(deviceId, cameraInfo.exposure);
                    }
                };

                switch (state)
                {
                    case null: {
                        ImGui.TextUnformatted("Record not started");
                        startCapture();
                    } break;
                    case CameraState.Stopped: {
                        ImGui.TextUnformatted("Record stopped");
                        startCapture();
                    } break;
                    case CameraState.StartCapturing: {
                        ImGui.TextColored(Color.yellow, "Capturing is initializing");
                    } break;
                    case CameraState.Capturing capturing:
                    {
                        if (cameraCache.cameraTextures[id] is { image: { } } texture) {
                            ImGui.Image(UImGuiUtility.GetTextureId(texture.image),
                                new Vector2(texture.image.width, texture.image.height) / 2, new Vector2(0, 1), new Vector2(1, 0));
                        }

                        if (ImGui.Button("Stop capture")) {
                            CameraImageGetter.StopCameraCapture(cameraInfo.deviceId.Value);
                        }

                        if (ImGui.SliderInt("Exposure", ref cameraInfo.exposure, -5, 0)) {
                            CameraImageGetter.SetCameraExposure(cameraInfo.deviceId.Value, cameraInfo.exposure);
                            loadedState.isDirty = true;
                        }
                    } break;
                    case CameraState.Stopping: {
                        ImGui.TextColored(Color.magenta, "Capturing is stopping");
                    } break;
                    case CameraState.Error error:
                    {
                        ImGui.TextColored(Color.red, "Error");
                        ImGui.TextUnformatted(error.message);
                        startCapture();
                    } break;
                }

                var preview = "N/A";
                if (cameraInfo.selectedResolution is { } resolutionId)
                {
                    var resolution = cameraInfo.availableResolutions[resolutionId];
                    preview = $"{resolution.x}x{resolution.y}";
                }

                if (ImGui.BeginCombo("Resolution", preview))
                {
                    for (var i = 0; i < cameraInfo.availableResolutions.Count; i++)
                    {
                        var resolution = cameraInfo.availableResolutions[i];
                        if (ImGui.Selectable($"{resolution.x}x{resolution.y}", i == cameraInfo.selectedResolution))
                        {
                            cameraInfo.selectedResolution = i;
                            loadedState.isDirty = true;
                        }
                    }

                    ImGui.EndCombo();
                }

                ImGui.EndGroup();
                ImGui.PopID();
            }
        }

        ImGui.End();
    }

    bool DrawTestsWindow(LoadedExperimentsInfoSate loadedState)
    {
        bool testsWindowOpen = true;
        if (ImGui.Begin("Tests", ref testsWindowOpen))
        {

            if (loadedState.SelectedExperiment == null)
            {
                ImGui.Text("Please select any experiment");
            }
            else
            {
                var experiment = loadedState.GetExperimentDataState(loadedState.SelectedExperiment.Value);

                switch (experiment.data.rawData)
                {
                    case null:
                        ImGui.Text("N/A");
                        break;
                    case RawExperimentDataLoadError:
                        ImGui.Text("Error");
                        break;
                    case RawExperimentData rawData:
                    {
                        if (experiment.data.calculatedData == null)
                        {
                            ImGui.Text("Data is not calculated");
                        }
                        else
                        {
                            var allSelected = experiment.selectedTests.Count == rawData.tests.Length;
                            if (ImGui.Checkbox("All", ref allSelected))
                            {
                                if (allSelected)
                                {
                                    for (var i = 0; i < rawData.tests.Length; i++)
                                    {
                                        experiment.selectedTests.Add(i);
                                    }
                                }
                                else
                                {
                                    experiment.selectedTests.Clear();
                                }
                            }

                            ImGui.Separator();

                            var xBoundaryWidth = ImGui.GetWindowPos().x + ImGui.GetWindowContentRegionWidth();
                            var size = new Vector2(30, 30);

                            var style = ImGui.GetStyle();

                            for (var testId = 0; testId < rawData.tests.Length; testId++)
                            {
                                ImGui.PushID(testId);
                                bool selected = experiment.selectedTests.Contains(testId);

                                ImGui.PushStyleVar(ImGuiStyleVar.SelectableTextAlign, new Vector2(0.5f, 0.5f));

                                switch (experiment.data.calculatedData[testId])
                                {
                                    case null:
                                    {
                                        ImGui.PushStyleColor(ImGuiCol.Button, Color.gray);

                                        if (ImGui.Button($"{testId}", size))
                                        {
                                            experiment.data.calculatedData[testId] =
                                                CreateCalculateDataTask(rawData.tests[testId], loadedState.Treshold);
                                        }

                                        ImGui.PopStyleColor();

                                        break;
                                    }
                                    case CalculateExperimentDataReadyState:
                                    {
                                        var stylePushed = false;
                                        if (experiment.selectedGroup is { } groupId && groupId.testId == testId)
                                        {
                                            stylePushed = true;
                                            ImGui.PushStyleColor(ImGuiCol.Header, Color.green);
                                        }
                                        if (ImGui.Selectable($"{testId}", selected, ImGuiSelectableFlags.None, size))
                                        {
                                            if (!selected)
                                            {
                                                experiment.selectedTests.Add(testId);
                                            }
                                            else
                                            {
                                                experiment.selectedTests.Remove(testId);
                                            }
                                        }

                                        if (stylePushed)
                                        {
                                            ImGui.PopStyleColor();
                                        }
                                        
                                        break;
                                    }
                                    case CalculateExperimentDataInProgressState progressState:
                                    {
                                        var stylePushed = false;
                                        if (progressState.task.Status == TaskStatus.WaitingToRun)
                                        {
                                            ImGui.PushStyleColor(ImGuiCol.Button, Color.white);
                                            stylePushed = true;
                                        } else if (progressState.task.Status == TaskStatus.Running)
                                        {
                                            ImGui.PushStyleColor(ImGuiCol.Button, Color.yellow);
                                            stylePushed = true;
                                        }

                                        if (ImGui.Button($"{testId}", size))
                                        {
                                            progressState.calculateDataCancellationToken.Cancel();
                                            experiment.data.calculatedData[testId] = null;
                                        }

                                        if (stylePushed)
                                        {
                                            ImGui.PopStyleColor();
                                        }

                                        break;
                                    }
                                }

                                ImGui.PopStyleVar();

                                var lastButtonX = ImGui.GetItemRectMax().x;
                                var nextButtonX = lastButtonX + style.ItemSpacing.x + size.x;

                                if (nextButtonX < xBoundaryWidth)
                                {
                                    ImGui.SameLine();
                                }

                                ImGui.PopID();
                            }

                        }
                        break;
                    }
                }
            }
        }
        ImGui.End();
        return testsWindowOpen;
    }

    void DrawPointHistory(FindPoints.Event[] events, int groupId)
    {
        ImGui.Text($"History of group: {groupId}");
        ImGui.Separator();
        var currentGroupId = -1;

        void DrawHeader(int id)
        {
            if (currentGroupId != id)
            {
                ImGui.Text($"Begin group {id}");
                ImGui.Separator();
                currentGroupId = id;
            }
        }

        foreach (var infoEvent in events)
        {
            switch (infoEvent)
            {
                case FindPoints.RayIntersectionEvent e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text($"Ray ({e.ray1Id.camId}:{e.ray1Id.rayId}) and ({e.ray2Id.camId}:{e.ray2Id.rayId}) has been intersected");
                        break;
                    }
                case FindPoints.GroupMergeEvent e:
                    {
                        DrawHeader(e.groupId);
                        if (e.groupId == groupId)
                        {
                            ImGui.Text($"Group has been created by merging {e.group1Id} and {e.group2Id}");
                        }
                        else
                        {
                            ImGui.Text($"Group has been merged to {e.groupId}");
                        }

                        break;
                    }
                case FindPoints.RayDeleteEvent e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text($"Ray ({e.rayId.camId}.{e.rayId.rayId}) has been removed");
                        break;
                    }
                case FindPoints.GroupDeletedDueToRepeatEvent e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text("Group deleted due to repeat");
                        break;
                    }
                case FindPoints.GroupRegisteredAsMax e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text("Group registered as max");
                        break;
                    }
                case FindPoints.GroupIsUncotradicted e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text("Group is uncontradicted");
                        break;
                    }
                case FindPoints.GroupContradictionFound e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text($"Group is contradicted with [{String.Join(',', e.groups.Select(x => x.ToString()))}]");
                        break;
                    }
                case FindPoints.GroupSelectedAsMostCompact e:
                    {
                        DrawHeader(e.groupId);
                        ImGui.Text($"Group selected as most compact");
                        break;
                    }
            }

            ImGui.Separator();
        }
    }

    void DrawRayInfoWindow(ExperimentDataState experiment, CalculateExperimentDataReadyState readyState, RayId ray)
    {
        if (ImGui.Begin("Ray Info"))
        {
            ImGui.Text($"Ray: {ray.index.camId}:{ray.index.rayId}");

            ImGui.Separator();
        }

        ImGui.End();
    }

    void DrawPointInfo(ExperimentDataState experiment, CalculateExperimentDataReadyState readyState, GroupId groupId)
    {
        var point = readyState.points[readyState.groupIdToPointIndex[groupId.groupIdInTest]];

        ImGui.Text($"Group id: {groupId.groupIdInTest}");
        ImGui.Text($"Score: {point.score}");

        if (ImGui.CollapsingHeader("Rays", ImGuiTreeNodeFlags.DefaultOpen))
        {
            foreach (var pointId in point.pointIds)
            {
                if (ImGui.Button($"{pointId.camId}:{pointId.rayId}"))
                {
                    experiment.selectedRay = new RayId(groupId.testId, pointId);
                }
            }
        }

        if (ImGui.CollapsingHeader("Images", ImGuiTreeNodeFlags.DefaultOpen))
        {
        }
    }

    bool DrawPointInfoWindow(LoadedExperimentsInfoSate loadedState)
    {
        var open = true; 
        if (ImGui.Begin("Point info", ref open))
        {
            if (loadedState.SelectedExperiment == null)
            {
                ImGui.Text("Please select any experiment and group");
            }
            else
            {
                var experiment = loadedState.GetExperimentDataState(loadedState.SelectedExperiment.Value);

                if (experiment.selectedGroup == null)
                {
                    ImGui.Text("Please select any group");
                }
                else if (experiment.data.calculatedData[experiment.selectedGroup.Value.testId] is not CalculateExperimentDataReadyState readyState)
                {
                    ImGui.TextColored(Color.red, "ERROR: It seems that there are some data corruption, try to reevaluate experiment");
                }
                else
                {
                    var groupId = experiment.selectedGroup.Value;
                    var events = readyState.groupEvents[groupId.groupIdInTest];
                    var point = readyState.points[readyState.groupIdToPointIndex[groupId.groupIdInTest]];

                    if (ImGui.BeginTabBar("##info"))
                    {
                        if (ImGui.BeginTabItem("History"))
                        {
                            DrawPointHistory(events, groupId.groupIdInTest);
                            ImGui.EndTabItem();
                        }

                        if (ImGui.BeginTabItem("Info"))
                        {
                            DrawPointInfo(experiment, readyState, groupId);
                            ImGui.EndTabItem();
                        }
                        ImGui.EndTabBar();
                    }
                }
            }
        }
        ImGui.End();
        return open;
    }

    void DrawCameraSelectionState(LoadedExperimentsInfoSate state)
    {
        if (state.SelectedExperiment is null)
        {
            return;
        }

        var experiment = state.GetExperimentDataState(state.SelectedExperiment.Value);

        if (experiment.cameraSelectionStates is null)
        {
            return;
        }

        var cameraSelectionState = experiment.cameraSelectionStates;

        var btnSize = new Vector2(20, 20);

        if (ImGui.Begin("Cameras"))
        {
            ImGui.PushStyleVar(ImGuiStyleVar.SelectableTextAlign, new Vector2(0.5f, 0.5f));

            if (ImGui.Button("Reset camera up"))
            {
                var newRotation = Quaternion.LookRotation(Camera.main.transform.rotation * Vector3.forward);
                Camera.main.transform.rotation = newRotation;
            }

            {
                var drawFrustum = experiment.cameraSelectionStates.Any(x => x.drawFrustum);
                if (ImGui.Selectable("F", ref drawFrustum, ImGuiSelectableFlags.None, btnSize))
                {
                    for (var i = 0; i < experiment.cameraSelectionStates.Length; i++)
                    {
                        experiment.cameraSelectionStates[i].drawFrustum = drawFrustum;
                    }
                }

                ImGui.SameLine();

                var drawImagePlane = experiment.cameraSelectionStates.Any(x => x.drawImagePlane);
                if (ImGui.Selectable("I", ref drawImagePlane, ImGuiSelectableFlags.None, btnSize))
                {
                    for (var i = 0; i < experiment.cameraSelectionStates.Length; i++)
                    {
                        experiment.cameraSelectionStates[i].drawImagePlane = drawImagePlane;
                    }
                }

                ImGui.SameLine();

                var linkFrustumSize = state.LinkFrustumSize;
                ImGui.Selectable("L", ref linkFrustumSize, ImGuiSelectableFlags.None, btnSize);
                state.LinkFrustumSize = linkFrustumSize;

                ImGui.SameLine();

                var frustumSize = state.FrustumSize;
                ImGui.SetNextItemWidth(-0.01f);
                ImGui.SliderFloat("frustumSize", ref frustumSize, 0.1f, 5.0f);
                state.FrustumSize = frustumSize;
            }

            for (var camId = 0; camId < cameraSelectionState.Length; camId++)
            {
                ImGui.Separator();

                ImGui.PushID(camId);

                var drawFrustum = cameraSelectionState[camId].drawFrustum;
                ImGui.Selectable("F", ref drawFrustum, ImGuiSelectableFlags.None, btnSize);
                cameraSelectionState[camId].drawFrustum = drawFrustum;

                ImGui.SameLine();

                var drawImagePlane = cameraSelectionState[camId].drawImagePlane;
                ImGui.Selectable("I", ref drawImagePlane, ImGuiSelectableFlags.None, btnSize);
                cameraSelectionState[camId].drawImagePlane = drawImagePlane;

                ImGui.SameLine();

                if (ImGui.Button("V", btnSize))
                {
                    if (experiment.data.rawData is RawExperimentData rawData)
                    {
                        var transformMatrix = rawData.cameras[camId].inverseTransformMatrix;
                        Camera.main.transform.position = transformMatrix.GetPosition();
                        Camera.main.transform.rotation = transformMatrix.rotation;
                    }
                }

                ImGui.SameLine(/*(btnSize.x + ImGui.GetStyle().WindowPadding.x) * 3*/);

                var frustumSize = state.LinkFrustumSize ? state.FrustumSize : cameraSelectionState[camId].frustumSizeMultiplier;
                ImGui.SetNextItemWidth(-0.01f);
                ImGui.SliderFloat("frustumSize", ref frustumSize, 0.1f, 5.0f);

                if (state.LinkFrustumSize)
                {
                    state.FrustumSize = frustumSize;
                }
                else
                {
                    cameraSelectionState[camId].frustumSizeMultiplier = frustumSize;
                }

                ImGui.PopID();
            }

            ImGui.PopStyleVar();
        }
        ImGui.End();
    }

    void DrawFilterWidgets(LoadedExperimentsInfoSate state)
    {
        var filterData = state.FilterData;

        ImGui.Checkbox("Filter By Score", ref filterData.EnableFilterByScore);

        if (filterData.EnableFilterByScore)
        {
            if (ImGui.DragFloat("Min score", ref filterData.MinScore, 0.0001f, 0, state.Treshold))
            {
                if (filterData.MinScore > filterData.MaxScore)
                {
                    filterData.MaxScore = filterData.MinScore;
                }
            }

            if (ImGui.DragFloat("Max score", ref filterData.MaxScore, 0.0001f, 0, state.Treshold))
            {
                if (filterData.MaxScore < filterData.MinScore)
                {
                    filterData.MinScore = filterData.MaxScore;
                }
            }
        }

        ImGui.Checkbox("Filter By Ray Count", ref filterData.EnableFilterByRayCount);

        if (filterData.EnableFilterByRayCount)
        {
            if (ImGui.SliderInt("Min Ray Count", ref filterData.MinRayCount, 2, 4))
            {
                if (filterData.MinRayCount > filterData.MaxRayCount)
                {
                    filterData.MaxRayCount = filterData.MinRayCount;
                }
            }

            if (ImGui.SliderInt("Max Ray Count", ref filterData.MaxRayCount, 2, 4))
            {
                if (filterData.MaxRayCount < filterData.MinRayCount)
                {
                    filterData.MinRayCount = filterData.MaxRayCount;
                }
            }
        }

        state.FilterData = filterData;
    }

    void DrawToolBar(LoadedExperimentsInfoSate loadedState)
    {
        var buttonSize = new Vector2(30, 30);

        ImGui.SetNextWindowPos(new Vector2(10, 35));
        ImGui.PushStyleVar(ImGuiStyleVar.WindowRounding, 0.0f);
        if (ImGui.Begin("ToolBar", ImGuiWindowFlags.AlwaysAutoResize | ImGuiWindowFlags.NoDocking | ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoTitleBar))
        {
            ImGui.PushStyleVar(ImGuiStyleVar.SelectableTextAlign, new Vector2(0.5f, 0.5f));

            if (ImGui.Selectable("Ray", loadedState.DrawRays, ImGuiSelectableFlags.None, buttonSize))
            {
                loadedState.DrawRays = !loadedState.DrawRays;
            }

            ImGui.SameLine();

            if (ImGui.Selectable("DOSP", loadedState.DrawOnlySelectedPoint, ImGuiSelectableFlags.None, buttonSize))
            {
                loadedState.DrawOnlySelectedPoint = !loadedState.DrawOnlySelectedPoint;
            }

            ImGui.PopStyleVar();
        }
        ImGui.PopStyleVar();
        ImGui.End();
    }

    TaskToken<List<CameraInfo>> CreateLoadCameraInfoTask(string path)
    {
        return m_taskScheduler.EnqueueTask(() =>
        {
            var text = File.ReadAllText(path);
            return JsonConvert.DeserializeObject<List<CameraInfo>>(text);
        });
    }

    void UpdateState(LoadedExperimentsInfoSate loadedState)
    {
        if (loadedState.CameraInfos is null && loadedState.Settings.CamerasInfoPath is not null && loadedState.Settings.CamerasInfoPath.Length > 0)
        {
            loadedState.CameraInfos = new CameraInfosLoadingState
            {
                taskToken = CreateLoadCameraInfoTask(loadedState.Settings.CamerasInfoPath)
            };
        }

        if (loadedState.CameraInfos is CameraInfosLoadingState loadingState)
        {
            if (loadingState.taskToken.IsCompleted)
            {
                try
                {
                    var cameraInfos = loadingState.taskToken.Result;

                    loadedState.CameraInfos = new CameraInfosLoadedState
                    {
                        cameraInfos = cameraInfos
                    };
                }
                catch (Exception ex)
                {
                    loadedState.CameraInfos = new CameraInfosLoadErrorState
                    {
                        error = ex.Message
                    };
                }
            }
        }

        for (var experimentId = 0; experimentId < loadedState.ExperimentsCount; experimentId++)
        {
            var experiment = loadedState.GetExperimentDataState(experimentId);

            if (experiment.loadTestTask != null && experiment.loadTestTask.loadTestResultTask.IsCompleted)
            {
                try
                {
                    var data = experiment.loadTestTask.loadTestResultTask.Result;

                    experiment.data.rawData = new RawExperimentData
                        { cameras = data.cameras, tests = data.tests.ToArray() };
                    experiment.data.calculatedData = new CalculateExperimentDataState[data.tests.Count];
                    experiment.cameraSelectionStates = new CameraSelectionState[data.cameras.Length];

                    for (var i = 0; i < experiment.cameraSelectionStates.Length; i++)
                    {
                        experiment.cameraSelectionStates[i].frustumSizeMultiplier = 1;
                    }
                }
                catch (Exception e)
                {
                    experiment.data.rawData = new RawExperimentDataLoadError { exception = e };
                }

                experiment.loadTestTask = null;
            }

            if (experiment.data.calculatedData == null)
            {
                continue;
            }

            for (var testId = 0; testId < experiment.data.calculatedData.Length; testId++)
            {
                switch (experiment.data.calculatedData[testId])
                {
                    case CalculateExperimentDataInProgressState progressState:
                    {
                        if (progressState.task.IsCompleted)
                        {
                            experiment.data.calculatedData[testId] = progressState.task.Result;
                        }
                        break;
                    }
                }
            }
        }
    }

    void UpdateVisualCache(LoadedExperimentsInfoSate loadedState, VisualCache cache)
    {
        if (cache.cameraImages is not { } cameraImages)
        {
            return;
        }

        for (var i = 0; i < cameraImages.Length; i++)
        {
            if (cache.cameraImages[i] is CameraImageRawLoadedState rawLoaded)
            {
                var pointsImageTexture = new Texture2D(2, 2);
                var objectImageTexture = new Texture2D(2, 2);

                pointsImageTexture.LoadImage(rawLoaded.pointsImage);
                objectImageTexture.LoadImage(rawLoaded.objectImage);

                cache.cameraImages[i] = new CameraImageReadyState
                {
                    pointsImage = pointsImageTexture,
                    objectImage = objectImageTexture
                };
            }
        }

        if (loadedState.SelectedExperiment is not { } selectedExperiment)
        {
            return;
        }

        var experiment = loadedState.GetExperimentDataState(selectedExperiment);

        //if (experiment.selectedGroup is { } selectedGroup)
        //{
        //    if (cache.selectedGroupId != experiment.selectedGroup)
        //    {
        //        for (var i = 0; i < cache.windowCameraImages.Length; i++)
        //        {
        //            cache.windowCameraImages[i].cvPointsImage = 
        //        }
        //    }
        //}
    }

    private static uint ColorToUInt(Color color)
    {
        var color32 = (Color32)color;
        return (uint)((color32.a << 24) | (color32.b << 16) |
                      (color32.g << 8) | (color32.r << 0));
    }

    bool DrawCameraImages(LoadedExperimentsInfoSate loadedState, VisualCache cache)
    {
        var open = true;
        if (ImGui.Begin("Camera images", ref open))
        {
            if (loadedState.SelectedExperiment is not { } selectedExperiment)
            {
                return open;
            }

            if (cache.cameraImages is null)
            {
                return open;
            }

            var drawList = ImGui.GetWindowDrawList();

            var experiment = loadedState.GetExperimentDataState(selectedExperiment);

            var cameraImageSizeScale = loadedState.CameraImageSizeScale;
            ImGui.SliderFloat("Image size", ref cameraImageSizeScale, 0.1f, 1.0f);
            loadedState.CameraImageSizeScale = cameraImageSizeScale;

            var xBoundaryWidth = ImGui.GetWindowContentRegionWidth();
            var style = ImGui.GetStyle();

            var cameraImageSize = new Vector2(1280, 960);

            if (experiment.data.rawData is RawExperimentData rawData)
            {
                for (var camId = 0; camId < rawData.cameras.Length; camId++)
                {
                    ImGui.PushID(camId);
                    if (cache.cameraImages[camId] is not CameraImageReadyState imageReady)
                    {
                        continue;
                    }

                    var windowCursorPos = ImGui.GetCursorPos();
                    var screenCursorPos = ImGui.GetCursorScreenPos();

                    var drawCameraImageSize = cameraImageSize * cameraImageSizeScale;

                    var startCursorPos = ImGui.GetCursorPos();

                    ImGui.Image(UImGuiUtility.GetTextureId(imageReady.pointsImage), drawCameraImageSize, new Vector2(0, 1), new Vector2(1, 0));

                    var endCursorPos = ImGui.GetCursorPos();

                    if (experiment.selectedGroup is not { } selectedGroup)
                    {
                        continue;
                    }

                    if (experiment.data.calculatedData[selectedGroup.testId] is
                        CalculateExperimentDataReadyState readyState)
                    {
                        for (var camPointIndex = 0; camPointIndex < rawData.tests[selectedGroup.testId].camPoints[camId].Count; camPointIndex++)
                        {
                            var localPointPos = (cameraImageSize * new Vector2(0, 1) + new Vector2(1, -1) * rawData.tests[selectedGroup.testId].camPoints[camId][camPointIndex]) *
                                            cameraImageSizeScale;

                            ImGui.SetCursorPos(windowCursorPos + localPointPos);

                            drawList.AddCircle(screenCursorPos + localPointPos, 10.0f * cameraImageSizeScale, ColorToUInt(Color.red), 12, 5);

                            var selected = false;
                            if (readyState.rayIndexToGroupId.TryGetValue(new RayIndex(camId, camPointIndex), out var groupIndex))
                            {
                                selected = readyState.points[readyState.groupIdToPointIndex[selectedGroup.groupIdInTest]].pointIds.Contains(new RayIndex(camId, camPointIndex));
                                if (ImGui.Button($"{camPointIndex}"))
                                {
                                    experiment.selectedGroup = new GroupId(selectedGroup.testId, groupIndex);
                                }
                            }
                            
                            drawList.AddCircleFilled(screenCursorPos + localPointPos, 10.0f * cameraImageSizeScale, ColorToUInt(selected ? Color.green : Color.red), 12);
                        }
                    }

                    var nextButtonX =  startCursorPos.x + drawCameraImageSize.x * 2;

                    if (nextButtonX < xBoundaryWidth)
                    {
                        ImGui.SetCursorPos(startCursorPos + new Vector2(drawCameraImageSize.x, 0));
                    }
                    else
                    {
                        ImGui.SetCursorPos(endCursorPos);
                    }
                }
            }
        }
        ImGui.End();

        return open;
    }

    void DrawScenesList(List<SimulatedScene> scenes, LoadedExperimentsInfoSate loadedState, SimulatedScenesWindowState state, SimulatedSceneVisualCache cache)
    {
        if (ImGui.Button("Reset selection"))
        {
            state.selectedScene = null;
            Destroy(cache.scene);
            cache.scene = null;
            state.sceneToScan = null;
        }

        if (state.sceneToScan is not null)
        {
            ImGui.DragFloat("Noise (px)", ref state.noise, 0.1f, 0.0f, 10.0f);
            if (ImGui.Button("Generate data with noise"))
            {
                var experimentData = state.sceneToScan.CreateRawExperimentData(state.noise);

                //foreach (var simulationResult in experimentData.result) {
                //    foreach (var point in simulationResult.originalPoints) {
                //        Instantiate(PointPrefab, point, Quaternion.identity);
                //    }
                //}

                loadedState.AddExperiment(experimentData.ToRawExperimentData(), scenes[state.selectedScene.Value].name);
            }

            if (ImGui.Button("Generate data")) {
                var experimentData = state.sceneToScan.CreateRawExperimentDataGroundTruthCamPositions(state.noise);

                loadedState.AddExperiment(experimentData.ToRawExperimentData(), scenes[state.selectedScene.Value].name);
            }
        }

        for (var index = 0; index < scenes.Count; index++)
        {
            var scene = scenes[index];
            var selected = index == state.selectedScene;
            if (ImGui.Selectable(scene.name, selected) && !selected)
            {
                cache.scene = Instantiate(scene.scene);

                state.sceneToScan = cache.scene.GetComponent<SceneToScan>();

                state.selectedScene = index;
            }
        }
    }

    void DrawSimulatedScenesWindow(List<SimulatedScene> scenes, LoadedExperimentsInfoSate loadedState, SimulatedScenesWindowState state, SimulatedSceneVisualCache cache) {
        if (ImGui.Begin("Simulated scenes"))
        {
            DrawScenesList(scenes, loadedState, state, cache);
            ImGui.Separator();
        }

        ImGui.End();
    }

    void DrawPhysicalCameraInfos(CameraInfosLoadedState loadedState, PhysicalCameraWindowState windowState)
    {
        if (ImGui.Button("Add physical camera"))
        {
            ImGui.OpenPopup("Add physical camera");
        }

        if (ImGui.BeginPopup("Add physical camera"))
        {
            var state = windowState.addPhysicalCamera;

            ImGui.InputText("Name", ref state.currentName, 50);

            ImGui.InputInt("Unique Id", ref state.uniqueId);

            ImGui.InputInt2("Resolution", ref state.resolution[0]);

            if (state.resolution.Any(x => x == 0))
            {
                ImGui.TextColored(Color.red, "Enter resolution");
            }
            else if (loadedState.cameraInfos.Any(x => x.uniqueId == state.uniqueId)) 
            {
                ImGui.TextColored(Color.red, "Id is not unique");
            }
            else
            {
                if (ImGui.Button("Add"))
                {
                    loadedState.cameraInfos.Add(new CameraInfo
                    {
                        uniqueId = state.uniqueId,
                        availableResolutions = new List<Vector2> { new (state.resolution[0], state.resolution[1]) },
                        calibrationInfos = new List<CalibrationInfo>(),
                        name = state.currentName
                    });
                    ImGui.CloseCurrentPopup();
                }
            }
            ImGui.EndPopup();
        }
            
        foreach (var cameraInfo in loadedState.cameraInfos)
        {
            ImGui.PushID(cameraInfo.uniqueId);

            ImGui.TextUnformatted($"{cameraInfo.name}, unique id: {cameraInfo.uniqueId}");

            if (ImGui.BeginPopup("Import calibration info"))
            {
                var importCalibState = windowState.importCalibInfo;

                if (ImGui.InputText("Path", ref importCalibState.currentImportPath, 200))
                {
                    importCalibState.calibMatrixFileExists = File.Exists($"{importCalibState.currentImportPath}Mtx.csv");
                    importCalibState.distCoeffsFileExists = File.Exists($"{importCalibState.currentImportPath}Dist.csv");
                    importCalibState.fileExists = File.Exists(importCalibState.currentImportPath);
                }

                if (importCalibState.currentImportPath.Length == 0) {
                    ImGui.TextColored(Color.red, "Provide some path");
                }
                else if ((!importCalibState.calibMatrixFileExists || !importCalibState.distCoeffsFileExists) && !importCalibState.fileExists) {
                    ImGui.TextColored(Color.red, "Some files does not exist");
                }
                else if (importCalibState.fileExists && !importCalibState.currentImportPath.EndsWith(".json")) {
                    ImGui.TextColored(Color.red, "File must have .csv or .json extension");
                }
                else if (!importCalibState.userRequestedCustomName) {
                    importCalibState.currentName = Path.GetFileNameWithoutExtension(importCalibState.currentImportPath);
                }

                ImGui.InputText("Name", ref importCalibState.currentName, 50);

                if (importCalibState.currentName.Length == 0) {
                    ImGui.TextColored(Color.red, "Provide some name");
                }
                else if (importCalibState.calibMatrixFileExists && importCalibState.distCoeffsFileExists) {
                    if (ImGui.Button("Import .csv"))
                    {
                        try
                        {
                            var calibMtx = MathematicaRayLoader.LoadMatrix($"{importCalibState.currentImportPath}Mtx.csv");
                            var distCoeffs = MathematicaRayLoader.LoadFloatArray($"{importCalibState.currentImportPath}Dist.csv");

                            cameraInfo.calibrationInfos.Add(new CalibrationInfo(DateTime.Now, importCalibState.currentName, importCalibState.chosenResolution, calibMtx, distCoeffs));
                            loadedState.isDirty = true;

                            ImGui.CloseCurrentPopup();
                        }
                        catch (Exception ex) {
                            importCalibState.loadErrorMessage = ex.Message;
                            ImGui.OpenPopup("Error load");
                        }
                    }
                    ImGui.SameLine();
                } else if (importCalibState.fileExists)
                {
                    if (ImGui.Button("Import .json")) {
                        try
                        {
                            var data = JsonConvert
                                .DeserializeObject<PhysicalCameraWindowState.PhysicalCameraCalibrationData>(
                                    importCalibState.currentImportPath);

                            var calibMtx = Matrix4x4.identity;

                            for (var x = 0; x < data.intrinsicMatrix.GetLength(0); x++)
                            {
                                for (var y = 0; y < data.intrinsicMatrix.GetLength(1); y++)
                                {
                                    calibMtx[x, y] = data.intrinsicMatrix[x, y];
                                }
                            }

                            cameraInfo.calibrationInfos.Add(new CalibrationInfo(DateTime.Now, importCalibState.currentName, importCalibState.chosenResolution, calibMtx, data.distCoeffs));
                            loadedState.isDirty = true;

                            ImGui.CloseCurrentPopup();
                        }
                        catch (Exception ex) {
                            importCalibState.loadErrorMessage = ex.Message;
                            ImGui.OpenPopup("Error load");
                        }
                    }
                    ImGui.SameLine();
                }

                if (ImGui.Button("Cancel"))
                {
                    ImGui.CloseCurrentPopup();
                }

                if (ImGui.BeginPopup("Error load"))
                {
                    ImGui.TextColored(Color.red, "Error during load");
                    ImGui.TextUnformatted(importCalibState.loadErrorMessage);

                    if (ImGui.Button("Shtosh"))
                    {
                        ImGui.CloseCurrentPopup();
                    }
                    ImGui.EndPopup();
                }

                ImGui.EndPopup();
            }

            if (ImGui.CollapsingHeader("Available Resolutions"))
            {
                foreach (var resolution in cameraInfo.availableResolutions)
                {
                    ImGui.TextUnformatted($"{resolution.x}x{resolution.y}");

                    if (ImGui.Button("Import calibration info"))
                    {
                        ImGui.OpenPopup("Import calibration info");
                        windowState.importCalibInfo.chosenResolution = resolution;
                    }

                    foreach (var calibrationInfo in cameraInfo.calibrationInfos.Where(x => x.resolution == resolution))
                    {
                        ImGui.TextUnformatted($"Name: {calibrationInfo.name}");
                        ImGui.TextUnformatted($"Date: {calibrationInfo.calibrationTime}");
                        ImGui.Separator();
                    }
                }
            }

            ImGui.PopID();
        }
    }

    bool DrawPhysicalCameraInfos(Settings settings, ref CameraInfosState cameraInfos, PhysicalCameraWindowState windowState)
    {
        bool open = true;
        if (ImGui.Begin("Physical Cameras"))
        {
            switch (cameraInfos)
            {
                case null:
                    var cameraInfoImportState = windowState.importCameraInfoState;
                    ImGui.InputText("Camera Info Path", ref cameraInfoImportState.cameraInfoPath, 200);

                    try
                    {
                        if (File.Exists(cameraInfoImportState.cameraInfoPath))
                        {
                            if (ImGui.Button("Import"))
                            {
                                settings.CamerasInfoPath = cameraInfoImportState.cameraInfoPath;
                                settings.isDirty = true;
                            }
                        }
                        else
                        {
                            ImGui.TextColored(Color.yellow, "File does not exist, create?");
                            if (ImGui.Button("Create"))
                            {
                                var text = JsonConvert.SerializeObject(new List<CameraInfo>());
                                File.WriteAllText(cameraInfoImportState.cameraInfoPath, text);
                                settings.CamerasInfoPath = cameraInfoImportState.cameraInfoPath;
                                settings.isDirty = true;
                            }
                        }
                    }
                    catch (Exception ex) {
                        cameraInfoImportState.importError = ex.Message;
                    }

                    if (cameraInfoImportState.importError is {} message)
                    {
                        ImGui.TextUnformatted("Import error:");
                        ImGui.TextColored(Color.red, message);
                    }
                    break;
                case CameraInfosLoadingState:
                    ImGui.TextColored(Color.yellow, "Info is currently loading");
                    break;
                case CameraInfosLoadErrorState errorState:
                {
                    if (ImGui.Button("Retry"))
                    {
                        cameraInfos = null;
                    }
                    if (ImGui.Button("Clear path")) {
                        cameraInfos = null;
                        settings.CamerasInfoPath = string.Empty;
                    }
                    ImGui.TextColored(Color.red, "Error during camera infos");
                    ImGui.TextUnformatted(errorState.error);
                }
                    break;
                case CameraInfosLoadedState loadedState:
                    DrawPhysicalCameraInfos(loadedState, windowState);
                    DrawPhysicalCameraImages(loadedState, m_physicalCameraCache);
                    break;
            }
        }

        ImGui.End();

        return open;
    }

    void OnLayout(UImGui.UImGui obj)
    {
        m_taskScheduler.Update();

        if (m_state is InitialState)
        {
            try
            {
                var experiments = Loader.LoadExperimentsInfo();
                m_state = new LoadedExperimentsInfoSate(experiments);
            }
            catch (Exception e)
            {
                m_state = new LoadExperimentsErrorState { error = e.ToString() };
            }
        }

        if (m_state is LoadExperimentsErrorState error)
        {
            if (ImGui.BeginPopupModal("Load error"))
            {
                ImGui.Text($"Save file is corrupted: {error.error}");

                if (ImGui.Button("Clear file"))
                {
                    var experiments = new Settings();
                    Loader.SaveExperimentsInfo(experiments);
                    m_state = new LoadedExperimentsInfoSate(experiments);
                    ImGui.CloseCurrentPopup();
                }
                ImGui.SameLine();
                if (ImGui.Button("Retry"))
                {
                    m_state = new InitialState();
                    ImGui.CloseCurrentPopup();
                }

                ImGui.EndPopup();
            }
        }

        if (m_state is LoadedExperimentsInfoSate loadedState)
        {
            DrawVisual(loadedState, m_visualCache, m_taskScheduler);

            DrawToolBar(loadedState);

            DrawCameraSelectionState(loadedState);

            if (loadedState.ExperimentsWindowState.Open)
            {
                loadedState.ExperimentsWindowState.Open = DrawExperimentsWindow(loadedState);
            }

            if (loadedState.TestsWindowsIsOpen)
            {
                loadedState.TestsWindowsIsOpen = DrawTestsWindow(loadedState);
            }

            if (loadedState.PointInfoWindowsIsOpen)
            {
                loadedState.PointInfoWindowsIsOpen = DrawPointInfoWindow(loadedState);
            }

            if (loadedState.CameraImagesIsOpen)
            {
                loadedState.CameraImagesIsOpen = DrawCameraImages(loadedState, m_visualCache);
            }

            if (loadedState.IsSettingsDirty)
            {
                Loader.SaveExperimentsInfo(loadedState.Settings);
                loadedState.MarkSettingsClean();
            }

            if (loadedState.PhysicalCameraWindowState.isOpen) {
                loadedState.PhysicalCameraWindowState.isOpen = DrawPhysicalCameraInfos(loadedState.Settings, ref loadedState.CameraInfos, loadedState.PhysicalCameraWindowState);
            }

            if (loadedState.CameraInfos is CameraInfosLoadedState { isDirty: true } infoLoaded)
            {
                File.WriteAllText(loadedState.Settings.CamerasInfoPath, JsonConvert.SerializeObject(infoLoaded.cameraInfos));
                infoLoaded.isDirty = false;

            }

            DrawSimulatedScenesWindow(Scenes, loadedState, m_simulatedScenesWindowState, m_simulatedSceneVisualCache);

            UpdateState(loadedState);
            UpdateVisualCache(loadedState, m_visualCache);
        }

        if (ImGui.BeginMainMenuBar()) {
            if (m_state is LoadedExperimentsInfoSate state)
            {
                if (ImGui.BeginMenu("Windows"))
                {
                    var drawExperimentsWindow = state.ExperimentsWindowState.Open;
                    ImGui.Checkbox("Experiments", ref drawExperimentsWindow);
                    state.ExperimentsWindowState.Open = drawExperimentsWindow;

                    var drawTestsWindow = state.TestsWindowsIsOpen;
                    ImGui.Checkbox("Tests", ref drawTestsWindow);
                    state.TestsWindowsIsOpen = drawTestsWindow;

                    var drawCameraImagesWindow = state.CameraImagesIsOpen;
                    ImGui.Checkbox("Camera images", ref drawCameraImagesWindow);
                    state.CameraImagesIsOpen = drawCameraImagesWindow;

                    ImGui.Checkbox("Physical cameras", ref state.PhysicalCameraWindowState.isOpen);

                    ImGui.EndMenu();
                }

                if (ImGui.MenuItem("Point Info"))
                {
                    state.PointInfoWindowsIsOpen = !state.PointInfoWindowsIsOpen;
                }

                if (ImGui.BeginMenu("View"))
                {
                    var drawRays = state.DrawRays;
                    ImGui.Checkbox("Draw rays", ref drawRays);
                    state.DrawRays = drawRays;

                    var rayLength = state.RayLength;
                    ImGui.DragFloat("Ray length", ref rayLength, 0.1f, 0.1f, 6.0f);
                    state.RayLength = rayLength;

                    var pointSize = state.PointSize;
                    ImGui.DragFloat("Point size", ref pointSize, 0.01f, 0.01f, 0.1f);
                    state.PointSize = pointSize;

                    if (ImGui.BeginCombo("Colorization", state.ColorizationType.ToString()))
                    {
                        foreach (var type in Enum.GetValues(typeof(ColorizationType)))
                        {
                            if (ImGui.Selectable(type.ToString(), (ColorizationType)type == state.ColorizationType))
                            {
                                state.ColorizationType = (ColorizationType)type;
                            }
                        }
                        ImGui.EndCombo();
                    }

                    DrawFilterWidgets(state);

                    ImGui.EndMenu();
                }

                if (ImGui.MenuItem("Exit"))
                {
                    Application.Quit();
                }
            }

            ImGui.EndMainMenuBar();
        }
    }
}
