using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Newtonsoft.Json;
using ImGuiNET;

class ExperimentInfo
{
    public string Name { get; set; }
    public string Path { get; set; }
    public bool AutoLoad { get; set; }
}

class Settings
{
    public Settings()
    {
        Experiments = new List<ExperimentInfo>();
    }

    public List<ExperimentInfo> Experiments { get; set; }
    public int? SelectedExperiment { get; set; }
    public bool ExperimentsWindowIsOpen { get; set; }
    public bool TestsWindowIsOpen { get; set; }
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

class RawExperimentDataState
{
}

class RawExperimentData : RawExperimentDataState
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
    public FindPoints.Event[] events;
    public FoundPoint[] points;
    public Dictionary<int, FindPoints.Event[]> groupEvents;
    public Dictionary<int, int> groupIdToPointIndex;
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

class ExperimentDataState
{
    public ExperimentInfo info;
    public ExperimentData data;
    public HashSet<int> selectedTests;
    public HashSet<RayId> selectedRays;
    public LoadTestTaskData loadTestTask;
    public GroupId? selectedGroup;
}

class LoadedExperimentsInfoSate : State
{
    public LoadedExperimentsInfoSate(Settings info)
    {
        m_settings = info;
        m_experiments = info.Experiments.Select(x => new ExperimentDataState { info = x, data = new ExperimentData(), selectedTests = new HashSet<int>(), selectedRays = new HashSet<RayId>()}).ToList();
        ExperimentsWindowState = new ExperimentsWindowState{  };
    }

    private Settings m_settings;
    private bool m_settingsIsDirty;
    private List<ExperimentDataState> m_experiments;

    public int ExperimentsCount => m_experiments.Count;

    public ExperimentDataState GetExperimentDataState(int index)
    {
        return m_experiments[index];
    }

    public void AddExperiment(ExperimentInfo info)
    {
        m_experiments.Add(new ExperimentDataState{ info = info, data = new ExperimentData() });
        m_settingsIsDirty = true;
    }

    public void RemoveExperiment(int index)
    {
        m_experiments.RemoveAt(index);
        m_settingsIsDirty = true;
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

    public bool IsSettingsDirty => m_settingsIsDirty;

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

class VisualCache
{
    public int? selectedExperiment;
    public Dictionary<GroupId, Visual.Point> pointObjects = new();
    public Dictionary<RayId, Visual.Ray> rayObjects = new();
    public GameObject pointPrefab;
    public GameObject rayPrefab;
    public GroupId? selectedGroupId;
}

public class Main : MonoBehaviour
{
    private State m_state = new InitialState();
    private TaskScheduler m_taskScheduler = new(8);
    private VisualCache m_visualCache;

    public GameObject PointPrefab;
    public GameObject RayPrefab;

    void Start()
    {
        m_visualCache = new VisualCache { pointPrefab = PointPrefab, rayPrefab = RayPrefab };
    }

    // Start is called before the first frame update
    void OnEnable()
    {
        ImGuiUn.Layout += OnLayout;
    }

    // Update is called once per frame
    void OnDisable()
    {
        ImGuiUn.Layout -= OnLayout;
    }

    static void DrawVisual(LoadedExperimentsInfoSate state, VisualCache cache)
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
        }

        cache.selectedExperiment = state.SelectedExperiment.Value;

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
                        pointComponent.onClick = (x) =>
                        {
                            cache.selectedGroupId = groupId;
                            experiment.selectedGroup = groupId;
                        };
                        cache.pointObjects.Add(groupId, pointComponent);
                    }
                    else
                    {
                        existedPointObject.Selected = cache.selectedGroupId != null &&
                                                      existedPointObject.groupId == cache.selectedGroupId.Value;
                    }
                }

                foreach (var selectedRay in experiment.selectedRays)
                {
                    if (!cache.rayObjects.ContainsKey(selectedRay))
                    {
                        var rayObject = Instantiate(cache.rayPrefab);
                        var rayComponent = rayObject.GetComponent<Visual.Ray>();
                        if (experiment.data.rawData is RawExperimentData rawData)
                        {
                            rayComponent.SetRay(
                                rawData.tests[selectedRay.testId].rays[selectedRay.index.camId][selectedRay.index.rayId].ray);

                            rayComponent.SetLength(2);
                        }
                        cache.rayObjects.Add(selectedRay, rayComponent);
                    }
                }
            }
        }
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

            for (var i = 0; i < points.Length; i++)
            {
                groupIdToPointIndex.Add(points[i].groupIndex, i);
            }

            return new CalculateExperimentDataReadyState
            {
                events = events.ToArray(),
                points = points,
                groupIdToPointIndex = groupIdToPointIndex,
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
        if (ImGui.Begin("Experiments", ref experimentsWindowOpen))
        {
            if (ImGui.Button("Add"))
            {
                ImGui.OpenPopup("Add experiment");
            }

            bool open = true;
            if (ImGui.BeginPopupModal("Add experiment", ref open, ImGuiWindowFlags.AlwaysAutoResize))
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
                    ImGui.TextColored(Color.red, "Directory is not existed");
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

            if (loadedState.ExperimentDeleteRequested != null && ImGui.BeginPopupModal("Delete experiment?", ref open, ImGuiWindowFlags.AlwaysAutoResize))
            {
                ImGui.Text("Do you really want to delete this experiment? ");
                ImGui.Text($"{loadedState.GetExperimentDataState(loadedState.ExperimentDeleteRequested.Value).info.Name}");
                if (ImGui.Button("Delete"))
                {
                    loadedState.RemoveExperiment(loadedState.ExperimentDeleteRequested.Value);
                    loadedState.ExperimentDeleteRequested = null;
                    ImGui.CloseCurrentPopup();
                }
                ImGui.SameLine();
                if (ImGui.Button("Cancel"))
                {
                    loadedState.ExperimentDeleteRequested = null;
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
                    ImGui.BeginPopupModal("Delete experiment?");
                }

                ImGui.SameLine();
                var autoload = experiment.info.AutoLoad;
                if (ImGui.Checkbox("Autoload", ref autoload))
                {
                    experiment.info.AutoLoad = autoload;
                    loadedState.MarkSettingsDirty();
                    ImGui.BeginPopupModal("Delete experiment?");
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
                                else
                                {
                                    try
                                    {
                                        var data = experiment.loadTestTask.loadTestResultTask.Result;

                                        experiment.data.rawData = new RawExperimentData { cameras = data.cameras, tests = data.tests.ToArray() };
                                        experiment.data.calculatedData = new CalculateExperimentDataState[data.tests.Count];
                                    }
                                    catch (Exception e)
                                    {
                                        experiment.data.rawData = new RawExperimentDataLoadError { exception = e };
                                    }
                                    experiment.loadTestTask = null;
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
                                ImGui.Separator();
                                for (var testId = 0; testId < rawData.tests.Length; testId++)
                                {
                                    ImGui.PushID(testId);
                                    bool selected = experiment.selectedTests.Contains(testId);
                                    var width = ImGui.GetWindowContentRegionWidth();

                                    switch (experiment.data.calculatedData[testId])
                                    {
                                        case null:
                                            {
                                                ImGui.Text($"{testId}");
                                                ImGui.SameLine();
                                                ImGui.TextColored(Color.gray, "Unavailable");
                                                ImGui.SameLine();
                                                if (ImGui.Button("Calculate"))
                                                {
                                                    experiment.data.calculatedData[testId] =
                                                        CreateCalculateDataTask(rawData.tests[testId], loadedState.Treshold);
                                                }
                                                break;
                                            }
                                        case CalculateExperimentDataReadyState readyState:
                                            {
                                                ImGui.Text($"{testId}");
                                                ImGui.SameLine();
                                                ImGui.Checkbox("Selected", ref selected);
                                                if (selected)
                                                {
                                                    experiment.selectedTests.Add(testId);
                                                }
                                                else
                                                {
                                                    experiment.selectedTests.Remove(testId);
                                                }
                                                break;
                                            }
                                        case CalculateExperimentDataInProgressState progressState:
                                            {
                                                ImGui.Text($"{testId} ");
                                                ImGui.SameLine();

                                                if (progressState.task.Status == TaskStatus.WaitingToRun)
                                                {
                                                    ImGui.TextColored(Color.white, "Waiting...");
                                                }

                                                if (progressState.task.Status == TaskStatus.Running)
                                                {
                                                    ImGui.TextColored(Color.yellow, "In progress...");
                                                }

                                                ImGui.SameLine();
                                                if (ImGui.Button("Stop"))
                                                {
                                                    progressState.calculateDataCancellationToken.Cancel();
                                                    experiment.data.calculatedData[testId] = null;
                                                }
                                                break;
                                            }
                                    }
                                    ImGui.Separator();
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
                        ImGui.Text($"Group is contradicted with {e.groups}");
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
                    
                    if (ImGui.BeginTabBar("##info"))
                    {
                        if (ImGui.BeginTabItem("History"))
                        {
                            DrawPointHistory(events, groupId.groupIdInTest);
                            ImGui.EndTabItem();
                        }

                        if (ImGui.BeginTabItem("Info"))
                        {
                            if (ImGui.Button("Show rays"))
                            {
                                foreach (var rayIndex in readyState.points[readyState.groupIdToPointIndex[groupId.groupIdInTest]].pointIds)
                                {
                                    experiment.selectedRays.Add(new RayId{index = rayIndex, testId = groupId.testId});
                                }
                            }
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

    void OnLayout()
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
            DrawVisual(loadedState, m_visualCache);

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

            for (var experimentId = 0; experimentId < loadedState.ExperimentsCount; experimentId++)
            {
                var experiment = loadedState.GetExperimentDataState(experimentId);

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

            if (loadedState.IsSettingsDirty)
            {
                Loader.SaveExperimentsInfo(loadedState.Settings);
                loadedState.MarkSettingsClean();
            }
        }

        ImGui.BeginMainMenuBar();

        if (m_state is LoadedExperimentsInfoSate state)
        {
            if (ImGui.MenuItem("Experiments"))
            {
                state.ExperimentsWindowState.Open = !state.ExperimentsWindowState.Open;
            }

            if (ImGui.MenuItem("Tests"))
            {
                state.TestsWindowsIsOpen = !state.TestsWindowsIsOpen;
            }

            if (ImGui.MenuItem("Point Info"))
            {
                state.PointInfoWindowsIsOpen = !state.PointInfoWindowsIsOpen;
            }

            if (ImGui.MenuItem("Exit"))
            {
                Application.Quit();
            }
        }

        ImGui.EndMainMenuBar();
    }
}
