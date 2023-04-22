using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;


public struct RayIndex
{
    public int camId;
    public int rayId;

    public RayIndex(int cam_id, int ray_id)
    {
        camId = cam_id;
        rayId = ray_id;
    }
}
public struct FoundPoint
{
    public Vector3 point;
    public RayIndex[] pointIds;
    public int groupIndex;
    public FindPoints.Segment segment;
    public float[] distanceToRay;
    public float score;
    public float maxDistanceFromCenter;
    public float midDistanceFromCameras;
    public List<int>[] nearestNeighborsRayId;
    public List<int>[] nearestNeighborsPointId;
    public List<int>[] intersectedNeighborsRayId;
    public List<int>[] availablePoints;
}

public struct IndexedRay
{
    public Ray ray;
    public RayIndex pointIndex;
}

public struct Group
{
    public int id;
}

public static class FindPoints { 

    public struct Segment
    {
        public Vector3 s1;
        public Vector3 s2;

        public Segment(Vector3 s1_, Vector3 s2_)
        {
            s1 = s1_;
            s2 = s2_;
        }

        public Vector3 point => (s1 + s2) / 2;
        public float length => Vector3.Distance(s1, s2);
    }

    private static Segment findRayIntersection(Ray cam1Ray, Ray cam2Ray)
    {
        var a = cam1Ray.direction.normalized;
        var b = cam2Ray.direction.normalized;
        var c = cam2Ray.origin - cam1Ray.origin;

        var D = cam1Ray.origin + a * ((-Vector3.Dot(a, b) * Vector3.Dot(b, c) + Vector3.Dot(a, c) * Vector3.Dot(b, b)) /
                                       (Vector3.Dot(a, a) * Vector3.Dot(b, b) - Vector3.Dot(a, b) * Vector3.Dot(a, b)));
        var E = cam2Ray.origin + b * ((Vector3.Dot(a, b) * Vector3.Dot(a, c) - Vector3.Dot(b, c) * Vector3.Dot(a, a)) /
                                       (Vector3.Dot(a, a) * Vector3.Dot(b, b) - Vector3.Dot(a, b) * Vector3.Dot(a, b)));

        return new Segment(D, E);
    }

    private static float distanceToLine(Ray ray, Vector3 point)
    {
        return Vector3.Cross(ray.direction, point - ray.origin).magnitude;
    }

    private static int getBestCameraId(List<IndexedRay>[] cameraRays, HashSet<RayIndex> visited)
    {
        var bestCameraId = -1;
        var bestCameraRaysCount = 0;

        for (var i = 0; i < cameraRays.Length; i++)
        {
            var visitedCount = 0;

            for (var j = 0; j < cameraRays[i].Count; j++)
            {
                if (visited.Contains(new RayIndex(i, j)))
                {
                    ++visitedCount;
                }
            }

            var cameraRaysCount = cameraRays[i].Count - visitedCount;
            if (cameraRaysCount > bestCameraRaysCount)
            {
                bestCameraRaysCount = cameraRaysCount;
                bestCameraId = i;
            }
        }

        return bestCameraId;
    }

    private struct RayIntersection
    {
        public int camId;
        public int rayId;
        public float distance;
        public Segment segment;
        public RayIndex ray1Index;
        public RayIndex ray2Index;

        public RayIntersection(int camId_, int rayId_, float distance_, Segment segment_, RayIndex ray1Id, RayIndex ray2Id)
        {
            camId = camId_;
            rayId = rayId_;
            distance = distance_;
            segment = segment_;
            ray1Index = ray1Id;
            ray2Index = ray2Id;
        }
    }

    private class IntersectionWindow
    {
        public List<RayIntersection> intersections;
        public float score;
        public float nearestNeighborsDistancesScore;
        public int nearestPointsSetUnionLength;
        public float[] distanceToRay;
        public List<int>[] nearestNeighborsRayId;
        public List<int>[] nearestNeighborsPointId;
        public List<int>[] intersectedNeighborsRayId;
        public List<int>[] availablePoints;
    }

    public struct IntersectionGroup
    {
        public int[] camRayIndices;
        public int id;
        public float score;
    }

    public struct IntersectionGroupRaw
    {
        public int[] camRayIndices;
        public int id;
    }

    public class Event
    {
    }

    public class RayIntersectionEvent : Event
    {
        public RayIndex ray1Id;
        public RayIndex ray2Id;
        public Segment segment;
        public int groupId;
    }

    public class GroupMergeEvent : Event
    {
        public int group1Id;
        public int group2Id;
        public int groupId;
        public RayIndex[] rayIndices;
        public Vector3 point;
    }

    public class RayDeleteEvent : Event
    {
        public int groupId;
        public RayIndex rayId;
    }

    public class GroupDeletedDueToRepeatEvent : Event
    {
        public int groupId;
    }

    public class GroupRegisteredAsMax : Event
    {
        public int groupId;
    }
    public class GroupIsUncotradicted : Event
    {
        public int groupId;
    }

    public class GroupContradictionFound : Event
    {
        public int groupId;
        public int[] groups;
    }

    public class GroupSelectedAsMostCompact : Event
    {
        public int groupId;
    }

    public class GroupDeleteEvent : Event
    {
        public int groupId;
    }

    public struct IntersectionGroupIterations
    {
        public SortedSet<int> rayIndices;
        public Vector3 middlePoint;
        public float midDistanceFromCameras;
    }

    public struct SetIntersectionGroup
    {
        public SortedSet<int> camRayIndices;
        public float score;
    }

    public class MatchConsistencyResult
    {
        public List<int[]> consistentGroups;
        public List<IntersectionGroup> conflictingGroups;
    }

    private class RayIntersectionWindows
    {
        public int rayId;
        public List<IntersectionWindow> windows;
    }

    private class NearestNeighborsAndAvailablePoints
    {
        public List<int> nearestNeighbors;
        public List<KeyValuePair<int, int>> availablePoints;
    }

    public class FilterBasedOnGroupsResultIteration
    {
        public List<IntersectionGroup> groups;
        public int uncotradictedCount;
        public int contradictedCount;
    }

    public class FilterBasedOnGroupsResult
    {
        public List<IntersectionGroup> intersectionGroups;
        public List<IntersectionGroup>[] intersectionGroupsIterations;
        public FilterBasedOnGroupsResultIteration[] iterations;
    }

    public struct AlgorithmProgress
    {
        public int calculatedCount;
        public int testsCount;
    }

    private static NearestNeighborsAndAvailablePoints FindKNearestPointIds(List<Vector2>[] points2D, List<FoundPoint> points3D, int camId, Vector2 point, int k)
    { 
        var availablePoints = new List<KeyValuePair<int, int>>(points3D.Select((x, i) => new KeyValuePair<int, FoundPoint>(i, x)).Where(x => x.Value.pointIds.Any(y => y.camId == camId)).Select(x => new KeyValuePair<int, int>(x.Key, x.Value.pointIds.First(y => y.camId == camId).rayId)));

        availablePoints.Sort((x, y) => Vector2.Distance(points2D[camId][x.Value], point).CompareTo(Vector2.Distance(points2D[camId][y.Value], point)));

        return new NearestNeighborsAndAvailablePoints { availablePoints = availablePoints, nearestNeighbors = new List<int>(availablePoints.Take(k).Select(x => x.Key))};
    }

    private static bool IsMatchConsistency(List<IntersectionGroup> groups, List<int>[][] rayIndexInGroups, int current_cam_id, int current_ray_id, HashSet<int> visitedGroups, HashSet<int> currentVisitedGroups, bool[][] camRayVisited, int[] currentGroup)
    {
        camRayVisited[current_cam_id][current_ray_id] = true;

        var camsCount = rayIndexInGroups.Length;

        bool consistent = true;

        foreach (var group_id in rayIndexInGroups[current_cam_id][current_ray_id])
        {
            if (visitedGroups.Contains(group_id))
            {
                continue;
            }

            visitedGroups.Add(group_id);
            currentVisitedGroups.Add(group_id);

            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var a = currentGroup[cam_id];
                var b = groups[group_id].camRayIndices[cam_id];

                if (a != -1 && b != -1 && a != b)
                {
                    consistent = false;
                }

                if (a == -1)
                {
                    currentGroup[cam_id] = b;
                }
            }

            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var a = currentGroup[cam_id];
                var b = groups[group_id].camRayIndices[cam_id];

                if (b >= 0 && !camRayVisited[cam_id][b])
                {
                    consistent &= IsMatchConsistency(groups, rayIndexInGroups, cam_id, b, visitedGroups, currentVisitedGroups, camRayVisited, currentGroup);
                }
            }
        }

        return consistent;
    }

    public static MatchConsistencyResult EnforceMatchConsistency(List<IntersectionGroup> groups)
    {
        var camsCount = groups.First().camRayIndices.Length;

        var consistentGroups = new List<int[]>();

        var rayIndexInGroups = new List<int>[camsCount][];

        var rayIndexVisited = new bool[camsCount][];

        for (var cam_id = 0; cam_id < camsCount; cam_id++)
        {
            var rayIndexCount = groups.Max(x => x.camRayIndices[cam_id]) + 1;

            rayIndexVisited[cam_id] = new bool[rayIndexCount];
            rayIndexInGroups[cam_id] = new List<int>[rayIndexCount];

            for (var rayIndex = 0; rayIndex < rayIndexCount; rayIndex++)
            {
                rayIndexInGroups[cam_id][rayIndex] = new List<int>();
            }
        }

        for (var group_id = 0; group_id < groups.Count; group_id++)
        {
            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var rayIndex = groups[group_id].camRayIndices[cam_id];
                if (rayIndex >= 0)
                {
                    rayIndexInGroups[cam_id][rayIndex].Add(group_id);
                }
            }
        }

        var visitedGroups = new HashSet<int>();

        var conflictingGroups = new List<IntersectionGroup>();

        for (var group_id = 0; group_id < groups.Count; group_id++)
        {
            if (visitedGroups.Contains(group_id))
            {
                continue;
            }

            var currentGroup = (int[]) groups[group_id].camRayIndices.Clone();

            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var ray_id = groups[group_id].camRayIndices[cam_id];

                if (ray_id < 0 || rayIndexVisited[cam_id][ray_id])
                {
                    continue;
                }

                var currentVisitedGroups = new HashSet<int>();
                if (IsMatchConsistency(groups, rayIndexInGroups, cam_id, ray_id, visitedGroups, currentVisitedGroups, rayIndexVisited, currentGroup))
                {
                    consistentGroups.Add(currentGroup);
                }
                else
                {
                    foreach (var group in currentVisitedGroups)
                    {
                        conflictingGroups.Add(groups[group]);
                    }
                }
            }
        }

        return new MatchConsistencyResult
        {
            consistentGroups = consistentGroups,
            conflictingGroups = conflictingGroups
        };
    }

    public static List<int[]> FilterSubsets(List<int[]> groups)
    {
        var res = new List<int[]>();

        var camsCount = groups.First().Length;

        var rayIndexInGroups = new SortedSet<int>[camsCount][];

        for (var cam_id = 0; cam_id < camsCount; cam_id++)
        {
            var rayIndexCount = groups.Max(x => x[cam_id]) + 1;

            rayIndexInGroups[cam_id] = new SortedSet<int>[rayIndexCount];

            for (var rayIndex = 0; rayIndex < rayIndexCount; rayIndex++)
            {
                rayIndexInGroups[cam_id][rayIndex] = new SortedSet<int>();
            }
        }

        for (var group_id = 0; group_id < groups.Count; group_id++)
        {
            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var rayIndex = groups[group_id][cam_id];
                if (rayIndex >= 0)
                {
                    rayIndexInGroups[cam_id][rayIndex].Add(group_id);
                }
            }
        }

        var visitedGroup = new bool[groups.Count];

        for (var group_id = 0; group_id < groups.Count; group_id++)
        {
            if (visitedGroup[group_id])
            {
                continue;
            }

            SortedSet<int> intersection = null;
            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var ray_index = groups[group_id][cam_id];
                if (ray_index >= 0)
                {
                    if (intersection == null)
                    {
                        intersection = new SortedSet<int>(rayIndexInGroups[cam_id][ray_index]);
                    }
                    else
                    {
                        intersection.IntersectWith(rayIndexInGroups[cam_id][ray_index]);
                    }
                }
            }

            if (intersection.Count == 1)
            {
                res.Add(groups[group_id]);
            } 
            else if (intersection.Count > 1)
            {
                bool notSmaller = true;
                bool identical = true;
                foreach (var other_group_id in intersection)
                {
                    for (var cam_id = 0; cam_id < camsCount; cam_id++)
                    {
                        var a = groups[other_group_id][cam_id];
                        var b = groups[group_id][cam_id];
                        if (groups[other_group_id][cam_id] >= 0 && groups[group_id][cam_id] < 0)
                        {
                            notSmaller = false;
                            identical = false;
                            break;
                        }

                        identical &= a == b;
                    }
                }

                if (notSmaller)
                {
                    res.Add(groups[group_id]);
                    if (identical)
                    {
                        foreach (var other_group_id in intersection)
                        {
                            visitedGroup[other_group_id] = true;
                        }
                    }
                }
            }
        }

        return res;
    }

    public static List<IntersectionGroup> FindIntersectionGroups(List<IndexedRay>[] cameraRays, float treshold)
    {
        var res = new List<IntersectionGroup>();

        for (var firstCamId = 0; firstCamId < cameraRays.Length; firstCamId++)
        {
            for (var ray1_id = 0; ray1_id < cameraRays[firstCamId].Count; ++ray1_id)
            {
                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == firstCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        var segment = findRayIntersection(
                            cameraRays[firstCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        var midPoint = (segment.s1 + segment.s2) / 2;

                        var midDistance = (Vector3.Distance(cameraRays[firstCamId][ray1_id].ray.origin, midPoint) +
                                           Vector3.Distance(cameraRays[secondCamId][ray2_id].ray.origin, midPoint)) / 2;

                        if (segment.length < treshold * midDistance)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[firstCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[firstCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        ++j;
                    }

                    var camRayIndices = new int[cameraRays.Length];

                    for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
                    {
                        camRayIndices[cam_id] = -1;
                    }

                    camRayIndices[firstCamId] = ray1_id;
                    foreach (var ray in set)
                    {
                        camRayIndices[ray.Key] = ray.Value.rayId;
                    }

                    res.Add(new IntersectionGroup {
                        camRayIndices = camRayIndices
                    });
                }
            }
        }

        return res;
    }

    public static FoundPoint CalculatePoint(List<IndexedRay>[] cameraRays, int[] group)
    {
        var pointCloud = new Vector3[cameraRays.Length * cameraRays.Length];

        var pointIds = new List<RayIndex>();
        var i = 0;
        for (var cam1_id = 0; cam1_id < cameraRays.Length; cam1_id++)
        {
            if (group[cam1_id] < 0)
            {
                continue;
            }

            pointIds.Add(new RayIndex(cam1_id, group[cam1_id]));

            for (var cam2_id = cam1_id + 1; cam2_id < cameraRays.Length; cam2_id++)
            {
                if (group[cam2_id] < 0)
                {
                    continue;
                }

                var segment = findRayIntersection(cameraRays[cam1_id][group[cam1_id]].ray,
                    cameraRays[cam2_id][group[cam2_id]].ray);

                pointCloud[i++] = segment.s1;
                pointCloud[i++] = segment.s2;
            }
        }

        var middle = pointCloud.Take(i).Aggregate((x, y) => x + y) / i;

        var variance = Mathf.Sqrt(pointCloud.Take(i).Select(x => Mathf.Pow((middle - x).magnitude, 2)).Sum() / pointCloud.Length);

        var maxDistanceFromCenter = pointCloud.Take(i).Max(x => Vector3.Distance(middle, x));

        var raysCount = 0;
        var midDistanceFromCameras = 0.0f;
        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
        {
            var ray_id = group[cam_id];
            if (ray_id < 0)
            {
                continue;
            }

            raysCount++;

            midDistanceFromCameras += Vector3.Distance(cameraRays[cam_id][ray_id].ray.origin, middle);
        }

        midDistanceFromCameras /= raysCount;

        return new FoundPoint
        {
            point = middle,
            score = variance,
            pointIds = pointIds.ToArray(),
            maxDistanceFromCenter = maxDistanceFromCenter,
            midDistanceFromCameras = midDistanceFromCameras
        };
    }

    public static void RecalculateGroupScoreIfNeeded(List<IndexedRay>[] cameraRays, ref IntersectionGroup group)
    {
        if (group.score > 0)
        {
            return;
        }

        group.score = CalculatePoint(cameraRays, group.camRayIndices).score;
    }

    public static FoundPoint[] GetPoints(List<IndexedRay>[] cameraRays, List<int[]> groups)
    {
        return groups.Select(group => CalculatePoint(cameraRays, group)).ToArray();
    }

    public static FoundPoint[] GetPoints(List<IndexedRay>[] cameraRays, List<IntersectionGroup> groups)
    {
        return groups.Select(group =>
        {
            var point = CalculatePoint(cameraRays, group.camRayIndices);
            point.groupIndex = group.id;
            return point;
        }).ToArray();
    }

    public static FilterBasedOnGroupsResult FilterBasedOnGroups(List<IndexedRay>[] cameraRays, List<IntersectionGroupRaw> _groups, List<Event> events = null)
    {
        var res = new List<int>();

        var groups = _groups.Select(x => new IntersectionGroup { camRayIndices = x.camRayIndices, score = 0, id = x.id }).ToList();

        var camsCount = cameraRays.Length;

        //Указывает, в каких группах находится луч [cam_id][ray_index]
        var rayIndexInGroups = new SortedSet<int>[camsCount][];

        for (var cam_id = 0; cam_id < camsCount; cam_id++)
        {
            var rayIndexCount = groups.Max(x => x.camRayIndices[cam_id]) + 1;

            rayIndexInGroups[cam_id] = new SortedSet<int>[rayIndexCount];

            for (var rayIndex = 0; rayIndex < rayIndexCount; rayIndex++)
            {
                rayIndexInGroups[cam_id][rayIndex] = new SortedSet<int>();
            }
        }

        for (var group_id = 0; group_id < groups.Count; group_id++)
        {
            for (var cam_id = 0; cam_id < camsCount; cam_id++)
            {
                var rayIndex = groups[group_id].camRayIndices[cam_id];
                if (rayIndex >= 0)
                {
                    rayIndexInGroups[cam_id][rayIndex].Add(group_id);
                }
            }
        }

        var addedGroups = new bool[groups.Count];

        var intersectionGroupIterations = new List<List<IntersectionGroup>>();
        var resIterations = new List<FilterBasedOnGroupsResultIteration>();

        while (true)
        {
            var intersectionGroup = new List<IntersectionGroup>(groups.Count + res.Count);

            for (var group_id = 0; group_id < groups.Count; group_id++)
            {
                if (addedGroups[group_id])
                {
                    continue;
                }

                if (groups[group_id].score == 0)
                {
                    var group = groups[group_id];
                    group.score = CalculatePoint(cameraRays, groups[group_id].camRayIndices).score;
                    groups[group_id] = group;
                }

                intersectionGroup.Add(new IntersectionGroup{ camRayIndices = (int[])groups[group_id].camRayIndices.Clone(), score = groups[group_id].score });
            }

            //foreach (var group_id in res)
            //{
            //    intersectionGroup.Add(new IntersectionGroup { camRayIndices = (int[])groups[group_id].camRayIndices.Clone(), score = groups[group_id].score });
            //}

            var maxRayInGroupCount = -1;

            for (var group_id = 0; group_id < groups.Count; group_id++)
            {
                if (!addedGroups[group_id])
                {
                    var rayCount = groups[group_id].camRayIndices.Count(y => y >= 0);
                    maxRayInGroupCount = rayCount > maxRayInGroupCount ? rayCount : maxRayInGroupCount;
                }
            }

            var maxGroups = new SortedSet<int>();

            // Получаем группы с максимальным количеством пересечений
            for (var group_id = 0; group_id < groups.Count; group_id++)
            {
                if (groups[group_id].camRayIndices.Count(x => x >= 0) == maxRayInGroupCount && !addedGroups[group_id])
                {
                    maxGroups.Add(group_id);
                    if (events != null)
                    {
                        events.Add(new GroupRegisteredAsMax { groupId = groups[group_id].id });
                    }
                }
            }

            intersectionGroupIterations.Add(intersectionGroup);

            if (maxGroups.Count == 0)
            {
                break;
            }

            var visitedGroups = new bool[groups.Count];

            var lastEndIndex = res.Count;

            // Выделяем группы, которые являются единственными максимальными на своем луче
            foreach (var group_id in maxGroups)
            {
                if (visitedGroups[group_id])
                {
                    continue;
                }

                bool contradicted = false;

                var contradictedGroups = new SortedSet<int> { group_id };

                for (var cam_id = 0; cam_id < camsCount; cam_id++)
                {
                    var ray_id = groups[group_id].camRayIndices[cam_id];
                    if (ray_id < 0)
                    {
                        continue;
                    }

                    var intersection = new SortedSet<int>(rayIndexInGroups[cam_id][ray_id]);
                    intersection.IntersectWith(maxGroups);

                    if (intersection.Count > 1)
                    {
                        contradicted = true;
                    }

                    foreach (var contradicted_group_id in intersection)
                    {
                        contradictedGroups.Add(contradicted_group_id);
                        visitedGroups[contradicted_group_id] = true;
                    }
                }

                if (!contradicted)
                {
                    res.Add(group_id);

                    addedGroups[group_id] = true;

                    if (events != null)
                    {
                        events.Add(new GroupIsUncotradicted
                        {
                            groupId = groups[group_id].id
                        });
                    }
                }
                else
                {
                    if (events != null)
                    {
                        events.Add(new GroupContradictionFound
                        {
                            groupId = group_id,
                            groups = contradictedGroups.ToArray()
                        });
                    }
                }
            }

            for (var res_group_id = lastEndIndex; res_group_id < res.Count; res_group_id++)
            {
                // Исключаем из всех смежных групп лучи, на которых была образована добавленная группа
                for (var cam_id = 0; cam_id < camsCount; cam_id++)
                {
                    var ray_id = groups[res[res_group_id]].camRayIndices[cam_id];

                    if (ray_id < 0)
                    {
                        continue;
                    }

                    foreach (var another_group_id in rayIndexInGroups[cam_id][ray_id])
                    {
                        if (another_group_id == res[res_group_id])
                        {
                            continue;
                        }
                        
                        groups[another_group_id].camRayIndices[cam_id] = -1;

                        if (events != null)
                        {
                            events.Add(new RayDeleteEvent
                            {
                                groupId = groups[another_group_id].id,
                                rayId = new RayIndex { camId = cam_id, rayId = ray_id }
                            });
                        }

                        var rayCount = 0;
                        for (var cam2_id = 0; cam2_id < cameraRays.Length; cam2_id++)
                        {
                            if (groups[another_group_id].camRayIndices[cam2_id] >= 0)
                            {
                                rayCount++;
                            }
                        }

                        if (rayCount < 2)
                        {
                            addedGroups[another_group_id] = true;
                            if (events != null)
                            {
                                events.Add(new GroupDeleteEvent
                                {
                                    groupId = groups[another_group_id].id
                                });
                            }
                        }

                        maxGroups.Remove(another_group_id);
                    }

                    rayIndexInGroups[cam_id][ray_id].Clear();

                    maxGroups.Remove(res[res_group_id]);
                }
            }

            var uncontradictedCount = res.Count - lastEndIndex;

            var contradictedGroupsCount = maxGroups.Count;

            // Рассматриваем оставшиеся конфликтующие группы
            while (maxGroups.Count > 0)
            {
                foreach (var group_id in maxGroups)
                {
                    if (groups[group_id].score == 0)
                    {
                        var group = groups[group_id];
                        group.score = CalculatePoint(cameraRays, groups[group_id].camRayIndices).score;
                        groups[group_id] = group;
                    }
                }

                // Выбираем наиболее компактную группу
                var most_compact_group_id = maxGroups.OrderBy(x => groups[x].score).First();

                res.Add(most_compact_group_id);
                addedGroups[most_compact_group_id] = true;

                if (events != null)
                {
                    events.Add(new GroupSelectedAsMostCompact
                    {
                        groupId = groups[most_compact_group_id].id
                    });
                }

                // Исклюачем из дальнейшего рассмотрения все группы, которые лежат на лучах, которые мы добавили в решение
                for (var cam_id = 0; cam_id < camsCount; cam_id++)
                {
                    var ray_id = groups[most_compact_group_id].camRayIndices[cam_id];

                    if (ray_id < 0)
                    {
                        continue;
                    }

                    foreach (var another_group_id in rayIndexInGroups[cam_id][ray_id])
                    {
                        if (another_group_id == most_compact_group_id)
                        {
                            continue;
                        }

                        groups[another_group_id].camRayIndices[cam_id] = -1;

                        if (events != null)
                        {
                            events.Add(new RayDeleteEvent
                            {
                                groupId = groups[another_group_id].id,
                                rayId = new RayIndex { camId = cam_id, rayId = ray_id }
                            });
                        }

                        var rayCount = 0;
                        for (var cam2_id = 0; cam2_id < cameraRays.Length; cam2_id++)
                        {
                            if (groups[another_group_id].camRayIndices[cam2_id] >= 0)
                            {
                                rayCount++;
                            }
                        }

                        if (rayCount < 2)
                        {
                            addedGroups[another_group_id] = true;
                            if (events != null)
                            {
                                events.Add(new GroupDeleteEvent
                                {
                                    groupId = groups[another_group_id].id
                                });
                            }
                        }

                        maxGroups.Remove(another_group_id);
                    }

                    rayIndexInGroups[cam_id][ray_id].Clear();

                    maxGroups.Remove(most_compact_group_id);
                }
            }

            resIterations.Add(new FilterBasedOnGroupsResultIteration
            {
                groups = res.Skip(lastEndIndex).Select(x => groups[x]).ToList(),
                uncotradictedCount = uncontradictedCount,
                contradictedCount = contradictedGroupsCount
            });
        }

        return new FilterBasedOnGroupsResult
        {
            intersectionGroups = res.Select(x => groups[x]).ToList(),
            intersectionGroupsIterations = intersectionGroupIterations.ToArray(),
            iterations = resIterations.ToArray()
        };
    }

    public static FoundPoint[] FindBasedOnGroups(List<IndexedRay>[] cameraRays, float treshold, List<Event> events = null)
    {
        var groups = FindIntersectionGroupsBasedOnIterations(cameraRays, treshold, events);

        if (groups.Count == 0)
        {
            return Array.Empty<FoundPoint>();
        }

        var filteredGroups = FilterBasedOnGroups(cameraRays, groups, events);

        return GetPoints(cameraRays, filteredGroups.intersectionGroups);
    }

    public static FoundPoint[] FindBasedOnPaper(List<IndexedRay>[] cameraRays, float treshold)
    {
        var groups = FindIntersectionGroups(cameraRays, treshold);

        var matches = EnforceMatchConsistency(groups);

        return GetPoints(cameraRays, matches.consistentGroups);
    }

    //private bool IsIntersect(List<IndexedRay>[] cameraRays, float treshold, int[] group, out Vector3 midPoint, out float maxDistance)
    //{
    //    midPoint = Vector3.zero;

    //    for (var cam1_id = 0; cam1_id < cameraRays.Length; cam1_id++)
    //    {
    //        var ray1_id = group[cam1_id];

    //        if (ray1_id < 0)
    //        {
    //            continue;
    //        }

    //        for (var cam2_id = cam1_id + 1; cam2_id < cameraRays.Length; cam2_id++)
    //        {
    //            var ray2_id = group[cam2_id];

    //            if (ray2_id < 0)
    //            {
    //                continue;
    //            }

    //            var segment = findRayIntersection(
    //                cameraRays[cam1_id][ray1_id].ray,
    //                cameraRays[cam2_id][ray2_id].ray
    //            );

    //            var midPoint = (segment.s1 + segment.s2) / 2;

    //            var midDistance = (Vector3.Distance(cameraRays[cam1_id][ray1_id].ray.origin, midPoint) +
    //                               Vector3.Distance(cameraRays[cam2_id][ray2_id].ray.origin, midPoint)) / 2;

    //            if (segment.length < treshold * midDistance)
    //            {
    //                rayIntersections.Add(new RayIntersection(
    //                    secondCamId,
    //                    ray2_id,
    //                    Vector3.Distance(segment.s1, cameraRays[firstCamId][ray1_id].ray.origin),
    //                    segment,
    //                    cameraRays[firstCamId][ray1_id].pointIndex,
    //                    cameraRays[secondCamId][ray2_id].pointIndex
    //                ));
    //            }
    //        }
    //    }
    //}


    // Новый красивый алгоритм
    public static List<IntersectionGroupRaw> FindIntersectionGroupsBasedOnIterations(List<IndexedRay>[] cameraRays, float treshold, List<Event> events = null)
    {
        var groups = new List<IntersectionGroupIterations>();

        var group = new int[cameraRays.Length];

        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
        {
            group[cam_id] = -1;
        }

        var camRayIdToRayId = new int[cameraRays.Length][];
        var rayIdToCamRayId = new List<RayIndex>();

        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
        {
            camRayIdToRayId[cam_id] = new int[cameraRays[cam_id].Count];

            for (var ray_id = 0; ray_id < cameraRays[cam_id].Count; ray_id++)
            {
                rayIdToCamRayId.Add(new RayIndex(cam_id, ray_id));
                camRayIdToRayId[cam_id][ray_id] = rayIdToCamRayId.Count - 1;
            }
        }

        for (var cam1_id = 0; cam1_id < cameraRays.Length; cam1_id++)
        {
            for (var ray1_id = 0; ray1_id < cameraRays[cam1_id].Count; ray1_id++)
            {
                for (var cam2_id = cam1_id + 1; cam2_id < cameraRays.Length; cam2_id++)
                {
                    for (var ray2_id = 0; ray2_id < cameraRays[cam2_id].Count; ray2_id++)
                    {
                        group[cam1_id] = ray1_id;
                        group[cam2_id] = ray2_id;

                        var point = CalculatePoint(cameraRays, group);

                        group[cam1_id] = -1;
                        group[cam2_id] = -1;

                        if (point.maxDistanceFromCenter < treshold * point.midDistanceFromCameras)
                        {
                            groups.Add(new IntersectionGroupIterations
                            { 
                                rayIndices = new SortedSet<int> {
                                    camRayIdToRayId[cam1_id][ray1_id],
                                    camRayIdToRayId[cam2_id][ray2_id],
                                },
                                middlePoint = point.point,
                                midDistanceFromCameras = point.midDistanceFromCameras
                            });

                            if (events != null)
                            {
                                events.Add(new RayIntersectionEvent
                                {
                                    groupId = groups.Count - 1,
                                    ray1Id = new RayIndex(cam1_id, ray1_id),
                                    ray2Id = new RayIndex(cam2_id, ray2_id),
                                    segment = findRayIntersection(cameraRays[cam1_id][ray1_id].ray,
                                        cameraRays[cam2_id][ray2_id].ray)
                                });
                            }
                        }
                    }
                }
            }
        }

        var deletedGroups = new List<bool>(groups.Count);
        deletedGroups.AddRange(Enumerable.Repeat(false, groups.Count));

        var groupByRayId = new SortedSet<int>[rayIdToCamRayId.Count];
        for (var ray_id = 0; ray_id < rayIdToCamRayId.Count; ray_id++)
        {
            groupByRayId[ray_id] = new SortedSet<int>();
        }

        var uniqueGroups = new HashSet<SortedSet<int>>(SortedSet<int>.CreateSetComparer());

        var lastEndIndex = 0;
        for (var iter_id = 3; iter_id < cameraRays.Length + 1; iter_id++)
        {
            for (var ray_id = 0; ray_id < cameraRays.Length; ray_id++)
            {
                group[ray_id] = -1;
            }

            var lastStartIndex = lastEndIndex;
            lastEndIndex = groups.Count;

            foreach (var sortedSet in groupByRayId)
            {
                sortedSet.Clear();
            }

            for (var group_id = lastStartIndex; group_id < lastEndIndex; group_id++)
            {
                foreach (var rayIndex in groups[group_id].rayIndices)
                {
                    groupByRayId[rayIndex].Add(group_id);
                }
            }

            for (var group_id = lastStartIndex; group_id < lastEndIndex; group_id++)
            {
                foreach (var ray_id in groups[group_id].rayIndices)
                {
                    foreach (var another_group_id in groupByRayId[ray_id])
                    {
                        if (group_id == another_group_id)
                        {
                            continue;
                        }

                        if (deletedGroups[another_group_id])
                        {
                            continue;
                        }

                        var distance = Vector3.Distance(groups[group_id].middlePoint,
                            groups[another_group_id].middlePoint);

                        var localTreshold = treshold * 2 * (groups[group_id].midDistanceFromCameras +
                                                             groups[another_group_id].midDistanceFromCameras) / 2;

                        if (distance > localTreshold)
                        {
                            continue;
                        }

                        var unionRayIndices = new SortedSet<int>(groups[group_id].rayIndices);
                        unionRayIndices.UnionWith(groups[another_group_id].rayIndices);

                        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
                        {
                            group[cam_id] = -1;
                        }

                        var contradicted = false;
                        foreach (var cam_id2 in unionRayIndices.Select(x => rayIdToCamRayId[x].camId).ToList())
                        {
                            if (group[cam_id2] > 0)
                            {
                                contradicted = true;
                                break;
                            }
                            group[cam_id2] = 1;
                        }

                        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
                        {
                            group[cam_id] = -1;
                        }

                        if (contradicted)
                        {
                            continue;
                        }

                        var intersectRayIndices = new SortedSet<int>(groups[group_id].rayIndices);

                        intersectRayIndices.IntersectWith(groups[another_group_id].rayIndices);

                        if (uniqueGroups.Contains(unionRayIndices))
                        {
                            deletedGroups[another_group_id] = true;
                            if (events != null)
                            {
                                events.Add(new GroupDeletedDueToRepeatEvent
                                {
                                    groupId = another_group_id
                                });
                            }
                            continue;
                        }

                        foreach (var ray_id2 in unionRayIndices)
                        {
                            var rayIndex = rayIdToCamRayId[ray_id2];
                            group[rayIndex.camId] = rayIndex.rayId;
                        }

                        var point = CalculatePoint(cameraRays, group);

                        for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
                        {
                            group[cam_id] = -1;
                        }

                        if (point.maxDistanceFromCenter < point.midDistanceFromCameras * treshold)
                        {
                            if (intersectRayIndices.Count != 0 && intersectRayIndices.Count != iter_id - 2)
                            {
                                throw new SystemException("ERROR");
                            }

                            deletedGroups[group_id] = true;
                            deletedGroups[another_group_id] = true;

                            groups.Add(new IntersectionGroupIterations
                            {
                                middlePoint = point.point,
                                rayIndices = unionRayIndices,
                                midDistanceFromCameras = point.midDistanceFromCameras
                            });

                            if (events != null)
                            {
                                events.Add(new GroupMergeEvent
                                {
                                    group1Id = group_id,
                                    group2Id = another_group_id,
                                    groupId = groups.Count - 1,
                                    point = point.point,
                                    rayIndices = unionRayIndices.Select(x => rayIdToCamRayId[x]).ToArray()
                                });
                            }

                            deletedGroups.Add(false);
                            uniqueGroups.Add(unionRayIndices);
                        }
                    }
                }
            }
        }

        return groups.Select((x, i) => new Tuple<int, IntersectionGroupIterations>(i, x)).Where(x => !deletedGroups[x.Item1]).Select(x =>
        {
            var _group = new int[cameraRays.Length];
            for (var cam_id = 0; cam_id < cameraRays.Length; cam_id++)
            {
                _group[cam_id] = -1;
            }

            foreach (var ray_id in x.Item2.rayIndices)
            {
                _group[rayIdToCamRayId[ray_id].camId] = rayIdToCamRayId[ray_id].rayId;
            }

            return new IntersectionGroupRaw { camRayIndices = _group, id = x.Item1 };
        }).ToList();
    }

    /*
     * То же самое что и предыдущий, только теперь еще смотрим на расстояния ближайших соседей от 
     */
    public static FoundPoint[] FindBasedOnKNearestNeighborsWithDistances(List<IndexedRay>[] cameraRays, float treshold,
        List<Vector2>[] cameraPoints, int k, int kMin, float minNeighborsScore)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        while (true)
        {
            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();

            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        if (segment.length < treshold)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                List<IntersectionWindow> intersectionWindows = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    var middle = Vector3.zero;

                    middle += rayIntersections[i].segment.s1 + rayIntersections[i].segment.s2;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        middle += rayIntersections[j].segment.s1;
                        middle += rayIntersections[j].segment.s2;
                        ++j;
                    }

                    middle /= set.Count * 2;

                    var variance = 0.0f;

                    foreach (var intersection in set.Values)
                    {
                        variance += Mathf.Pow((middle - intersection.segment.s1).magnitude, 2);
                        variance += Mathf.Pow((middle - intersection.segment.s2).magnitude, 2);
                    }

                    variance = Mathf.Sqrt(variance / set.Count * 2);

                    var nearestPointIds = new NearestNeighborsAndAvailablePoints[cameraRays.Length];
                    var distanceToRays = new float[cameraRays.Length];

                    var allCamRays = new RayIndex[set.Values.Count + 1];

                    {
                        int index = 0;
                        allCamRays[index++] = new RayIndex(bestCamId, ray1_id);
                        foreach (var value in set.Values)
                        {
                            allCamRays[index++] = new RayIndex(value.camId, value.rayId);
                        }
                    }


                    foreach (var camRay in allCamRays)
                    {
                        nearestPointIds[camRay.camId] = FindKNearestPointIds(cameraPoints, res, camRay.camId,
                            cameraPoints[camRay.camId][camRay.rayId], k);

                        distanceToRays[camRay.camId] = distanceToLine(cameraRays[camRay.camId][camRay.rayId].ray, middle);
                    }

                    SortedSet<int> intersectedPointIds = nearestPointIds.Aggregate(new SortedSet<int>(nearestPointIds[bestCamId].nearestNeighbors), (a, b) =>
                    {
                        if (b != null)
                        {
                            a.IntersectWith(b.nearestNeighbors);
                        }

                        return a;
                    });

                    var camPointDistances = new float[allCamRays.Length][];
                    {
                        var camIndex = 0;
                        foreach (var camRay in allCamRays)
                        {
                            var pointDistances = new float[intersectedPointIds.Count];
                            int index = 0;
                            float sum = 0;

                            foreach (var intersectedPointId in intersectedPointIds)
                            {
                                var rayIndex = res[intersectedPointId].pointIds.First(x => x.camId == camRay.camId);
                                index++;
                                pointDistances[index] =
                                    Vector2.Distance(cameraPoints[camRay.camId][camRay.rayId],
                                        cameraPoints[rayIndex.camId][rayIndex.rayId]);
                                sum += pointDistances[index];
                            }

                            for (var pointIndex = 0; pointIndex < pointDistances.Length; pointIndex++)
                            {
                                pointDistances[pointIndex] /= sum;
                            }

                            camPointDistances[camIndex] = pointDistances;
                        }
                    }

                    var nearestNeighborsScore = 0.0f;
                    {
                        var middlePointDistances = new float[intersectedPointIds.Count];
                        foreach (var pointDistances in camPointDistances)
                        {
                            for (var index = 0; index < pointDistances.Length; index++)
                            {
                                middlePointDistances[index] += pointDistances[index];
                            }
                        }
                        for (var index = 0; index < middlePointDistances.Length; index++)
                        {
                            middlePointDistances[index] /= camPointDistances.Length;
                        }

                        foreach (var pointDistances in camPointDistances)
                        {
                            for (var index = 0; index < pointDistances.Length; index++)
                            {
                                nearestNeighborsScore += Mathf.Pow(middlePointDistances[index] - pointDistances[index], 2);
                            }
                        }

                        nearestNeighborsScore /= camPointDistances.Length * middlePointDistances.Length;
                    }

                    var window = new IntersectionWindow { intersections = set.Values.ToList(), nearestNeighborsDistancesScore = nearestNeighborsScore, distanceToRay = distanceToRays, score = variance, nearestPointsSetUnionLength = intersectedPointIds.Count, nearestNeighborsPointId = nearestPointIds.Select(x => x?.nearestNeighbors).ToArray(), nearestNeighborsRayId = nearestPointIds.Select((x, u) => x?.nearestNeighbors.Select(y => res[y].pointIds.First(z => z.camId == u).rayId).ToList()).ToArray(), intersectedNeighborsRayId = Enumerable.Range(0, cameraRays.Length).Select(x => intersectedPointIds.Where(y => res[y].pointIds.Any(u => u.camId == x)).Select(y => res[y].pointIds.First(u => u.camId == x).rayId).ToList()).ToArray() /*, availablePoints = nearestPointIds.Select(x => x?.availablePoints) */};
                    if (intersectionWindows == null)
                    {
                        intersectionWindows = new List<IntersectionWindow> { window };
                    }
                    else
                    {
                        intersectionWindows.Add(window);
                    }
                }

                if (intersectionWindows != null)
                {
                    intersections.Add(new RayIntersectionWindows { windows = intersectionWindows, rayId = ray1_id });
                }
            }

            if (intersections.Count == 0)
            {
                break;
            }

            IntersectionWindow bestIntersectionWindow = null;

            intersections = intersections
                .OrderByDescending(x => x.windows.Max(y => y.nearestPointsSetUnionLength))
                .ThenByDescending(x => -x.windows.Count)
                .ThenByDescending(x => x.windows.Max(y => y.intersections.Count)).ToList();

            var bestCount = intersections[0].windows.Count;
            var bestWidth = float.MaxValue;
            var bestRay = -1;
            var bestInWindowCount = 0;

            foreach (var intersection in intersections)
            {
                foreach (var window in intersection.windows)
                {
                    if (res.Count > k && window.nearestPointsSetUnionLength < kMin)
                    {
                        continue;
                    }

                    if (window.nearestNeighborsDistancesScore < minNeighborsScore)
                    {
                        continue;
                    }

                    if (bestInWindowCount > window.intersections.Count)
                    {
                        continue;
                    }

                    if (bestInWindowCount < window.intersections.Count)
                    {
                        bestWidth = float.MaxValue;
                        bestInWindowCount = window.intersections.Count;
                    }

                    if (window.score < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.score;
                    }
                }
            }

            if (bestRay < 0)
            {
                break;
            }

            var point = Vector3.zero;
            var pointIds = new List<RayIndex>();
            pointIds.Add(cameraRays[bestCamId][bestRay].pointIndex);
            visitedRays.Add(new RayIndex(bestCamId, bestRay));

            foreach (var intersection in bestIntersectionWindow.intersections)
            {
                point += intersection.segment.point;
                pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
            }

            point /= bestIntersectionWindow.intersections.Count;

            var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray(), distanceToRay = bestIntersectionWindow.distanceToRay, score = bestWidth, nearestNeighborsRayId = bestIntersectionWindow.nearestNeighborsRayId, nearestNeighborsPointId = bestIntersectionWindow.nearestNeighborsPointId, intersectedNeighborsRayId = bestIntersectionWindow.intersectedNeighborsRayId };

            res.Add(foundPoint);
        }

        return res.ToArray();
    }

    /*
     * То же самое что и предыдущий, только теперь не смотрим на количество групп на луче а перебираем вообще все.
     */
    public static FoundPoint[] FindBasedOnKNearestNeighborsWithout(List<IndexedRay>[] cameraRays, float treshold,
       List<Vector2>[] cameraPoints, int k, int kMin)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        while (true)
        {
            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();

            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        var midPoint = (segment.s1 + segment.s2) / 2;

                        var midDistance = (Vector3.Distance(cameraRays[bestCamId][ray1_id].ray.origin, midPoint) +
                                           Vector3.Distance(cameraRays[secondCamId][ray2_id].ray.origin, midPoint)) / 2;

                        if (segment.length < treshold * midDistance)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                List<IntersectionWindow> intersectionWindows = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    var middle = Vector3.zero;

                    middle += rayIntersections[i].segment.s1 + rayIntersections[i].segment.s2;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        middle += rayIntersections[j].segment.s1;
                        middle += rayIntersections[j].segment.s2;
                        ++j;
                    }

                    middle /= set.Count * 2;

                    var variance = 0.0f;

                    foreach (var intersection in set.Values)
                    {
                        variance += Mathf.Pow((middle - intersection.segment.s1).magnitude, 2);
                        variance += Mathf.Pow((middle - intersection.segment.s2).magnitude, 2);
                    }

                    variance = Mathf.Sqrt(variance / set.Count * 2);

                    var nearestPointIds = new NearestNeighborsAndAvailablePoints[cameraRays.Length];
                    var distanceToRays = new float[cameraRays.Length];

                    nearestPointIds[bestCamId] = FindKNearestPointIds(cameraPoints, res, bestCamId,
                        cameraPoints[bestCamId][ray1_id], k);

                    distanceToRays[bestCamId] = distanceToLine(cameraRays[bestCamId][ray1_id].ray, middle);

                    foreach (var value in set.Values)
                    {
                        nearestPointIds[value.camId] = FindKNearestPointIds(cameraPoints, res, value.camId,
                            cameraPoints[value.camId][value.rayId], k);

                        distanceToRays[value.camId] = distanceToLine(cameraRays[value.camId][value.rayId].ray, middle);
                    }

                    SortedSet<int> intersectedPointIds = nearestPointIds.Aggregate(new SortedSet<int>(nearestPointIds[bestCamId].nearestNeighbors), (a, b) =>
                    {
                        if (b != null)
                        {
                            a.IntersectWith(b.nearestNeighbors);
                        }

                        return a;
                    });

                    var window = new IntersectionWindow { intersections = set.Values.ToList(), distanceToRay = distanceToRays, score = variance, nearestPointsSetUnionLength = intersectedPointIds.Count, nearestNeighborsPointId = nearestPointIds.Select(x => x?.nearestNeighbors).ToArray(), nearestNeighborsRayId = nearestPointIds.Select((x, u) => x?.nearestNeighbors.Select(y => res[y].pointIds.First(z => z.camId == u).rayId).ToList()).ToArray(), intersectedNeighborsRayId = Enumerable.Range(0, cameraRays.Length).Select(x => intersectedPointIds.Where(y => res[y].pointIds.Any(u => u.camId == x)).Select(y => res[y].pointIds.First(u => u.camId == x).rayId).ToList()).ToArray() /*, availablePoints = nearestPointIds.Select(x => x?.availablePoints) */};
                    if (intersectionWindows == null)
                    {
                        intersectionWindows = new List<IntersectionWindow> { window };
                    }
                    else
                    {
                        intersectionWindows.Add(window);
                    }
                }

                if (intersectionWindows != null)
                {
                    intersections.Add(new RayIntersectionWindows { windows = intersectionWindows, rayId = ray1_id });
                }
            }

            if (intersections.Count == 0)
            {
                break;
            }

            IntersectionWindow bestIntersectionWindow = null;

            intersections = intersections
                .OrderByDescending(x => x.windows.Max(y => y.nearestPointsSetUnionLength))
                .ThenByDescending(x => -x.windows.Count)
                .ThenByDescending(x => x.windows.Max(y => y.intersections.Count)).ToList();

            var bestCount = intersections[0].windows.Count;
            var bestWidth = float.MaxValue;
            var bestRay = -1;
            var bestInWindowCount = 0;

            foreach (var intersection in intersections)
            {
                foreach (var window in intersection.windows)
                {
                    if (res.Count > k && window.nearestPointsSetUnionLength < kMin)
                    {
                        continue;
                    }

                    if (bestInWindowCount > window.intersections.Count)
                    {
                        continue;
                    }

                    if (bestInWindowCount < window.intersections.Count)
                    {
                        bestWidth = float.MaxValue;
                        bestInWindowCount = window.intersections.Count;
                    }

                    if (window.score < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.score;
                    }
                }
            }

            if (bestRay < 0)
            {
                break;
            }

            var point = Vector3.zero;
            var pointIds = new List<RayIndex>();
            pointIds.Add(cameraRays[bestCamId][bestRay].pointIndex);
            visitedRays.Add(new RayIndex(bestCamId, bestRay));

            foreach (var intersection in bestIntersectionWindow.intersections)
            {
                point += intersection.segment.point;
                pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
            }

            point /= bestIntersectionWindow.intersections.Count;

            var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray(), distanceToRay = bestIntersectionWindow.distanceToRay, score = bestWidth, nearestNeighborsRayId = bestIntersectionWindow.nearestNeighborsRayId, nearestNeighborsPointId = bestIntersectionWindow.nearestNeighborsPointId, intersectedNeighborsRayId = bestIntersectionWindow.intersectedNeighborsRayId };

            res.Add(foundPoint);
        }

        return res.ToArray();
    }

    /*
     * То же самое что и предыдущий, только теперь еще формируем для каждой группы множество точек уже найденных точек находящихся рядом в кадре камеры.
     * Сначала выбираем те лучи, у которых больше всего ближайших найденных точек
     * Дальше как в предудыщем
     */
    public static FoundPoint[] FindBasedOnKNearestNeighbors(List<IndexedRay>[] cameraRays, float treshold,
        List<Vector2>[] cameraPoints, int k, int kMin)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        while (true)
        {
            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();

            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        if (segment.length < treshold)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                List<IntersectionWindow> intersectionWindows = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    var middle = Vector3.zero;

                    middle += rayIntersections[i].segment.s1 + rayIntersections[i].segment.s2;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        middle += rayIntersections[j].segment.s1;
                        middle += rayIntersections[j].segment.s2;
                        ++j;
                    }

                    middle /= set.Count * 2;

                    var variance = 0.0f;

                    foreach (var intersection in set.Values)
                    {
                        variance += Mathf.Pow((middle - intersection.segment.s1).magnitude, 2);
                        variance += Mathf.Pow((middle - intersection.segment.s2).magnitude, 2);
                    }

                    variance = Mathf.Sqrt(variance / set.Count * 2);

                    var nearestPointIds = new NearestNeighborsAndAvailablePoints[cameraRays.Length];
                    var distanceToRays = new float[cameraRays.Length];

                    nearestPointIds[bestCamId] = FindKNearestPointIds(cameraPoints, res, bestCamId,
                        cameraPoints[bestCamId][ray1_id], k);

                    foreach (var value in set.Values)
                    {
                        nearestPointIds[value.camId] = FindKNearestPointIds(cameraPoints, res, value.camId,
                            cameraPoints[value.camId][value.rayId], k);

                        distanceToRays[value.camId] = (middle - value.segment.s2).magnitude;
                    }

                    SortedSet<int> intersectedPointIds = nearestPointIds.Aggregate(new SortedSet<int>(nearestPointIds[bestCamId].nearestNeighbors), (a, b) =>
                    {
                        if (b != null)
                        {
                            a.IntersectWith(b.nearestNeighbors);
                        }

                        return a;
                    });

                    var window = new IntersectionWindow { intersections = set.Values.ToList(), score = variance, nearestPointsSetUnionLength = intersectedPointIds.Count, distanceToRay = distanceToRays, nearestNeighborsPointId = nearestPointIds.Select(x => x?.nearestNeighbors).ToArray(), nearestNeighborsRayId = nearestPointIds.Select((x, u) => x?.nearestNeighbors.Select(y => res[y].pointIds.First(z => z.camId == u).rayId).ToList()).ToArray(), intersectedNeighborsRayId = Enumerable.Range(0, cameraRays.Length).Select(x => intersectedPointIds.Where(y => res[y].pointIds.Any(u => u.camId == x)).Select(y => res[y].pointIds.First(u => u.camId == x).rayId).ToList()).ToArray() /*, availablePoints = nearestPointIds.Select(x => x?.availablePoints) */};
                    if (intersectionWindows == null)
                    {
                        intersectionWindows = new List<IntersectionWindow> { window };
                    }
                    else
                    {
                        intersectionWindows.Add(window);
                    }
                }

                if (intersectionWindows != null)
                {
                    intersections.Add(new RayIntersectionWindows { windows = intersectionWindows, rayId = ray1_id });
                }
            }

            if (intersections.Count == 0)
            {
                break;
            }

            IntersectionWindow bestIntersectionWindow = null;

            intersections = intersections
                .OrderByDescending(x => x.windows.Max(y => y.nearestPointsSetUnionLength))
                .ThenByDescending(x => x.windows.Max(y => y.intersections.Count))
                .ThenByDescending(x => -x.windows.Count).ToList();

            var bestCount = intersections[0].windows.Count;
            var bestWidth = float.MaxValue;
            var bestRay = -1;
            var bestInWindowCount = 0;

            foreach (var intersection in intersections)
            {
                if (intersection.windows.Count > bestCount)
                {
                    break;
                }

                foreach (var window in intersection.windows)
                {
                    if (res.Count > k && window.nearestPointsSetUnionLength < kMin)
                    {
                        continue;
                    }

                    if (bestInWindowCount > window.intersections.Count)
                    {
                        continue;
                    }

                    if (bestInWindowCount < window.intersections.Count)
                    {
                        bestWidth = float.MaxValue;
                        bestInWindowCount = window.intersections.Count;
                    }

                    if (window.score < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.score;
                    }
                }
            }

            if (bestRay < 0)
            {
                break;
            }

            var point = Vector3.zero;
            var pointIds = new List<RayIndex>();
            pointIds.Add(cameraRays[bestCamId][bestRay].pointIndex);
            visitedRays.Add(new RayIndex(bestCamId, bestRay));

            foreach (var intersection in bestIntersectionWindow.intersections)
            {
                point += intersection.segment.point;
                pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
            }

            point /= bestIntersectionWindow.intersections.Count;

            var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray(), distanceToRay = bestIntersectionWindow.distanceToRay, score = bestWidth, nearestNeighborsRayId = bestIntersectionWindow.nearestNeighborsRayId, nearestNeighborsPointId = bestIntersectionWindow.nearestNeighborsPointId, intersectedNeighborsRayId = bestIntersectionWindow.intersectedNeighborsRayId};

            res.Add(foundPoint);
        }

        return res.ToArray();
    }

    /*
     * То же самое что и предыдущий,
     * Выбираем группы в которых наибольшее количество пересечений.
     * Из них группы луч в которых имеет наименьшее количество групп.
     * Из них группы лучи в которых пересекаются лучше всего.
     */
    public static FoundPoint[] FindNew(List<IndexedRay>[] cameraRays, float treshold)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        while (true)
        {
            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();

            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        if (segment.length < treshold)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                List<IntersectionWindow> intersectionWindows = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    var middle = Vector3.zero;

                    middle += rayIntersections[i].segment.s1 + rayIntersections[i].segment.s2;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        middle += rayIntersections[j].segment.s1;
                        middle += rayIntersections[j].segment.s2;
                        ++j;
                    }

                    middle /= set.Count * 2;

                    var variance = 0.0f;

                    foreach (var intersection in set.Values)
                    {
                        variance += Mathf.Pow((middle - intersection.segment.s1).magnitude, 2);
                        variance += Mathf.Pow((middle - intersection.segment.s2).magnitude, 2);
                    }

                    variance = Mathf.Sqrt(variance / set.Count * 2);

                    var window = new IntersectionWindow { intersections = set.Values.ToList(), score = variance };
                    if (intersectionWindows == null)
                    {
                        intersectionWindows = new List<IntersectionWindow> { window };
                    }
                    else
                    {
                        intersectionWindows.Add(window);
                    }
                }

                if (intersectionWindows != null)
                {
                    intersections.Add(new RayIntersectionWindows { windows = intersectionWindows, rayId = ray1_id });
                }
            }

            if (intersections.Count == 0)
            {
                break;
            }

            IntersectionWindow bestIntersectionWindow = null;

            intersections = intersections
                .OrderByDescending(x => x.windows.Select(y => y.intersections.Count).Max())
                .ThenByDescending(x => -x.windows.Count).ToList();

            var bestCount = intersections[0].windows.Count;
            var bestWidth = float.MaxValue;
            var bestRay = -1;
            var bestInWindowCount = 0;

            foreach (var intersection in intersections)
            {
                if (intersection.windows.Count > bestCount)
                {
                    break;
                }

                foreach (var window in intersection.windows)
                {
                    if (bestInWindowCount > window.intersections.Count)
                    {
                        continue;
                    }

                    if (bestInWindowCount < window.intersections.Count)
                    {
                        bestWidth = float.MaxValue;
                        bestInWindowCount = window.intersections.Count;
                    }

                    if (window.score < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.score;
                    }
                }
            }

            var point = Vector3.zero;
            var pointIds = new List<RayIndex>();
            pointIds.Add(cameraRays[bestCamId][bestRay].pointIndex);
            visitedRays.Add(new RayIndex(bestCamId, bestRay));

            foreach (var intersection in bestIntersectionWindow.intersections)
            {
                point += intersection.segment.point;
                pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
            }

            point /= bestIntersectionWindow.intersections.Count;

            var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray(), score = bestWidth };

            res.Add(foundPoint);
        }

        return res.ToArray();
    }

    /*
     * Для каждого луча из камеры с наибольшим количеством лучей строим проекции всех остальных лучей со всех остальных камер.
     * Бежим по проекциям, формируем группы точек.
     * Теперь выбираем луч, в котором меньше всего групп.
     * Из групп сформированных для этого луча, выбираем группы с наибольшим количеством пересечений.
     * Их оставшихся групп выбираем группу с наименьшей шириной.
     * Все лучи в группе помечаем как найденные и больше не рассматриваем их.
     */
    public static FoundPoint[] Find(List<IndexedRay>[] cameraRays, float treshold)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        while (true)
        {
            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();
            
            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray, 
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        if (segment.length < treshold)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                List<IntersectionWindow> bestIntersectionWindows = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    float width = rayIntersections[i].segment.length;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        width = rayIntersections[j].distance - rayIntersections[i].distance;
                        ++j;
                    }

                    if (bestIntersectionWindows == null || bestIntersectionWindows[0].intersections.Count <= set.Count)
                    {
                        var window = new IntersectionWindow { intersections = set.Values.ToList(), score = width };
                        if (bestIntersectionWindows != null && bestIntersectionWindows[0].intersections.Count == set.Count)
                        {
                            bestIntersectionWindows.Add(window);
                        }
                        bestIntersectionWindows = new List<IntersectionWindow> { window };
                    }
                }

                if (bestIntersectionWindows != null)
                {
                    intersections.Add(new RayIntersectionWindows { windows = bestIntersectionWindows, rayId = ray1_id });
                }
            }

            if (intersections.Count == 0)
            {
                break;
            }

            IntersectionWindow bestIntersectionWindow = null;

            intersections.Sort((x, y) => x.windows.Count.CompareTo(y.windows.Count));

            var bestCount = intersections[0].windows.Count;
            var bestWidth = float.MaxValue;
            var bestRay = 0;
            var bestInWindowCount = 0;

            foreach (var intersection in intersections)
            {
                if (intersection.windows.Count > bestCount)
                {
                    break;
                }

                foreach (var window in intersection.windows)
                {
                    if (bestInWindowCount > window.intersections.Count)
                    {
                        continue;
                    }

                    if (bestInWindowCount < window.intersections.Count)
                    {
                        bestWidth = float.MaxValue;
                        bestInWindowCount = window.intersections.Count;
                    }

                    if (window.score < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.score;
                    }
                }
            }

            var point = Vector3.zero;
            var pointIds = new List<RayIndex>();
            pointIds.Add(cameraRays[bestCamId][bestRay].pointIndex);
            visitedRays.Add(new RayIndex(bestCamId, bestRay));

            foreach (var intersection in bestIntersectionWindow.intersections)
            {
                point += intersection.segment.point;
                pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
            }

            point /= bestIntersectionWindow.intersections.Count;

            var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray() };

            res.Add(foundPoint);
        }

        return res.ToArray();
    }

    public static FoundPoint[] FindOld(List<IndexedRay>[] cameraRays, float treshold)
    {
        var visitedRays = new HashSet<RayIndex>();

        var res = new List<FoundPoint>();

        bool somethingChanged = true;

        while (true)
        {
            if (!somethingChanged)
            {
                break;
            }

            somethingChanged = false;

            var bestCamId = getBestCameraId(cameraRays, visitedRays);

            if (bestCamId < 0)
            {
                break;
            }

            var intersections = new List<RayIntersectionWindows>();

            for (var ray1_id = 0; ray1_id < cameraRays[bestCamId].Count; ++ray1_id)
            {
                if (visitedRays.Contains(new RayIndex(bestCamId, ray1_id)))
                {
                    continue;
                }

                var rayIntersections = new List<RayIntersection>();

                for (var secondCamId = 0; secondCamId < cameraRays.Length; secondCamId++)
                {
                    if (secondCamId == bestCamId)
                    {
                        continue;
                    }

                    for (var ray2_id = 0; ray2_id < cameraRays[secondCamId].Count; ray2_id++)
                    {
                        if (visitedRays.Contains(new RayIndex(secondCamId, ray2_id)))
                        {
                            continue;
                        }

                        var segment = findRayIntersection(
                            cameraRays[bestCamId][ray1_id].ray,
                            cameraRays[secondCamId][ray2_id].ray
                        );

                        if (segment.length < treshold)
                        {
                            rayIntersections.Add(new RayIntersection(
                                secondCamId,
                                ray2_id,
                                Vector3.Distance(segment.s1, cameraRays[bestCamId][ray1_id].ray.origin),
                                segment,
                                cameraRays[bestCamId][ray1_id].pointIndex,
                                cameraRays[secondCamId][ray2_id].pointIndex
                            ));
                        }
                    }
                }

                rayIntersections.Sort((x, y) => x.distance.CompareTo(y.distance));

                IntersectionWindow bestIntersectionWindow = null;

                for (var i = 0; i < rayIntersections.Count; i++)
                {
                    var j = i + 1;
                    var set = new Dictionary<int, RayIntersection>();
                    set[rayIntersections[i].camId] = rayIntersections[i];

                    float last_distance = rayIntersections[i].distance;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].camId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].camId] = rayIntersections[j];
                        last_distance = rayIntersections[j].distance;
                        ++j;
                    }

                    float width = last_distance - rayIntersections[i].distance;

                    if (bestIntersectionWindow == null || bestIntersectionWindow.intersections.Count <= set.Count)
                    {
                        if (bestIntersectionWindow != null && bestIntersectionWindow.intersections.Count == set.Count && bestIntersectionWindow.score > width)
                        {
                            continue;
                        }
                        bestIntersectionWindow = new IntersectionWindow { intersections = set.Values.ToList(), score = width };
                    }
                }

                if (bestIntersectionWindow == null)
                {
                    continue;
                }

                var point = Vector3.zero;
                var pointIds = new List<RayIndex>();
                pointIds.Add(cameraRays[bestCamId][ray1_id].pointIndex);
                visitedRays.Add(new RayIndex(bestCamId, ray1_id));

                foreach (var intersection in bestIntersectionWindow.intersections)
                {
                    point += intersection.segment.point;
                    pointIds.Add(cameraRays[intersection.camId][intersection.rayId].pointIndex);
                    visitedRays.Add(new RayIndex(intersection.camId, intersection.rayId));
                }

                point /= bestIntersectionWindow.intersections.Count;

                var foundPoint = new FoundPoint { point = point, pointIds = pointIds.ToArray() };

                res.Add(foundPoint);
                somethingChanged = true;

                somethingChanged = true;
            }
        }

        return res.ToArray();
    }
}
