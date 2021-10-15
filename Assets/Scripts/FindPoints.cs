using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Threading;


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
    public float[] distanceToRay;
    public float score;
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

public static class FindPoints { 

    private struct Segment
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

    public class IntersectionGroup
    {
        public int[] camRayIndices;
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

                        if (segment.length < treshold)
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
                        camRayIndices = camRayIndices,
                        score = variance
                    });
                }
            }
        }

        return res;
    }

    public static FoundPoint[] GetPoints(List<IndexedRay>[] cameraRays, List<int[]> groups)
    {
        var res = new List<FoundPoint>();

        var pointCloud = new Vector3[cameraRays.Length * cameraRays.Length];

        foreach (var group in groups)
        {
            var i = 0;
            for (var cam1_id = 0; cam1_id < cameraRays.Length; cam1_id++)
            {
                if (group[cam1_id] < 0)
                {
                    continue;
                }

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

            var middle = pointCloud.Take(i).Aggregate((x, y) => x + y);

            var variance = Mathf.Sqrt(pointCloud.Take(i).Select(x => Mathf.Pow((middle - x).magnitude, 2)).Sum() / pointCloud.Length);

            res.Add(new FoundPoint
            {
                point = middle,
                score = variance
            });
        }

        return res.ToArray();
    }

    public static FoundPoint[] FindBasedOnPaper(List<IndexedRay>[] cameraRays, float treshold)
    {
        var groups = FindIntersectionGroups(cameraRays, treshold);

        var matches = EnforceMatchConsistency(groups);

        return GetPoints(cameraRays, matches.consistentGroups);
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
