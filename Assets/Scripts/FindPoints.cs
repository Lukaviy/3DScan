using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public struct FoundPoint
{
    public Vector3 point;
    public int[] pointIds;
}

public struct IndexedRay
{
    public Ray ray;
    public int pointIndex;
}

public static class FindPoints { 

    private struct RayIndex
    {
        int camId;
        int rayId;

        public RayIndex(int cam_id, int ray_id)
        {
            camId = cam_id;
            rayId = ray_id;
        }
    }

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

        public RayIntersection(int camId_, int rayId_, float distance_, Segment segment_)
        {
            camId = camId_;
            rayId = rayId_;
            distance = distance_;
            segment = segment_;
        }
    }

    private class IntersectionWindow
    {
        public List<RayIntersection> intersections;
        public float width;
    }

    private class RayIntersectionWindows
    {
        public int rayId;
        public List<IntersectionWindow> windows;
    }

    public static FoundPoint[] Find(List<IndexedRay>[] cameraRays, float treshold)
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
                                segment
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
                    set[rayIntersections[i].rayId] = rayIntersections[i];

                    float last_distance = rayIntersections[i].distance;

                    while (j < rayIntersections.Count && rayIntersections[j].distance - rayIntersections[i].distance < treshold)
                    {
                        if (set.ContainsKey(rayIntersections[j].rayId))
                        {
                            ++j;
                            continue;
                        }
                        set[rayIntersections[j].rayId] = rayIntersections[j];
                        last_distance = rayIntersections[j].distance;
                        ++j;
                    }

                    float width = last_distance - rayIntersections[i].distance;

                    if (bestIntersectionWindows == null || bestIntersectionWindows[0].intersections.Count <= set.Count)
                    {
                        var window = new IntersectionWindow { intersections = set.Values.ToList(), width = width };
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

            foreach (var intersection in intersections)
            {
                if (intersection.windows.Count > bestCount)
                {
                    break;
                }

                foreach (var window in intersection.windows)
                {
                    if (window.width < bestWidth)
                    {
                        bestIntersectionWindow = window;
                        bestRay = intersection.rayId;
                        bestWidth = window.width;
                    }
                }
            }

            var point = Vector3.zero;
            var pointIds = new List<int>();
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
            somethingChanged = true;
        }

        return res.ToArray();
    }
}
