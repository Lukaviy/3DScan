using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
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

    public static List<IndexedRay>[] Load(string path, int count)
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
}
