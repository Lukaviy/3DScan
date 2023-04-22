using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChessBoard : MonoBehaviour
{
    public GameObject PointPrefab;

    public List<Vector3> Points { get; private set; }

    public int XSize;
    public int YSize;

    public float ChessSize;

    void Start()
    {
        Points = new List<Vector3>();

        for (var x = 0; x < XSize; x++)
        {
            for (var y = 0; y < YSize; y++)
            {
                var objectPoints = new Vector3(x, y, 0) * ChessSize;
                var pos = transform.TransformPoint(new Vector3(x, y, 0) * ChessSize);
                Points.Add(objectPoints);

                Instantiate(PointPrefab, pos, Quaternion.identity, transform);
            }
        }
    }
}
