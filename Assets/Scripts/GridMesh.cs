using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public class GridMesh : MonoBehaviour
{
    public int GridSize;

    void Awake()
    {
        MeshFilter filter = gameObject.GetComponent<MeshFilter>();        
        var mesh = new Mesh();
        var verticies = new List<Vector3>();

        var indicies = new List<int>();
        for (var i = 0; i <= GridSize; i++)
        {
            verticies.Add(new Vector3(i - GridSize / 2, 0, -GridSize / 2));
            verticies.Add(new Vector3(i - GridSize / 2, 0, GridSize / 2));

            indicies.Add(4 * i + 0);
            indicies.Add(4 * i + 1);

            verticies.Add(new Vector3(-GridSize / 2, 0, i - GridSize / 2));
            verticies.Add(new Vector3(GridSize / 2, 0, i - GridSize / 2));

            indicies.Add(4 * i + 2);
            indicies.Add(4 * i + 3);
        }

        mesh.vertices = verticies.ToArray(); 
        mesh.SetIndices(indicies.ToArray(), MeshTopology.Lines, 0);
        filter.mesh = mesh;

        MeshRenderer meshRenderer = gameObject.GetComponent<MeshRenderer>();
        meshRenderer.material = new Material(Shader.Find("Sprites/Default"))
        {
            color = Color.white
        };
    }
}
