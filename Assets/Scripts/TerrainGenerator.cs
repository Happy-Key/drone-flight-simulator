using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TerrainGenerator : MonoBehaviour
{
    [SerializeField] private GameObject terrain;
    
    [SerializeField] private int terrainCells;
    [SerializeField] private float cellSize;
    [SerializeField] private float noiseFrequency;
    [SerializeField] private float maxHeight;

    void Start()
    {
        Vector3[] vertices = new Vector3[terrainCells * terrainCells];
        Vector2[] uvs = new Vector2[vertices.Length];
        int[] triangles = new int[(terrainCells - 1) * (terrainCells - 1) * 6];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector2 coord = new Vector2(i / terrainCells * cellSize, i % terrainCells * cellSize);
            vertices[i] = new Vector3(coord.x, 0f, coord.y);
            vertices[i].y += maxHeight * Mathf.Pow(Mathf.PerlinNoise(coord.x * noiseFrequency, coord.y * noiseFrequency), 4f) / 2f;
            for (int j = 1; j < 5; j++)
            {
                vertices[i].y += maxHeight * Mathf.PerlinNoise(coord.x * noiseFrequency * Mathf.Pow(2, j), coord.y * noiseFrequency * Mathf.Pow(2, j)) / Mathf.Pow(2, j + 1);
            }
        }
        for (int i = 0; i < uvs.Length; i++)
        {
            uvs[i] = new Vector2(vertices[i].x, vertices[i].z);
        }
        for (int i = 0; i < triangles.Length; i += 6)
        {
            int n = i / 6;
            n = n / (terrainCells - 1) * terrainCells + n % (terrainCells - 1);
            triangles[i] = n;
            triangles[i + 1] = n + 1;
            triangles[i + 2] = n + terrainCells;
            triangles[i + 3] = n + 1;
            triangles[i + 4] = n + terrainCells + 1;
            triangles[i + 5] = n + terrainCells;
        }

        Mesh mesh = terrain.GetComponent<MeshFilter>().mesh;
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.Clear();

        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();

        terrain.transform.position = new Vector3(-(terrainCells - 1) / 2f * cellSize, 0, -(terrainCells - 1) / 2f * cellSize);
    }
}
