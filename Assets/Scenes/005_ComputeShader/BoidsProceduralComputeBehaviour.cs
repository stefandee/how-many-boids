using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class BoidsProceduralComputeBehaviour : MonoBehaviour
{
    private const string KERNEL_SIMULATION = "Boids";

    #region Simulation Parameters
    [Header("Boids Params")]
    public int BoidsAmount = 20;
    public float MaxForce = 1.3f;
    public float MaxSpeed = 25f;
    public float DesiredSeparation = 2.0f;
    public float NeighborDist = 5f;

    [Header("Boids Rendering")]
    public ComputeShader Shader;
    public Mesh Mesh;
    public Material Material;
    public float Scale = 1;

    [Header("Bounds")]
    public Vector3 InitialBounds = new(50, 50, 50);

    public Vector3 VolumeBounds = new(50, 50, 50);

    [Header("Camera")]
    public Camera Camera;

    public float CameraDistance = 20;

    public bool CameraFollowsFirstBoid = false;
    #endregion // Simulation Parameters

    //internal data
    ComputeBuffer positionsBuffer;
    ComputeBuffer velocitiesBuffer;

    ComputeBuffer meshTriangles;
    ComputeBuffer meshPositions;
    int kernel;
    uint threadGroupSize;
    Bounds bounds;
    int threadGroups;

    void Start()
    {
        //program we're executing
        kernel = Shader.FindKernel(KERNEL_SIMULATION);
        Shader.GetKernelThreadGroupSizes(kernel, out threadGroupSize, out _, out _);
        
        //amount of thread groups we'll need to dispatch
        threadGroups = (int) ((BoidsAmount + (threadGroupSize - 1)) / threadGroupSize);

        //gpu buffer for the sphere positions
        positionsBuffer = new ComputeBuffer(BoidsAmount, sizeof(float) * 3);
        velocitiesBuffer = new ComputeBuffer(BoidsAmount, sizeof(float) * 3);

        PopulateComputeShaderBuffers();

        //gpu buffers for the mesh
        int[] triangles = Mesh.triangles;
        meshTriangles = new ComputeBuffer(triangles.Length, sizeof(int));
        meshTriangles.SetData(triangles);
        Vector3[] positions = Mesh.vertices.Select(p => p * Scale).ToArray(); //adjust scale here
        meshPositions = new ComputeBuffer(positions.Length, sizeof(float) * 3);
        meshPositions.SetData(positions);

        //give data to shaders
        Shader.SetBuffer(kernel, "Positions", positionsBuffer);
        Shader.SetBuffer(kernel, "Velocities", velocitiesBuffer);
        Shader.SetFloats("VolumeBounds", new float[3] { VolumeBounds.x, VolumeBounds.y, VolumeBounds.z });

        Shader.SetFloat("maxforce", MaxForce);
        Shader.SetFloat("maxspeed", MaxSpeed);
        Shader.SetFloat("desiredseparation", DesiredSeparation);
        Shader.SetFloat("desiredseparationSq", DesiredSeparation * DesiredSeparation);
        Shader.SetFloat("neighbordist", NeighborDist);
        Shader.SetFloat("neighbordistSq", NeighborDist * NeighborDist);

        Material.SetBuffer("SphereLocations", positionsBuffer);
        Material.SetBuffer("Triangles", meshTriangles);
        Material.SetBuffer("Positions", meshPositions);
        
        //bounds for frustum culling (20 is a magic number (radius) from the compute shader)
        bounds = new Bounds(Vector3.zero, Vector3.one * 20);
    }

    void Update()
    {
        //calculate positions
        Shader.SetFloat("DeltaTime", Time.deltaTime);
        Shader.Dispatch(kernel, threadGroups, 1, 1);
        
        //draw result
        Graphics.DrawProcedural(Material, bounds, MeshTopology.Triangles, meshTriangles.count, BoidsAmount);
    }

    void OnDestroy()
    {
        positionsBuffer.Dispose();
        meshTriangles.Dispose();
        meshPositions.Dispose();
    }

    private void PopulateComputeShaderBuffers()
    {
        float[] positions = new float[BoidsAmount * 3];
        float[] velocities = new float[BoidsAmount * 3];

        var xMaxBounds = Mathf.Abs(InitialBounds.x);
        var yMaxBounds = Mathf.Abs(InitialBounds.y);
        var zMaxBounds = Mathf.Abs(InitialBounds.z);

        for (int i = 0; i < BoidsAmount; i++)
        {
            var idx = i * 3;

            positions[idx] = UnityEngine.Random.Range(-xMaxBounds, xMaxBounds);
            positions[idx + 1] = UnityEngine.Random.Range(-yMaxBounds, yMaxBounds);
            positions[idx + 2] = UnityEngine.Random.Range(-zMaxBounds, zMaxBounds);

            var randomVelocity = UnityEngine.Random.onUnitSphere * UnityEngine.Random.Range(5f, 10f);

            velocities[idx] = randomVelocity.x;
            velocities[idx + 1] = randomVelocity.y;
            velocities[idx + 2] = randomVelocity.z;
        }

        positionsBuffer.SetData(positions);
        velocitiesBuffer.SetData(velocities);
    }
}
