using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Mathematics;

public class BoidsJobsBurstSimulation : MonoBehaviour
{
    #region Simulation Parameters
    [Header("Boids Params")]
    public GameObject Prefab;

    public int Amount = 20;
    public float MaxVelocity = 25f;
    public float MassCenterFactor = 0.04f;
    public float MinBoidDistance = 5f;
    public float MinBoidDistance2 = 10f;
    public float VelocityRepulseMagnitude = 0.5f;
    public float MatchVelocityFactor = 0.02f;
    public float BoundsBounceFactor = 25f;

    [Header("Bounds")]
    public Vector3 InitialBounds = new(50, 50, 50);

    public Vector3 VolumeBounds = new(50, 50, 50);

    [Header("Camera")]
    public Camera Camera;

    public float CameraDistance = 20;

    public bool CameraFollowsFirstBoid = false;
    #endregion // Simulation Parameters

    private NativeArray<JobBurstBoid> boids;

    private Transform[] boidsTransforms;

    private Unity.Mathematics.Random random;

    #region Unity Events
    void Start()
    {
        Debug.Log("Boids Jobs and Burst Simulation Started");
    }

    void OnEnable()
    {
        InitializeBoids();
    }

    private void OnDisable()
    {
        Debug.Log("OnDisable");

        // cleanup
        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        if (boids.Length > 0)
        {
            Debug.Log("Dispose boids NativeArray");
            boids.Dispose();
        }
    }

    void Update()
    {
        MoveBoids();

        // update the gameobject transforms
        for (var i = 0; i < boids.Length; i++)
        {
            boidsTransforms[i].localPosition = new Vector3(boids[i].LocalPosition.x, boids[i].LocalPosition.y, boids[i].LocalPosition.z);
        }

        if (CameraFollowsFirstBoid)
        {
            FocusCamera();
        }
    }

    void OnDestroy()
    {
        Debug.Log("OnDestroy");
        // boids.Dispose();
    }

    #endregion

    /// <summary>
    /// Camera follows the first boid at a distance
    /// </summary>
    private void FocusCamera()
    {
        /*
        // camera looks at the first boid
        Camera.transform.LookAt(boidToFollow.Transform, Vector3.up);

        // camera moves to keep the preferred distance
        // TODO lerp?
        var direction = (Camera.transform.position - boidToFollow.Transform.position).normalized;
        Camera.transform.position = boidToFollow.Transform.position + CameraDistance * direction;
        */
    }

    private void InitializeBoids()
    {
        boids = new NativeArray<JobBurstBoid>(Amount, Allocator.Persistent);
        boidsTransforms = new Transform[Amount];

        var xMaxBounds = Mathf.Abs(InitialBounds.x);
        var yMaxBounds = Mathf.Abs(InitialBounds.y);
        var zMaxBounds = Mathf.Abs(InitialBounds.z);

        random = new Unity.Mathematics.Random((uint)UnityEngine.Random.Range(0, 1851936439));

        float halfMaxVelocity = MaxVelocity * 0.5f;

        for (int i = 0; i < Amount; i++)
        {
            var boidGameObject = Instantiate(Prefab, transform);

            boidsTransforms[i] = boidGameObject.transform;

            boids[i] = new JobBurstBoid()
            {
                LocalPosition = new float3(
                UnityEngine.Random.Range(-xMaxBounds, xMaxBounds),
                UnityEngine.Random.Range(-yMaxBounds, yMaxBounds),
                UnityEngine.Random.Range(-zMaxBounds, zMaxBounds)
                ),
                Velocity = random.NextFloat3Direction() * (random.NextFloat(halfMaxVelocity) + halfMaxVelocity)
            };
        }
    }

    private void MoveBoids()
    {
        // init the parallel jobs
        BurstLocalBoidsParallelJob job = new();

        // allocate the results
        var result = new NativeArray<JobBurstBoid>(boids.Length, Allocator.TempJob);

        job.result = result;
        job.boids = boids;
        job.deltaTime = Time.deltaTime;
        job.time = Time.time;
        job.VolumeBounds = VolumeBounds;

        job.maxVelocity = MaxVelocity;
        job.massCenterFactor = MassCenterFactor;
        job.minBoidDistance = MinBoidDistance;
        job.minBoidDistance2 = MinBoidDistance2;
        job.velocityRepulseMagnitude = VelocityRepulseMagnitude;
        job.matchVelocityFactor = MatchVelocityFactor;
        job.boundsBounceFactor = BoundsBounceFactor;

        // additional randomness
        job.noiseOffset = random.NextFloat(0, 1);
        job.randomUnitVector = random.NextFloat3Direction();

        JobHandle handle = job.Schedule(boids.Length, 1);

        handle.Complete();

        // save the results for the next run
        // result.CopyTo(boids);
        for (var i = 0; i < result.Length; i++)
        {
            boids[i] = result[i];
        }

        result.Dispose();
    }
}
