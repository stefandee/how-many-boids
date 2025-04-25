using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class BoidsJobsSimulation : MonoBehaviour
{
    private struct Boid
    {
        public Vector3 LocalPosition;
        public Vector3 Velocity;

        public void Change(Vector3 pos, Vector3 v)
        {
            LocalPosition = pos;
            Velocity = v;
        }
    }

    private struct BoidsParallelJob : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<Boid> boids;

        public NativeArray<Boid> result;

        [ReadOnly]
        public Vector3 currentFlockCenter;

        [ReadOnly]
        public Vector3 currentFlockVelocity;

        [ReadOnly]
        public float deltaTime;

        [ReadOnly]
        public Vector3 VolumeBounds;

        [ReadOnly]
        public float MaxVelocity;

        [ReadOnly]
        public float MassCenterFactorA;

        [ReadOnly]
        public float MinBoidDistance;

        [ReadOnly]
        public float MatchVelocityFactor;

        [ReadOnly]
        public float BoundsBounceFactor;

        [ReadOnly]
        public float AvoidanceFactor;

        /// <summary>
        /// It's not optimal to use an if inside loops, there should be
        /// 2 different jobs instead.
        /// </summary>
        [ReadOnly]
        public bool FollowTheLeader;

        public void Execute(int i)
        {
            var v1 = FollowTheLeader && i == 0 ? Vector3.zero : Rule1a(i);
            var v2 = Rule2(i);
            var v3 = FollowTheLeader && i == 0 ? Vector3.zero : Rule3(i);
            var v4 = Rule4(i);

            // limit velocity
            var b = boids[i];

            b.Velocity = LimitVelocity(b.Velocity + v1 + v2 + v3 + v4);

            b.LocalPosition += deltaTime * b.Velocity;

            result[i] = b;
        }

        private Vector3 Rule1a(int boidIndex)
        {
            Vector3 center = FollowTheLeader ? boids[0].LocalPosition : (currentFlockCenter - boids[boidIndex].LocalPosition) / (boids.Length - 1);

            return (center - boids[boidIndex].LocalPosition).normalized * MassCenterFactorA;
        }

        private Vector3 Rule2(int boidIndex)
        {
            Vector3 result = Vector3.zero;
            Vector3 bLocalPosition = boids[boidIndex].LocalPosition;

            for (var i = 0; i < boids.Length; i++)
            {
                if (i != boidIndex /*&& Vector3.Distance(bx.Transform.localPosition, bLocalPosition) < MIN_BOID_DISTANCE*/)
                {
                    var pos = boids[i].LocalPosition;
                    var dx = pos.x - bLocalPosition.x;
                    var dy = pos.y - bLocalPosition.y;
                    var dz = pos.z - bLocalPosition.z;

                    if (dx * dx + dy * dy + dz * dz < MinBoidDistance * MinBoidDistance)
                    {
                        result -= (boids[i].LocalPosition - bLocalPosition).normalized;
                    }
                }
            }

            return result * AvoidanceFactor;
        }

        private Vector3 Rule3(int boidIndex)
        {
            Vector3 velocity = FollowTheLeader ? boids[0].Velocity : (currentFlockVelocity - boids[boidIndex].Velocity) / (boids.Length - 1);

            return (velocity - boids[boidIndex].Velocity).normalized * MatchVelocityFactor;
        }

        private Vector3 Rule4(int boidIndex)
        {
            var pos = boids[boidIndex].LocalPosition;

            Vector3 result = Vector3.zero;

            if (pos.x < -VolumeBounds.x)
            {
                result.x = 1;
            }
            else if (pos.x > VolumeBounds.x)
            {
                result.x = -1;
            }

            if (pos.y < -VolumeBounds.y)
            {
                result.y = 1;
            }
            else if (pos.y > VolumeBounds.y)
            {
                result.y = -1;
            }

            if (pos.z < -VolumeBounds.z)
            {
                result.z = 1;
            }
            else if (pos.z > VolumeBounds.z)
            {
                result.z = -1;
            }

            return BoundsBounceFactor * result.normalized;
        }

        private Vector3 LimitVelocity(Vector3 velocity)
        {
            var mag = velocity.sqrMagnitude;

            if (mag > MaxVelocity * MaxVelocity)
            {
                return velocity.normalized * MaxVelocity;
            }

            return velocity;
        }
    }

    #region Simulation Parameters
    [Header("Boids Params")]
    public GameObject Prefab;
    public int Amount = 20;
    public float MaxVelocity = 25f;
    public float MassCenterFactorA = 0.50f;
    public float MinBoidDistance = 10f;
    public float AvoidanceFactor = 0.25f;
    public float MatchVelocityFactor = 0.25f;
    public float BoundsBounceFactor = 5f;
    public bool FollowTheLeader = false;

    [Header("Bounds")]
    public Vector3 InitialBounds = new(50, 50, 50);

    public Vector3 VolumeBounds = new(50, 50, 50);

    [Header("Camera")]
    public Camera Camera;

    public float CameraDistance = 20;

    public bool CameraFollowsFirstBoid = false;
    #endregion // Simulation Parameters

    private NativeArray<Boid> boids;

    private Transform[] boidsTransforms;

    private Boid boidToFollow;

    private Vector3 currentFlockCenter;

    private Vector3 currentFlockVelocity;

    #region Unity Events
    void Start()
    {
        InitializeBoids();
    }

    void Update()
    {
        MoveBoids();

        // update the gameobjects
        for (var i = 0; i < boids.Length; i++)
        {
            boidsTransforms[i].localPosition = boids[i].LocalPosition;
        }

        if (CameraFollowsFirstBoid)
        {
            FocusCamera();
        }
    }

    void OnDestroy()
    {
        boids.Dispose();
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
        boids = new NativeArray<Boid>(Amount, Allocator.Persistent);
        boidsTransforms = new Transform[Amount];

        var xMaxBounds = Mathf.Abs(InitialBounds.x);
        var yMaxBounds = Mathf.Abs(InitialBounds.y);
        var zMaxBounds = Mathf.Abs(InitialBounds.z);

        currentFlockCenter = Vector3.zero;
        currentFlockVelocity = Vector3.zero;

        for (int i = 0; i < Amount; i++)
        {
            var boidGameObject = Instantiate(Prefab, transform);

            boidsTransforms[i] = boidGameObject.transform;

            boids[i] = new Boid()
            {
                LocalPosition = new Vector3(
                    Random.Range(-xMaxBounds, xMaxBounds),
                    Random.Range(-yMaxBounds, yMaxBounds),
                    Random.Range(-zMaxBounds, zMaxBounds)
                ),
                Velocity = new Vector3(
                    Random.Range(-1f, 1f),
                    Random.Range(-1f, 1f),
                    Random.Range(-1f, 1f)
                ).normalized * Random.Range(MaxVelocity * 0.5f, MaxVelocity)
            };

            currentFlockCenter += boids[i].LocalPosition;
            currentFlockVelocity += boids[i].Velocity;
        }

        boidToFollow = boids[0];
    }

    private void MoveBoids()
    {
        // ComputeFlockCenterAndVelocity();

        // init the parallel jobs
        BoidsParallelJob job = new();

        // allocate the results
        var result = new NativeArray<Boid>(boids.Length, Allocator.TempJob);

        job.result = result;
        job.boids = boids;
        job.currentFlockCenter = currentFlockCenter;
        job.currentFlockVelocity = currentFlockVelocity;
        job.deltaTime = Time.deltaTime;
        job.VolumeBounds = VolumeBounds;
        job.MaxVelocity = MaxVelocity;
        job.MassCenterFactorA = MassCenterFactorA;
        job.MatchVelocityFactor = MatchVelocityFactor;
        job.MinBoidDistance = MinBoidDistance;
        job.AvoidanceFactor = AvoidanceFactor;
        job.BoundsBounceFactor = BoundsBounceFactor;
        job.FollowTheLeader = FollowTheLeader;

        JobHandle handle = job.Schedule(boids.Length, 1);

        handle.Complete();

        // save the results for the next run
        // result.CopyTo(boids);
        currentFlockCenter = Vector3.zero;
        currentFlockVelocity = Vector3.zero;

        for (var i = 0; i < result.Length; i++)
        {
            boids[i] = result[i];

            // compute the flock center and velocity
            currentFlockCenter += boids[i].LocalPosition;
            currentFlockVelocity += boids[i].Velocity;
        }

        result.Dispose();
    }
}
