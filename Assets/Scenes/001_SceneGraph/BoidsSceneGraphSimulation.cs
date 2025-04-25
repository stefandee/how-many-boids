using System;
using UnityEngine;
using static UnityEngine.Random;

public class BoidsSceneGraphSimulation : MonoBehaviour
{
    /// <summary>
    /// TODO this should be struct
    /// </summary>
    private class Boid
    {
        public Transform Transform;
        public Vector3 Velocity;
    }

    #region Simulation Parameters

    [Header("Boids Params")]
    public GameObject Prefab;
    public int Amount = 20;
    public float MaxVelocity = 25f;
    public bool UseGlobalMassCenter = false;
    public float MassCenterFactorA = 0.50f;
    public float MassCenterFactorB = 0.25f;
    public float LocalMassCenterCountFactor = 0.25f;
    public float MinBoidDistance = 10f;
    public float AvoidanceFactor = 0.25f;
    public float MatchVelocityFactor = 0.25f;
    public float BoundsBounceFactor = 5f;

    [Header("Bounds")]
    public Vector3 InitialBounds = new(50, 50, 50);

    public Vector3 VolumeBounds = new(50, 50, 50);

    [Header("Camera")]
    public Camera Camera;

    public float CameraDistance = 20;

    public bool CameraFollowsFirstBoid = false;
    #endregion // Simulation Parameters

    private Boid[] boids;

    private Boid boidToFollow;

    private Vector3 currentCenter;

    private Vector3 currentGroupVelocity;

    void Start()
    {
        InitializeBoids();
    }

    void Update()
    {
        MoveBoids();

        if (CameraFollowsFirstBoid)
        {
            FocusCamera();
        }
    }

    /// <summary>
    /// Camera follows the first boid at a distance
    /// </summary>
    private void FocusCamera()
    {
        // camera looks at the first boid
        Camera.transform.LookAt(boidToFollow.Transform, Vector3.up);

        // camera moves to keep the preferred distance
        // TODO lerp?
        var direction = (Camera.transform.position - boidToFollow.Transform.position).normalized;
        Camera.transform.position = boidToFollow.Transform.position + CameraDistance * direction;
    }

    private void InitializeBoids()
    {
        boids = new Boid[Amount];

        var xMaxBounds = Mathf.Abs(InitialBounds.x);
        var yMaxBounds = Mathf.Abs(InitialBounds.y);
        var zMaxBounds = Mathf.Abs(InitialBounds.z);

        for (int i = 0; i < Amount; i++)
        {
            var boidGameObject = Instantiate(Prefab, transform);

            boids[i] = new Boid()
            {
                Transform = boidGameObject.transform,
                Velocity = new Vector3(
                    Range(-1f, 1f),
                    UnityEngine.Random.Range(-1f, 1f),
                    UnityEngine.Random.Range(-1f, 1f)
                ).normalized * UnityEngine.Random.Range(MaxVelocity * 0.5f, MaxVelocity)
            };

            boids[i].Transform.localPosition = new Vector3(
                UnityEngine.Random.Range(-xMaxBounds, xMaxBounds),
                UnityEngine.Random.Range(-yMaxBounds, yMaxBounds),
                UnityEngine.Random.Range(-zMaxBounds, zMaxBounds)
                );
        }

        boidToFollow = boids[0];
    }

    private void MoveBoids()
    {
        ComputeFlockCenterAndVelocity();

        foreach (var b in boids)
        {
            var v1 = UseGlobalMassCenter ? Rule1a(b) : Rule1b(b);
            var v2 = Rule2(b);
            var v3 = Rule3(b);
            var v4 = Rule4(b);

            // limit velocity
            b.Velocity = LimitVelocity(b.Velocity + v1 + v2 + v3 + v4);

            b.Transform.localPosition += Time.deltaTime * b.Velocity;
        }
    }

    /// <summary>
    /// Boids try to fly towards the center of mass of all boids
    /// </summary>
    /// <param name="b"></param>
    /// <returns>Vector3 as velocity change</returns>
    private Vector3 Rule1a(Boid b)
    {
        Vector3 center = (currentCenter - b.Transform.localPosition) / (boids.Length - 1);

        return (center - b.Transform.localPosition) * MassCenterFactorA;
    }

    /// <summary>
    /// Boids try to fly towards the center of mass of closest N boids
    /// 
    /// This is very very slow.
    /// </summary>
    /// <param name="b"></param>
    /// <returns>Vector3 as velocity change</returns>
    private Vector3 Rule1b(Boid b)
    {
        // compute the local center of mass
        Vector3 center = Vector3.zero;

        var localPos = b.Transform.localPosition;

        Array.Sort<Boid>(boids, (x, y) => Vector3.Distance(x.Transform.localPosition, localPos).CompareTo(Vector3.Distance(y.Transform.localPosition, localPos)));

        int count = Mathf.Min((int)(boids.Length * LocalMassCenterCountFactor), boids.Length);

        for (var i = 1; i < count; i++)
        {
            if (boids[i] != b)
            {
                center += boids[i].Transform.localPosition;
            }
        }

        center /= (count - 1);

        return (center - localPos) * MassCenterFactorB;
    }


    /// <summary>
    /// Boids try to keep a distance from neighboring boids
    /// </summary>
    /// <param name="b"></param>
    /// <returns>Vector3 as velocity change</returns>
    private Vector3 Rule2(Boid b)
    {
        Vector3 result = Vector3.zero;
        Vector3 bLocalPosition = b.Transform.localPosition;

        foreach (var bx in boids)
        {
            if (bx != b /*&& Vector3.Distance(bx.Transform.localPosition, bLocalPosition) < MIN_BOID_DISTANCE*/)
            {
                var pos = bx.Transform.localPosition;
                var dx = pos.x - bLocalPosition.x;
                var dy = pos.y - bLocalPosition.y;
                var dz = pos.z - bLocalPosition.z;

                if (dx * dx + dy * dy + dz * dz < MinBoidDistance * MinBoidDistance)
                {
                    result -= (bx.Transform.localPosition - bLocalPosition) * AvoidanceFactor;
                }
            }
        }

        return result;
    }

    /// <summary>
    /// Boids try to match velocity of near-by boids.
    /// </summary>
    /// <param name="b"></param>
    /// <returns></returns>
    private Vector3 Rule3(Boid b)
    {
        Vector3 velocity = (currentGroupVelocity - b.Velocity) / (boids.Length - 1);

        return (velocity - b.Velocity) * MatchVelocityFactor;
    }

    private Vector3 Rule4(Boid b)
    {
        var pos = b.Transform.localPosition;

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

    private void ComputeFlockCenterAndVelocity()
    {
        currentCenter = Vector3.zero;
        currentGroupVelocity = Vector3.zero;

        foreach (var b in boids)
        {
            currentCenter += b.Transform.localPosition;
            currentGroupVelocity += b.Velocity;
        }
    }
}
