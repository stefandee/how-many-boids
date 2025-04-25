using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using float3 = Unity.Mathematics.float3;

[BurstCompile(CompileSynchronously = true)]
public struct BurstLocalBoidsParallelJob : IJobParallelFor
{
    public NativeArray<JobBurstBoid> result;

    [ReadOnly]
    public NativeArray<JobBurstBoid> boids;

    [ReadOnly]
    public float deltaTime;

    [ReadOnly]
    public float time;

    [ReadOnly]
    public float noiseOffset;

    [ReadOnly]
    public float3 randomUnitVector;

    [ReadOnly]
    public float3 VolumeBounds;

    [ReadOnly]
    public float maxVelocity;

    [ReadOnly]
    public float massCenterFactor;

    [ReadOnly]
    public float minBoidDistance;

    [ReadOnly]
    public float minBoidDistance2;

    [ReadOnly]
    public float matchVelocityFactor;

    [ReadOnly]
    public float velocityRepulseMagnitude;

    [ReadOnly]
    public float boundsBounceFactor;

    public void Execute(int i)
    {
        var v2 = Rule123(i);
        var v4 = Rule4(i);

        // limit velocity
        var b = boids[i];

        b.Velocity = LimitVelocity(b.Velocity + v4 + v2);

        b.LocalPosition += deltaTime * b.Velocity;

        result[i] = b;
    }

    private float3 Rule123(int boidIndex)
    {
        float3 result = float3.zero;
        float3 localFlockCenter = float3.zero;
        float3 localFlockVelocity = float3.zero;
        float3 bLocalPosition = boids[boidIndex].LocalPosition;
        int neighborsCount = 0;

        float3 pos;
        float dSqr;

        for (var i = 0; i < boids.Length; i++)
        {
            pos = boids[i].LocalPosition;

            dSqr = math.lengthsq(pos - bLocalPosition);

            if (dSqr < minBoidDistance * minBoidDistance)
            {
                neighborsCount++;
                localFlockCenter += pos;
                localFlockVelocity += boids[i].Velocity;

                // separation
                result -= (pos - bLocalPosition) * velocityRepulseMagnitude;
            }
            else if (dSqr < minBoidDistance2 * minBoidDistance2)
            {
                neighborsCount++;
                localFlockCenter += pos;
                localFlockVelocity += boids[i].Velocity;
            }
        }

        var v1 = float3.zero;
        var v2 = float3.zero;

        if (neighborsCount > 1)
        {
            v1 = ((localFlockCenter - bLocalPosition) / (neighborsCount - 1) - bLocalPosition) * massCenterFactor;
            v2 = ((localFlockVelocity - boids[boidIndex].Velocity) / (neighborsCount - 1) - boids[boidIndex].Velocity) * matchVelocityFactor;
        }

        return v1 + v2 + result;
    }

    private float3 Rule4(int boidIndex)
    {
        var pos = boids[boidIndex].LocalPosition;

        float3 result = float3.zero;

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

        return boundsBounceFactor * math.normalizesafe(result);
    }

    private float3 LimitVelocity(float3 velocity)
    {
        var magSqr = math.lengthsq(velocity);

        if (magSqr > maxVelocity * maxVelocity)
        {
            return math.normalizesafe(velocity) * maxVelocity;
        }
        else if (magSqr < 100)
        {
            velocity += math.normalizesafe(velocity) * (maxVelocity * (1f + noiseOffset) / 5f);
        }

        return velocity;
    }
}
