using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using float3 = Unity.Mathematics.float3;

// heavily modified and optimized of code from https://processing.org/examples/flocking.html
[BurstCompile(CompileSynchronously = true)]
public struct BurstLocalBoidsParallelJobAlternate : IJobParallelFor
{
    public NativeArray<JobBurstBoid> result;

    [ReadOnly]
    public NativeArray<JobBurstBoid> boids;

    [ReadOnly]
    public float deltaTime;

    [ReadOnly]
    public float3 VolumeBounds;

    [ReadOnly]
    public float MaxForce;

    [ReadOnly]
    public float MaxSpeed;

    [ReadOnly]
    public float DesiredSeparation;

    [ReadOnly]
    public float NeighborDist;


    public void Execute(int i)
    {
        var b = boids[i];

        b.Velocity += SeparateAlignCohesion(i);
        b.LocalPosition += b.Velocity * deltaTime;
        b.LocalPosition = Wraparound(b.LocalPosition);

        result[i] = b;
    }

    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    float3 Seek(float3 target, float3 position, float3 velocity)
    {
        // A vector pointing from the position to the target
        float3 desired = target - position;

        // Scale to maximum speed
        desired = math.normalizesafe(desired);
        desired *= MaxSpeed;

        // Steering = Desired minus Velocity
        float3 steer = desired - velocity;

        steer = Limit(steer, MaxForce);

        return steer;
    }

    float3 SoftBounce(float3 velocity, float3 pos)
    {
        var steer = float3.zero;

        var x = pos.x;
        var y = pos.y;
        var z = pos.z;

        if (x < -VolumeBounds.x)
        {
            steer = Limit(math.reflect(velocity, math.right()) - velocity, MaxForce);
        }
        else if (x > VolumeBounds.x)
        {
            steer = Limit(math.reflect(velocity, math.left()) - velocity, MaxForce);
        }

        if (y < -VolumeBounds.y)
        {
            steer = Limit(math.reflect(velocity, math.up()) - velocity, MaxForce);
        }
        else if (y > VolumeBounds.y)
        {
            steer = Limit(math.reflect(velocity, math.down()) - velocity, MaxForce);
        }

        if (z < -VolumeBounds.z)
        {
            steer = Limit(math.reflect(velocity, math.forward() - velocity), MaxForce);
        }
        else if (z > VolumeBounds.z)
        {
            steer = Limit(math.reflect(velocity, math.back() - velocity), MaxForce);
        }

        // steer behaviour (otherwise this will bounce instantaneously)
        return steer;
    }

    float3 HardBounce(float3 velocity, float3 pos)
    {
        var steer = velocity;

        var x = pos.x;
        var y = pos.y;
        var z = pos.z;

        if (x < -VolumeBounds.x)
        {
            steer = Limit(math.reflect(velocity, math.right()), MaxSpeed);
        }
        else if (x > VolumeBounds.x)
        {
            steer = Limit(math.reflect(velocity, math.left()), MaxSpeed);
        }

        if (y < -VolumeBounds.y)
        {
            steer = Limit(math.reflect(velocity, math.up()), MaxSpeed);
        }
        else if (y > VolumeBounds.y)
        {
            steer = Limit(math.reflect(velocity, math.down()), MaxSpeed);
        }

        if (z < -VolumeBounds.z)
        {
            steer = Limit(math.reflect(velocity, math.forward()), MaxSpeed);
        }
        else if (z > VolumeBounds.z)
        {
            steer = Limit(math.reflect(velocity, math.back()), MaxSpeed);
        }

        // steer behaviour (otherwise this will bounce instantaneously)
        return steer;
    }

    // Wraparound
    float3 Wraparound(float3 pos)
    {
        var x = pos.x;
        var y = pos.y;
        var z = pos.z;

        if (x < -VolumeBounds.x)
        {
            pos.x = 2 * VolumeBounds.x - x;
        }
        else if (x > VolumeBounds.x)
        {
            pos.x = -2 * VolumeBounds.x + x;
        }

        if (y < -VolumeBounds.y)
        {
            pos.y = 2 * VolumeBounds.y - y;
        }
        else if (y > VolumeBounds.y)
        {
            pos.y = -2 * VolumeBounds.y + y;
        }

        if (z < -VolumeBounds.z)
        {
            pos.z = 2 * VolumeBounds.z - z;
        }
        else if (z > VolumeBounds.z)
        {
            pos.z = -2 * VolumeBounds.z + z;
        }

        return pos;
    }

    // Separation + Align + Cohesion combined
    // separation - checks for nearby boids and steers away
    // align - checks for nearby boids and matches their weighted velocity
    // cohesion - steers to the center mass of nearby boids
    float3 SeparateAlignCohesion(int index)
    {
        float3 separateSteer = float3.zero;
        float3 alignSum = float3.zero;
        float3 cohesionSum = float3.zero;

        int separateCount = 0;
        int otherCount = 0;

        var b = boids[index];

        // float3 pos;

        // For every boid in the system, check if it's too close
        for (int i = 0; i < boids.Length; i++)
        {
            // TODO this is the biggest issue right now
            float d = math.lengthsq(boids[i].LocalPosition - b.LocalPosition);

            // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
            if (d > 0)
            {
                if (d < DesiredSeparation * DesiredSeparation)
                {
                    // Calculate vector pointing away from neighbor
                    float3 diff = b.LocalPosition - boids[i].LocalPosition;
                    diff = math.normalizesafe(diff);
                    diff /= d;        // Weight by distance
                    separateSteer += diff;
                    separateCount++;            // Keep track of how many
                }
                else if (d < NeighborDist * NeighborDist)
                {
                    alignSum += boids[i].Velocity;
                    cohesionSum += boids[i].LocalPosition;
                    otherCount++;
                }
            }

        }

        // Average -- divide by how many
        if (separateCount > 0)
        {
            separateSteer /= separateCount;
        }

        // As long as the vector is greater than 0
        if (math.lengthsq(separateSteer) > 0)
        {
            // First two lines of code below could be condensed with new PVector setMag() method
            // Not using this method until Processing.js catches up
            // steer.setMag(maxspeed);

            // Implement Reynolds: Steering = Desired - Velocity
            separateSteer = math.normalizesafe(separateSteer);
            separateSteer *= MaxSpeed;
            separateSteer -= b.Velocity;
            separateSteer = Limit(separateSteer, MaxForce);
        }

        float3 alignSteer = float3.zero;
        float3 cohesionSteer = float3.zero;

        if (otherCount > 0)
        {
            alignSum /= otherCount;
            alignSum = math.normalizesafe(alignSum);
            alignSum *= MaxSpeed;

            alignSteer = alignSum - b.Velocity;
            alignSteer = Limit(alignSteer, MaxForce);

            cohesionSum /= otherCount;
            cohesionSteer = Seek(cohesionSum, b.LocalPosition, b.Velocity);
        }

        // Arbitrarily weight these forces
        return 1.5f * separateSteer + 1f * alignSteer + 1f * cohesionSteer;
    }

    float3 Limit(float3 v, float max)
    {
        if (math.lengthsq(v) > max * max)
        {
            return math.normalizesafe(v) * max;
        }

        return v;
    }
}
