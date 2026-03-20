using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace Pathfinding
{
    /// <summary>
    /// Moves a set of agents along their computed paths in parallel.
    /// Uses IJobParallelForTransform so each thread writes to a unique Transform.
    ///
    /// The job reads a flat path array (same layout as BatchAStarJob output)
    /// and per-agent waypoint progress state. After Execute the caller reads
    /// back WaypointIndices to persist progress across frames.
    /// </summary>
    [BurstCompile]
    public struct AgentMoveJob : IJobParallelForTransform
    {
        [ReadOnly] public NativeArray<int2> AllPaths;
        [ReadOnly] public NativeArray<int>  PathLengths;
        [ReadOnly] public NativeArray<int>  PathStartIndices; // agentIndex * MaxPathLength
        [ReadOnly] public float             NodeSize;
        [ReadOnly] public float3            GridOrigin;
        
        /// <summary>Current waypoint index for each agent (read + write).</summary>
        public NativeArray<int> WaypointIndices;
        
        public float Speed;
        public float StoppingDistance;
        public float DeltaTime;

        public void Execute(int index, TransformAccess transform)
        {
            var pathLength = PathLengths[index];
            if (pathLength == 0) return;

            var waypointIdx = WaypointIndices[index];
            if (waypointIdx >= pathLength) return; // already at destination

            var targetGrid  = AllPaths[PathStartIndices[index] + waypointIdx];
            var targetWorld = GridOrigin + new float3(targetGrid.x * NodeSize, 0f, targetGrid.y * NodeSize);
            var currentPos  = (float3)transform.position;

            // Flatten to XZ for distance check
            var toTarget = targetWorld - currentPos;
            toTarget.y = 0f;
            var dist = math.length(toTarget);

            if (dist <= StoppingDistance)
            {
                // Advance to next waypoint
                WaypointIndices[index] = waypointIdx + 1;
                return;
            }

            var direction = math.normalize(toTarget);
            var newPos    = currentPos + direction * Speed * DeltaTime;
            newPos.y         = currentPos.y; // preserve vertical position

            transform.position = newPos;

            // Face movement direction
            if (math.lengthsq(direction) > 0.001f)
                transform.rotation = UnityEngine.Quaternion.LookRotation(direction);
        }
    }
}
