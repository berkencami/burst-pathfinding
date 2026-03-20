using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding
{
    /// <summary>
    /// Schedules a batch of BoxcastCommands — one per grid cell — to detect
    /// obstacles on the physics thread. Results are applied back to the
    /// NativeArray&lt;PathNode&gt; by <see cref="ApplyResultsJob"/>.
    ///
    /// Usage (called from DynamicObstacleScanner):
    ///   1. Schedule()  → returns a JobHandle for the boxcast batch
    ///   2. ApplyResultsJob.Schedule(boxcastHandle) → updates PathNode.IsWalkable
    ///   3. Complete the final handle; dispose temporary arrays
    /// </summary>
    public static class GridScanJob
    {
        /// <summary>
        /// Builds BoxcastCommand array and schedules the batch.
        /// The caller owns <paramref name="commands"/> and <paramref name="results"/>
        /// and must dispose them after the returned handle is complete.
        /// </summary>
        public static JobHandle Schedule(
            NativeArray<PathNode> nodes,
            int width,
            int height,
            float nodeSize,
            Vector3 originOffset,
            LayerMask obstacleLayer,
            float castHeight,          // How high above the node centre to start the cast
            out NativeArray<BoxcastCommand> commands,
            out NativeArray<RaycastHit> results,
            JobHandle dependsOn = default,
            int minCommandsPerJob = 32,
            bool isHex = false)
        {
            var nodeCount = width * height;
            commands = new NativeArray<BoxcastCommand>(nodeCount, Allocator.TempJob);
            results  = new NativeArray<RaycastHit>(nodeCount, Allocator.TempJob);

            var buildJob = new BuildCommandsJob
            {
                Nodes        = nodes,
                Width        = width,
                NodeSize     = nodeSize,
                OriginOffset = (Unity.Mathematics.float3)originOffset,
                ObstacleMask = obstacleLayer.value,
                CastHeight   = castHeight,
                HalfExtents  = new Unity.Mathematics.float3(nodeSize * 0.45f, 0.1f, nodeSize * 0.45f),
                IsHex        = isHex,
                Commands     = commands
            };

            var buildHandle = buildJob.Schedule(nodeCount, minCommandsPerJob, dependsOn);
            return BoxcastCommand.ScheduleBatch(commands, results, minCommandsPerJob, buildHandle);
        }

        /// <summary>Fills the BoxcastCommand array in parallel — one command per node.</summary>
        private struct BuildCommandsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<PathNode> Nodes;
            public int Width;
            public float NodeSize;
            public Unity.Mathematics.float3 OriginOffset;
            public int ObstacleMask;
            public float CastHeight;
            public Unity.Mathematics.float3 HalfExtents;
            public bool IsHex;

            [WriteOnly] public NativeArray<BoxcastCommand> Commands;

            public void Execute(int i)
            {
                var node = Nodes[i];
                float3 worldPos;
                if (IsHex)
                {
                    var col = node.Position.x;
                    var row = node.Position.y;
                    worldPos = OriginOffset + new float3(
                        col * NodeSize,
                        CastHeight,
                        NodeSize * (row + 0.5f * (col & 1)));
                }
                else
                {
                    worldPos = OriginOffset + new float3(node.Position.x * NodeSize, CastHeight, node.Position.y * NodeSize);
                }

                Commands[i] = new BoxcastCommand(
                    worldPos,
                    HalfExtents,
                    Quaternion.identity,
                    Vector3.down,
                    new QueryParameters(ObstacleMask, false, QueryTriggerInteraction.Ignore, false),
                    CastHeight * 2f
                );
            }
        }

        /// <summary>
        /// Reads BoxcastCommand results and writes IsWalkable back to the grid.
        /// Schedule this after the boxcast batch handle.
        /// </summary>
        public struct ApplyResultsJob : IJobParallelFor
        {
            [ReadOnly]  public NativeArray<RaycastHit> Results;
            public NativeArray<PathNode> Nodes;

            public void Execute(int i)
            {
                var node = Nodes[i];
                // A hit means an obstacle occupies this cell → not walkable
                node.IsWalkable = Results[i].colliderInstanceID == 0;
                Nodes[i] = node;
            }
        }
    }
}
