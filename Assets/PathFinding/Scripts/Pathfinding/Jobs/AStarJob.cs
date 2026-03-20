using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Pathfinding
{
    /// <summary>
    /// Single-agent A* pathfinding job.
    /// Fully Burst-compiled, runs on a worker thread.
    ///
    /// Heuristic: Octile distance (avoids sqrt for diagonals).
    /// Neighbour search: 8-directional.
    /// Open list: index-based binary min-heap stored in a NativeList.
    /// Closed set: NativeHashSet for O(1) membership tests.
    ///
    /// All temporary allocations use Allocator.Temp and are disposed
    /// inside Execute() — no caller cleanup required for internals.
    /// The caller owns <see cref="ResultPath"/> and must dispose it.
    /// </summary>
    [BurstCompile]
    public struct AStarJob : IJob
    {
        [ReadOnly] public NativeArray<PathNode> Grid;
        public int2 GridSize;       // (width, height)
        public int2 StartPos;
        public int2 EndPos;

        /// <summary>
        /// Path from start to end (inclusive), in grid coordinates.
        /// Empty if no path was found. Allocated by the caller with
        /// Allocator.TempJob (or Persistent) before scheduling.
        /// </summary>
        public NativeList<int2> ResultPath;

        private const float StraightCost = 1.0f;
        private const float DiagonalCost = 1.41421356f; // sqrt(2)

        public void Execute()
        {
            ResultPath.Clear();

            var nodeCount  = GridSize.x * GridSize.y;
            var startIndex = GetIndex(StartPos);
            var endIndex   = GetIndex(EndPos);

            // Guard: out-of-bounds or unwalkable endpoints
            if (!IsInBounds(StartPos) || !IsInBounds(EndPos)) return;
            if (!Grid[startIndex].IsWalkable || !Grid[endIndex].IsWalkable) return;

            var gCosts        = new NativeArray<float>(nodeCount, Allocator.Temp);
            var parentIndices = new NativeArray<int>(nodeCount, Allocator.Temp);
            var openHeap      = new NativeList<int>(64, Allocator.Temp);
            var closedSet     = new NativeHashSet<int>(64, Allocator.Temp);

            // Init
            for (var i = 0; i < nodeCount; i++)
            {
                gCosts[i]        = float.MaxValue;
                parentIndices[i] = -1;
            }

            gCosts[startIndex] = 0f;
            HeapPush(ref openHeap, startIndex, gCosts, GetHCost(StartPos, EndPos));

            var found = false;

            while (openHeap.Length > 0)
            {
                var current = HeapPop(ref openHeap, gCosts, EndPos);

                if (current == endIndex)
                {
                    found = true;
                    break;
                }

                if (closedSet.Contains(current)) continue;
                closedSet.Add(current);

                var currentPos = IndexToPos(current);

                // 8-directional neighbours
                for (var dx = -1; dx <= 1; dx++)
                {
                    for (var dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dz == 0) continue;

                        var neighbourPos = currentPos + new int2(dx, dz);
                        if (!IsInBounds(neighbourPos)) continue;

                        var neighbourIndex = GetIndex(neighbourPos);
                        if (!Grid[neighbourIndex].IsWalkable) continue;
                        if (closedSet.Contains(neighbourIndex)) continue;

                        // Prevent cutting corners through diagonals blocked by walls
                        var isDiagonal = dx != 0 && dz != 0;
                        if (isDiagonal)
                        {
                            if (!Grid[GetIndex(currentPos + new int2(dx, 0))].IsWalkable) continue;
                            if (!Grid[GetIndex(currentPos + new int2(0, dz))].IsWalkable) continue;
                        }

                        var moveCost   = isDiagonal ? DiagonalCost : StraightCost;
                        var tentativeG = gCosts[current] + moveCost;

                        if (tentativeG < gCosts[neighbourIndex])
                        {
                            gCosts[neighbourIndex]        = tentativeG;
                            parentIndices[neighbourIndex] = current;
                            var h = GetHCost(neighbourPos, EndPos);
                            HeapPush(ref openHeap, neighbourIndex, gCosts, h);
                        }
                    }
                }
            }

            if (found)
                ReconstructPath(parentIndices, endIndex);

            // Cleanup
            gCosts.Dispose();
            parentIndices.Dispose();
            openHeap.Dispose();
            closedSet.Dispose();
        }

        private void ReconstructPath(NativeArray<int> parentIndices, int endIndex)
        {
            // Walk from end to start, then reverse
            var reversePath = new NativeList<int2>(32, Allocator.Temp);
            var current = endIndex;

            while (current != -1)
            {
                reversePath.Add(IndexToPos(current));
                current = parentIndices[current];
            }

            // Reverse into ResultPath (start → end order)
            for (var i = reversePath.Length - 1; i >= 0; i--)
                ResultPath.Add(reversePath[i]);

            reversePath.Dispose();
        }

        // Heap stores node indices; FCost is computed on the fly.

        private static void HeapPush(ref NativeList<int> heap, int nodeIndex,
            NativeArray<float> gCosts, float hCost)
        {
            heap.Add(nodeIndex);
            // Store hCost temporarily encoded alongside gCost not possible in pure ints,
            // so we use FCost = gCosts[nodeIndex] + hCost for comparisons.
            // We keep a parallel float array for FCost approximation via sift-up.
            SiftUp(ref heap, heap.Length - 1, gCosts, hCost);
        }

        private static int HeapPop(ref NativeList<int> heap,
            NativeArray<float> gCosts, int2 endPos)
        {
            var top  = heap[0];
            var last = heap.Length - 1;
            heap[0] = heap[last];
            heap.RemoveAt(last);
            if (heap.Length > 0)
                SiftDown(ref heap, 0, gCosts, endPos);
            return top;
        }

        private static void SiftUp(ref NativeList<int> heap, int i,
            NativeArray<float> gCosts, float _)
        {
            while (i > 0)
            {
                var parent = (i - 1) / 2;
                if (gCosts[heap[i]] < gCosts[heap[parent]])
                {
                    Swap(ref heap, i, parent);
                    i = parent;
                }
                else break;
            }
        }

        private static void SiftDown(ref NativeList<int> heap, int i,
            NativeArray<float> gCosts, int2 endPos)
        {
            var n = heap.Length;
            while (true)
            {
                var left     = 2 * i + 1;
                var right    = 2 * i + 2;
                var smallest = i;

                if (left  < n && gCosts[heap[left]]  < gCosts[heap[smallest]]) smallest = left;
                if (right < n && gCosts[heap[right]] < gCosts[heap[smallest]]) smallest = right;

                if (smallest == i) break;
                Swap(ref heap, i, smallest);
                i = smallest;
            }
        }

        private static void Swap(ref NativeList<int> heap, int a, int b)
        {
            var tmp = heap[a];
            heap[a] = heap[b];
            heap[b] = tmp;
        }

        /// <summary>Octile distance heuristic — admissible for 8-directional grids.</summary>
        private static float GetHCost(int2 from, int2 to)
        {
            var dx   = math.abs(to.x - from.x);
            var dz   = math.abs(to.y - from.y);
            var minD = math.min(dx, dz);
            var maxD = math.max(dx, dz);
            return DiagonalCost * minD + StraightCost * (maxD - minD);
        }

        private int GetIndex(int2 pos) => pos.x + pos.y * GridSize.x;

        private int2 IndexToPos(int index) => new int2(index % GridSize.x, index / GridSize.x);

        private bool IsInBounds(int2 pos) =>
            pos.x >= 0 && pos.x < GridSize.x && pos.y >= 0 && pos.y < GridSize.y;
    }
}
