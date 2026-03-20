using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Pathfinding
{
    /// <summary>
    /// Processes multiple pathfinding requests in parallel — one thread per agent.
    ///
    /// Output layout:
    ///   AllPaths is a flat NativeArray. Each agent's path starts at
    ///   PathStartIndices[agentIndex] and has length PathLengths[agentIndex].
    ///   PathStartIndices is pre-computed with MaxPathLength * agentIndex so
    ///   each slot is independent and there are no write conflicts.
    ///
    /// The caller allocates AllPaths, PathStartIndices, and PathLengths and
    /// must dispose them after reading results on the main thread.
    /// </summary>
    [BurstCompile]
    public struct BatchAStarJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PathNode>    Grid;
        [ReadOnly] public NativeArray<PathRequest> Requests;
        public int2 GridSize;
        public int MaxPathLength; // pre-allocated slots per agent

        [NativeDisableParallelForRestriction]
        public NativeArray<int2> AllPaths;        // flat: MaxPathLength * agentCount

        [NativeDisableParallelForRestriction]
        public NativeArray<int>  PathLengths;     // actual path length per agent

        private const float StraightCost = 1.0f;
        private const float DiagonalCost = 1.41421356f;

        public void Execute(int agentIndex)
        {
            var req       = Requests[agentIndex];
            var slotStart = agentIndex * MaxPathLength;
            PathLengths[agentIndex] = 0;

            if (!req.IsValid) return;

            var nodeCount  = GridSize.x * GridSize.y;
            var startIndex = GetIndex(req.StartPos);
            var endIndex   = GetIndex(req.EndPos);

            if (!IsInBounds(req.StartPos) || !IsInBounds(req.EndPos)) return;
            if (!Grid[startIndex].IsWalkable || !Grid[endIndex].IsWalkable) return;

            var gCosts        = new NativeArray<float>(nodeCount, Allocator.Temp);
            var parentIndices = new NativeArray<int>(nodeCount, Allocator.Temp);
            var openHeap      = new NativeList<int>(64, Allocator.Temp);
            var closedSet     = new NativeHashSet<int>(64, Allocator.Temp);

            for (var i = 0; i < nodeCount; i++)
            {
                gCosts[i]        = float.MaxValue;
                parentIndices[i] = -1;
            }

            gCosts[startIndex] = 0f;
            HeapPush(ref openHeap, startIndex, gCosts);

            var found = false;

            while (openHeap.Length > 0)
            {
                var current = HeapPop(ref openHeap, gCosts);

                if (current == endIndex) { found = true; break; }
                if (closedSet.Contains(current)) continue;
                closedSet.Add(current);

                var currentPos = IndexToPos(current);

                for (var dx = -1; dx <= 1; dx++)
                {
                    for (var dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dz == 0) continue;

                        var nPos = currentPos + new int2(dx, dz);
                        if (!IsInBounds(nPos)) continue;

                        var nIdx = GetIndex(nPos);
                        if (!Grid[nIdx].IsWalkable) continue;
                        if (closedSet.Contains(nIdx)) continue;

                        var isDiagonal = dx != 0 && dz != 0;
                        if (isDiagonal)
                        {
                            if (!Grid[GetIndex(currentPos + new int2(dx, 0))].IsWalkable) continue;
                            if (!Grid[GetIndex(currentPos + new int2(0, dz))].IsWalkable) continue;
                        }

                        var tentativeG = gCosts[current] + (isDiagonal ? DiagonalCost : StraightCost);
                        if (!(tentativeG < gCosts[nIdx])) continue;
                        gCosts[nIdx]        = tentativeG;
                        parentIndices[nIdx] = current;
                        HeapPush(ref openHeap, nIdx, gCosts);
                    }
                }
            }

            if (found)
            {
                // Reconstruct path directly into AllPaths slot
                var temp = new NativeList<int2>(32, Allocator.Temp);
                var curr = endIndex;
                while (curr != -1)
                {
                    temp.Add(IndexToPos(curr));
                    curr = parentIndices[curr];
                }

                var len = math.min(temp.Length, MaxPathLength);
                PathLengths[agentIndex] = len;

                for (var i = 0; i < len; i++)
                    AllPaths[slotStart + i] = temp[temp.Length - 1 - i]; // reverse

                temp.Dispose();
            }

            gCosts.Dispose();
            parentIndices.Dispose();
            openHeap.Dispose();
            closedSet.Dispose();
        }

        private static void HeapPush(ref NativeList<int> heap, int idx, NativeArray<float> g)
        {
            heap.Add(idx);
            var i = heap.Length - 1;
            while (i > 0)
            {
                var p = (i - 1) / 2;
                if (g[heap[i]] < g[heap[p]]) { Swap(ref heap, i, p); i = p; }
                else break;
            }
        }

        private static int HeapPop(ref NativeList<int> heap, NativeArray<float> g)
        {
            var top  = heap[0];
            var last = heap.Length - 1;
            heap[0] = heap[last];
            heap.RemoveAt(last);
            if (heap.Length > 0) SiftDown(ref heap, 0, g);
            return top;
        }

        private static void SiftDown(ref NativeList<int> heap, int i, NativeArray<float> g)
        {
            var n = heap.Length;
            while (true)
            {
                var l = 2 * i + 1;
                var r = 2 * i + 2;
                var s = i;
                if (l < n && g[heap[l]] < g[heap[s]]) s = l;
                if (r < n && g[heap[r]] < g[heap[s]]) s = r;
                if (s == i) break;
                Swap(ref heap, i, s);
                i = s;
            }
        }

        private static void Swap(ref NativeList<int> h, int a, int b)
        {
            (h[a], h[b]) = (h[b], h[a]);
        }

        private int GetIndex(int2 pos) => pos.x + pos.y * GridSize.x;

        private int2 IndexToPos(int i) => new int2(i % GridSize.x, i / GridSize.x);

        private bool IsInBounds(int2 pos) =>
            pos.x >= 0 && pos.x < GridSize.x && pos.y >= 0 && pos.y < GridSize.y;
    }
}
