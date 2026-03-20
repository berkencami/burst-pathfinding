using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Pathfinding
{
    /// <summary>
    /// Batch A* for flat-top odd-q hexagon grids.
    /// Uses cube-distance heuristic and uniform step cost (no diagonals).
    /// Parallel structure mirrors <see cref="BatchAStarJob"/> — one thread per agent.
    /// </summary>
    [BurstCompile]
    public struct HexBatchAStarJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<PathNode>    Grid;
        [ReadOnly] public NativeArray<PathRequest> Requests;
        public int2 GridSize;
        public int MaxPathLength;

        [NativeDisableParallelForRestriction]
        public NativeArray<int2> AllPaths;

        [NativeDisableParallelForRestriction]
        public NativeArray<int>  PathLengths;

        private const float StraightCost = 1.0f;

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
            var fCosts        = new NativeArray<float>(nodeCount, Allocator.Temp);
            var parentIndices = new NativeArray<int>(nodeCount, Allocator.Temp);
            var openHeap      = new NativeList<int>(64, Allocator.Temp);
            var closedSet     = new NativeHashSet<int>(64, Allocator.Temp);

            for (var i = 0; i < nodeCount; i++)
            {
                gCosts[i]        = float.MaxValue;
                fCosts[i]        = float.MaxValue;
                parentIndices[i] = -1;
            }

            gCosts[startIndex] = 0f;
            fCosts[startIndex] = HexHeuristic(req.StartPos, req.EndPos);
            HeapPush(ref openHeap, startIndex, fCosts);

            var found = false;

            while (openHeap.Length > 0)
            {
                var current = HeapPop(ref openHeap, fCosts);

                if (current == endIndex) { found = true; break; }
                if (closedSet.Contains(current)) continue;
                closedSet.Add(current);

                var currentPos = IndexToPos(current);
                var col = currentPos.x;
                var isOdd = (col & 1) == 1;

                // Flat-top odd-q: 6 neighbours
                // dQ:      {+1, +1,  0, -1, -1,  0}
                // dR_even: { 0, -1, -1, -1,  0, +1}
                // dR_odd:  {+1,  0, -1,  0, +1, +1}
                for (var d = 0; d < 6; d++)
                {
                    var dq = d is 0 or 1 ? 1 : d is 3 or 4 ? -1 : 0;
                    int dr;
                    if (isOdd)
                        dr = (d == 2) ? -1 : d is 0 or 4 or 5 ? +1 : 0;
                    else
                        dr = d is 1 or 2 or 3 ? -1 : (d == 5) ? +1 : 0;

                    var nPos = new int2(col + dq, currentPos.y + dr);
                    if (!IsInBounds(nPos)) continue;

                    var nIdx = GetIndex(nPos);
                    if (!Grid[nIdx].IsWalkable) continue;
                    if (closedSet.Contains(nIdx)) continue;

                    var tentativeG = gCosts[current] + StraightCost;
                    if (!(tentativeG < gCosts[nIdx])) continue;
                    gCosts[nIdx]        = tentativeG;
                    fCosts[nIdx]        = tentativeG + HexHeuristic(nPos, req.EndPos);
                    parentIndices[nIdx] = current;
                    HeapPush(ref openHeap, nIdx, fCosts);
                }
            }

            if (found)
            {
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
                    AllPaths[slotStart + i] = temp[temp.Length - 1 - i];

                temp.Dispose();
            }

            gCosts.Dispose();
            fCosts.Dispose();
            parentIndices.Dispose();
            openHeap.Dispose();
            closedSet.Dispose();
        }

        /// <summary>Cube distance between two odd-q offset coordinates.</summary>
        private static int HexHeuristic(int2 a, int2 b)
        {
            // Offset → axial
            var aq = a.x; var ar = a.y - (a.x - (a.x & 1)) / 2;
            var bq = b.x; var br = b.y - (b.x - (b.x & 1)) / 2;
            // Axial → cube distance
            var dx = math.abs(aq - bq);
            var dy = math.abs((-aq - ar) - (-bq - br));
            var dz = math.abs(ar - br);
            return math.max(math.max(dx, dy), dz);
        }

        private static void HeapPush(ref NativeList<int> heap, int idx, NativeArray<float> f)
        {
            heap.Add(idx);
            var i = heap.Length - 1;
            while (i > 0)
            {
                var p = (i - 1) / 2;
                if (f[heap[i]] < f[heap[p]]) { Swap(ref heap, i, p); i = p; }
                else break;
            }
        }

        private static int HeapPop(ref NativeList<int> heap, NativeArray<float> f)
        {
            var top  = heap[0];
            var last = heap.Length - 1;
            heap[0] = heap[last];
            heap.RemoveAt(last);
            if (heap.Length > 0) SiftDown(ref heap, 0, f);
            return top;
        }

        private static void SiftDown(ref NativeList<int> heap, int i, NativeArray<float> f)
        {
            var n = heap.Length;
            while (true)
            {
                var l = 2 * i + 1;
                var r = 2 * i + 2;
                var s = i;
                if (l < n && f[heap[l]] < f[heap[s]]) s = l;
                if (r < n && f[heap[r]] < f[heap[s]]) s = r;
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
