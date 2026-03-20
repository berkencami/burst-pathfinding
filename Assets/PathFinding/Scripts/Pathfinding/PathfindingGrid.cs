using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding
{
    public enum GridType { Square, Hexagon }

    /// <summary>
    /// Manages the NativeArray-backed grid of PathNodes. Owns the persistent
    /// native memory — call Dispose() (or let OnDestroy handle it) before
    /// destroying this component.
    /// </summary>
    public class PathfindingGrid : MonoBehaviour
    {
        [Header("Grid Type")]
        [SerializeField] private GridType gridType = GridType.Square;

        [Header("Grid Settings")]
        [SerializeField] private int width = 50;
        [SerializeField] private int height = 50;
        [SerializeField] private float nodeSize = 1f;
        [SerializeField] private Vector3 originOffset = Vector3.zero;

        [Header("Gizmos")]
        [SerializeField] private bool drawGizmos = true;
        [SerializeField] private bool drawAllNodes = false;
        [SerializeField] private Color walkableColor = new Color(0f, 1f, 0f, 0.15f);
        [SerializeField] private Color obstacleColor  = new Color(1f, 0f, 0f, 0.5f);
        [SerializeField] private bool showNodeIndices = false;
        [SerializeField] private bool showCostOverlay = false;

        public int Width => width;
        public int Height => height;
        public float NodeSize => nodeSize;
        public int NodeCount => width * height;
        public GridType GridType => gridType;

        public Bounds WorldBounds =>
            new(
                originOffset + new Vector3((width - 1) * nodeSize * 0.5f, 0f, (height - 1) * nodeSize * 0.5f),
                new Vector3(width * nodeSize, 0.1f, height * nodeSize));

        // Persistent NativeArray — lives for the lifetime of this component
        private NativeArray<PathNode> _nodes;
        private bool _initialized;

        private JobHandle _lastNodesHandle;

        public NativeArray<PathNode> Nodes => _nodes;

        public JobHandle GetNodesDependency() => _lastNodesHandle;
        public void SetNodesDependency(JobHandle h) => _lastNodesHandle = h;

        private void Awake()
        {
            Initialize();
        }

        private void Initialize()
        {
            if (_initialized) return;

            _nodes = new NativeArray<PathNode>(NodeCount, Allocator.Persistent);

            for (var x = 0; x < width; x++)
            {
                for (var z = 0; z < height; z++)
                {
                    var idx = GetIndex(x, z);
                    _nodes[idx] = new PathNode
                    {
                        Position = new int2(x, z),
                        Index = idx,
                        GCost = float.MaxValue,
                        HCost = 0f,
                        IsWalkable = true,
                        ParentIndex = -1
                    };
                }
            }

            _initialized = true;
        }

        private void OnDestroy()
        {
            Dispose();
        }

        private void Dispose()
        {
            if (_initialized && _nodes.IsCreated)
            {
                _nodes.Dispose();
                _initialized = false;
            }
        }

        /// <summary>Resets all nodes to walkable with max GCost and no parent. Safe to call from editor.</summary>
        public void ClearWalkability()
        {
            if (!_initialized || !_nodes.IsCreated) return;
            _lastNodesHandle.Complete();
            for (var i = 0; i < _nodes.Length; i++)
            {
                var n = _nodes[i];
                n.IsWalkable  = true;
                n.GCost       = float.MaxValue;
                n.ParentIndex = -1;
                _nodes[i]     = n;
            }
        }

        private int GetIndex(int x, int z) => x + z * width;
        private int GetIndex(int2 pos) => pos.x + pos.y * width;

        private bool IsInBounds(int x, int z) =>
            x >= 0 && x < width && z >= 0 && z < height;

        private bool IsInBounds(int2 pos) => IsInBounds(pos.x, pos.y);

        /// <summary>Returns the nearest grid cell for a world-space position.</summary>
        public int2 WorldToGrid(Vector3 worldPos)
        {
            if (gridType == GridType.Hexagon) return WorldToHex(worldPos);

            var local = worldPos - originOffset;
            var x = Mathf.RoundToInt(local.x / nodeSize);
            var z = Mathf.RoundToInt(local.z / nodeSize);
            x = math.clamp(x, 0, width - 1);
            z = math.clamp(z, 0, height - 1);
            return new int2(x, z);
        }

        /// <summary>Returns the world-space centre of a grid cell.</summary>
        public Vector3 GridToWorld(int2 gridPos)
        {
            if (gridType == GridType.Hexagon) return HexToWorld(gridPos);
            return originOffset + new Vector3(gridPos.x * nodeSize, 0f, gridPos.y * nodeSize);
        }

        public Vector3 GridToWorld(int index)
        {
            var x = index % width;
            var z = index / width;
            return GridToWorld(new int2(x, z));
        }

        /// <summary>
        /// World → squished flat-top odd-q offset using 3-candidate nearest-hex search.
        /// row_step == col_step == nodeSize (squished hex — z axis ~13% compressed).
        /// </summary>
        private int2 WorldToHex(Vector3 worldPos)
        {
            var local = worldPos - originOffset;

            var col0 = Mathf.RoundToInt(local.x / nodeSize);
            var bestDist2 = float.MaxValue;
            var bestCol = 0;
            var bestRow = 0;

            for (var dc = -1; dc <= 1; dc++)
            {
                var c = col0 + dc;
                var r = Mathf.RoundToInt(local.z / nodeSize - 0.5f * (c & 1));
                var cx = c * nodeSize;
                var cz = nodeSize * (r + 0.5f * (c & 1));
                var dx = local.x - cx;
                var dz = local.z - cz;
                var dist2 = dx * dx + dz * dz;
                if (!(dist2 < bestDist2)) continue;
                bestDist2 = dist2; bestCol = c; bestRow = r;
            }

            bestCol = math.clamp(bestCol, 0, width - 1);
            bestRow = math.clamp(bestRow, 0, height - 1);
            return new int2(bestCol, bestRow);
        }

        /// <summary>
        /// Odd-q offset → world-space centre (squished flat-top hex).
        /// row_step == nodeSize (z axis ~13% compressed vs. true flat-top).
        /// </summary>
        private Vector3 HexToWorld(int2 hexPos)
        {
            var col = hexPos.x;
            var row = hexPos.y;
            var x = col * nodeSize;
            var z = nodeSize * (row + 0.5f * (col & 1));
            return originOffset + new Vector3(x, 0f, z);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!drawGizmos) return;

            var bounds = WorldBounds;

            if (!_initialized || !_nodes.IsCreated)
            {
                Gizmos.color = Color.white;
                Gizmos.DrawWireCube(bounds.center, bounds.size);
                return;
            }

            _lastNodesHandle.Complete();

            var cam = UnityEditor.SceneView.lastActiveSceneView?.camera;
            bool nearEnough = cam != null && Vector3.Distance(cam.transform.position, bounds.center) < 20f;

            var maxCost = 0f;
            if (showCostOverlay)
            {
                for (var i = 0; i < NodeCount; i++)
                {
                    var c = _nodes[i].GCost;
                    if (c < float.MaxValue && c > maxCost) maxCost = c;
                }
            }

            for (var i = 0; i < NodeCount; i++)
            {
                var node = _nodes[i];
                if (!drawAllNodes && node.IsWalkable) continue;

                var center = GridToWorld(node.Position);

                if (showCostOverlay && node.GCost < float.MaxValue && maxCost > 0f)
                {
                    var t = node.GCost / maxCost;
                    Gizmos.color = Color.Lerp(Color.green, Color.red, t);
                }
                else
                {
                    Gizmos.color = node.IsWalkable ? walkableColor : obstacleColor;
                }

                if (gridType == GridType.Hexagon)
                    DrawHexGizmo(center, nodeSize * 0.6f, nodeSize * 0.45f);
                else
                    Gizmos.DrawCube(center, new Vector3(nodeSize * 0.9f, 0.05f, nodeSize * 0.9f));

                if (showNodeIndices && nearEnough)
                    UnityEditor.Handles.Label(center + Vector3.up * 0.1f, node.Index.ToString());
            }

            Gizmos.color = Color.white;
            Gizmos.DrawWireCube(bounds.center, bounds.size);
        }

        private static void DrawHexGizmo(Vector3 center, float rx, float rz)
        {
            // Elliptical flat-top hex: rx (x-radius) != rz (z-radius) due to squish
            for (var i = 0; i < 6; i++)
            {
                var a0 = i * Mathf.PI / 3f;
                var a1 = (i + 1) * Mathf.PI / 3f;
                var p0 = center + new Vector3(rx * Mathf.Cos(a0), 0f, rz * Mathf.Sin(a0));
                var p1 = center + new Vector3(rx * Mathf.Cos(a1), 0f, rz * Mathf.Sin(a1));
                Gizmos.DrawLine(p0, p1);
            }
        }
#endif
    }
}
