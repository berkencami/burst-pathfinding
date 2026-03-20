using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace Pathfinding
{
    /// <summary>
    /// Periodically issues a BoxcastCommand batch via <see cref="GridScanJob"/>
    /// to detect obstacle changes in the scene. When a change is found, notifies
    /// <see cref="PathfindingManager"/> so affected agents can repath.
    ///
    /// Attach alongside <see cref="PathfindingGrid"/> and <see cref="PathfindingManager"/>.
    /// </summary>
    [RequireComponent(typeof(PathfindingGrid))]
    public class DynamicObstacleScanner : MonoBehaviour
    {
        [Header("Scan Settings")]
        [SerializeField] private float _scanInterval = 0.25f;
        [SerializeField] private LayerMask _obstacleLayer = ~0;
        [SerializeField] private float _castHeight = 2f;

        private PathfindingGrid _grid;
        private PathfindingManager _manager;

        private float _timer;
        private JobHandle _pendingHandle;
        private bool _scanInFlight;
        private float _lastScanTime = -1f;

        public float LastScanTime  => _lastScanTime;
        public bool IsScanInFlight => _scanInFlight;
        public float ScanInterval  => _scanInterval;

        // Temp arrays kept between Schedule and Complete to avoid re-allocation
        private NativeArray<BoxcastCommand> _commands;
        private NativeArray<RaycastHit>     _results;

        private void Awake()
        {
            _grid    = GetComponent<PathfindingGrid>();
            _manager = GetComponent<PathfindingManager>();
        }

        private void Update()
        {
            if (_scanInFlight && _pendingHandle.IsCompleted)
            {
                _pendingHandle.Complete();
                ApplyAndNotify();
                _scanInFlight = false;
            }

            _timer += Time.deltaTime;
            if (!(_timer >= _scanInterval) || _scanInFlight) return;
            _timer = 0f;
            ScheduleScan();
        }

        private void OnDestroy()
        {
            // Ensure jobs complete before the NativeArrays are freed
            if (_scanInFlight)
            {
                _pendingHandle.Complete();
                _scanInFlight = false;
            }

            if (_commands.IsCreated) _commands.Dispose();
            if (_results.IsCreated)  _results.Dispose();
        }

        public void ScheduleScan()
        {
            var dep = _grid.GetNodesDependency();

            var batchHandle = GridScanJob.Schedule(
                _grid.Nodes,
                _grid.Width,
                _grid.Height,
                _grid.NodeSize,
                transform.position,
                _obstacleLayer,
                _castHeight,
                out _commands,
                out _results,
                dependsOn: dep,
                isHex: _grid.GridType == GridType.Hexagon);

            var applyJob = new GridScanJob.ApplyResultsJob
            {
                Results = _results,
                Nodes   = _grid.Nodes
            };

            _pendingHandle = applyJob.Schedule(_grid.NodeCount, 64, batchHandle);
            _grid.SetNodesDependency(_pendingHandle);
            _scanInFlight  = true;
        }

        private void ApplyAndNotify()
        {
            if (_commands.IsCreated) { _commands.Dispose(); }
            if (_results.IsCreated)  { _results.Dispose(); }

            _lastScanTime = Time.time;
            _manager?.InvalidatePaths();
        }
    }
}
