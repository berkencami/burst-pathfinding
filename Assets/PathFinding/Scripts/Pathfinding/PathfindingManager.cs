using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Pathfinding
{
    /// <summary>
    /// Central coordinator for the pathfinding system.
    ///
    /// Usage:
    ///   PathfindingManager.Instance.RequestPath(start, end, agentId, OnPathReady);
    ///   PathfindingManager.Instance.CancelPath(agentId);
    ///
    /// Architecture:
    ///   - Pending requests are queued each frame.
    ///   - LateUpdate batches all pending requests into a single BatchAStarJob.
    ///   - The job completes the following LateUpdate; callbacks fire on the main thread.
    ///   - InvalidatePaths() (called by DynamicObstacleScanner) marks all active agents
    ///     for repath at their next update interval.
    /// </summary>
    [RequireComponent(typeof(PathfindingGrid))]
    public class PathfindingManager : MonoBehaviour
    {
        public static PathfindingManager Instance { get; private set; }

        [Header("Batch Settings")]
        [SerializeField] private int maxAgents = 128;
        [SerializeField] private int maxPathLength = 512;

        private PathfindingGrid _grid;

        private readonly Queue<(PathRequest req, Action<List<Vector3>> callback)> _pending = new();
        private readonly Dictionary<int, Action<List<Vector3>>> _callbacks = new();

        private JobHandle _batchHandle;
        private bool _batchInFlight;

        private NativeArray<PathRequest> _requestBuffer;
        private NativeArray<int2>        _allPaths;
        private NativeArray<int>         _pathLengths;
        private NativeArray<int>         _pathStartIndices;
        private int                      _batchSize;

        private bool _invalidated;
        private readonly HashSet<int> _invalidatedAgents = new();

        private readonly Dictionary<int, PathfindingAgent> _agents = new();

        private void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;

            _grid = GetComponent<PathfindingGrid>();

            _requestBuffer    = new NativeArray<PathRequest>(maxAgents, Allocator.Persistent);
            _allPaths         = new NativeArray<int2>(maxAgents * maxPathLength, Allocator.Persistent);
            _pathLengths      = new NativeArray<int>(maxAgents, Allocator.Persistent);
            _pathStartIndices = new NativeArray<int>(maxAgents, Allocator.Persistent);

            for (var i = 0; i < maxAgents; i++)
                _pathStartIndices[i] = i * maxPathLength;
        }

        private void OnDestroy()
        {
            CompletePendingBatch();

            if (_requestBuffer.IsCreated)    _requestBuffer.Dispose();
            if (_allPaths.IsCreated)         _allPaths.Dispose();
            if (_pathLengths.IsCreated)      _pathLengths.Dispose();
            if (_pathStartIndices.IsCreated) _pathStartIndices.Dispose();

            if (Instance == this) Instance = null;
        }

        private void LateUpdate()
        {
            // 1. Complete previous batch and fire callbacks
            if (_batchInFlight && _batchHandle.IsCompleted)
            {
                _batchHandle.Complete();
                _batchInFlight = false;
                DeliverResults();
            }

            // 2. Schedule a new batch from the queue
            if (!_batchInFlight && _pending.Count > 0)
                ScheduleBatch();
        }

        /// <summary>
        /// Queues a pathfinding request. The callback fires on the main thread
        /// (typically 1 frame later) with the resulting world-space waypoints.
        /// An empty list indicates no path was found.
        /// </summary>
        public void RequestPath(Vector3 start, Vector3 end, int agentId, Action<List<Vector3>> callback)
        {
            var startGrid = _grid.WorldToGrid(start);
            var endGrid   = _grid.WorldToGrid(end);

            var req = new PathRequest
            {
                StartPos = startGrid,
                EndPos   = endGrid,
                AgentId  = agentId,
                IsValid  = true
            };

            _callbacks[agentId] = callback;
            _pending.Enqueue((req, callback));
        }

        /// <summary>Cancels any pending or in-flight request for this agent.</summary>
        public void CancelPath(int agentId)
        {
            _callbacks.Remove(agentId);
            _invalidatedAgents.Remove(agentId);
        }

        /// <summary>
        /// Called by DynamicObstacleScanner when the grid changes.
        /// All registered agents will repath at their next update.
        /// </summary>
        public void InvalidatePaths()
        {
            _invalidated = true;
            foreach (var kv in _agents)
                _invalidatedAgents.Add(kv.Key);
        }

        public IReadOnlyDictionary<int, PathfindingAgent> Agents => _agents;
        public bool IsBatchInFlight    => _batchInFlight;
        public int PendingRequestCount => _pending.Count;

        public void RegisterAgent(PathfindingAgent agent)   => _agents[agent.AgentId] = agent;
        public void UnregisterAgent(PathfindingAgent agent) => _agents.Remove(agent.AgentId);

        public bool IsPathInvalidated(int agentId) => _invalidatedAgents.Remove(agentId);

        private void ScheduleBatch()
        {
            _batchSize = 0;

            while (_pending.Count > 0 && _batchSize < maxAgents)
            {
                var (req, _) = _pending.Dequeue();
                _requestBuffer[_batchSize] = req;
                _batchSize++;
            }

            // Clear leftover slots
            for (var i = _batchSize; i < maxAgents; i++)
                _requestBuffer[i] = default;

            for (var i = 0; i < _batchSize; i++)
                _pathLengths[i] = 0;

            var dep = _grid.GetNodesDependency();

            if (_grid.GridType == GridType.Hexagon)
            {
                var job = new HexBatchAStarJob
                {
                    Grid          = _grid.Nodes,
                    Requests      = _requestBuffer,
                    GridSize      = new int2(_grid.Width, _grid.Height),
                    MaxPathLength = maxPathLength,
                    AllPaths      = _allPaths,
                    PathLengths   = _pathLengths
                };
                _batchHandle = job.Schedule(_batchSize, 1, dep);
            }
            else
            {
                var job = new BatchAStarJob
                {
                    Grid          = _grid.Nodes,
                    Requests      = _requestBuffer,
                    GridSize      = new int2(_grid.Width, _grid.Height),
                    MaxPathLength = maxPathLength,
                    AllPaths      = _allPaths,
                    PathLengths   = _pathLengths
                };
                _batchHandle = job.Schedule(_batchSize, 1, dep);
            }
            _grid.SetNodesDependency(_batchHandle);
            _batchInFlight = true;
        }

        private void DeliverResults()
        {
            for (var i = 0; i < _batchSize; i++)
            {
                var req = _requestBuffer[i];
                if (!req.IsValid) continue;
                if (!_callbacks.TryGetValue(req.AgentId, out var callback)) continue;

                var len = _pathLengths[i];
                var path = new List<Vector3>(len);

                for (var w = 0; w < len; w++)
                    path.Add(_grid.GridToWorld(_allPaths[_pathStartIndices[i] + w]));

                _callbacks.Remove(req.AgentId);
                callback?.Invoke(path);
            }

            if (_invalidated && _invalidatedAgents.Count == 0)
                _invalidated = false;
        }

        private void CompletePendingBatch()
        {
            if (!_batchInFlight) return;
            _batchHandle.Complete();
            _batchInFlight = false;
        }
    }
}
