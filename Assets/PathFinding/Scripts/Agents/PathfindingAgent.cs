using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding
{
    /// <summary>
    /// Attach to any GameObject that needs pathfinding.
    /// Requests paths from PathfindingManager, then hands them to
    /// AgentMovementController to execute.
    ///
    /// The agent automatically repaths when:
    ///   - The destination changes (call SetDestination).
    ///   - The grid is invalidated due to dynamic obstacles.
    ///   - The optional repathInterval elapses.
    /// </summary>
    [RequireComponent(typeof(AgentMovementController))]
    public class PathfindingAgent : MonoBehaviour
    {
        [Header("Identity")]
        [SerializeField] private int agentId = -1; // -1 = auto-assign from InstanceID

        [Header("Repath")]
        [SerializeField] private float repathInterval = 1f; // 0 = no automatic repath

        private AgentMovementController _movement;
        private Vector3 _destination;
        private bool _hasDestination;
        private float _repathTimer;

        public int AgentId        { get; private set; }
        public Vector3 Destination => _destination;
        public bool HasDestination => _hasDestination;

        private void Awake()
        {
            _movement = GetComponent<AgentMovementController>();
            AgentId   = agentId < 0 ? gameObject.GetInstanceID() : agentId;
        }

        private void OnEnable()
        {
            PathfindingManager.Instance?.RegisterAgent(this);
        }

        private void OnDisable()
        {
            PathfindingManager.Instance?.UnregisterAgent(this);
            PathfindingManager.Instance?.CancelPath(AgentId);
        }

        private void Update()
        {
            if (!_hasDestination) return;

            // Repath if the grid was updated due to a dynamic obstacle
            if (PathfindingManager.Instance != null &&
                PathfindingManager.Instance.IsPathInvalidated(AgentId))
            {
                RequestPath();
                return;
            }

            // Periodic repath
            if (repathInterval > 0f)
            {
                _repathTimer += Time.deltaTime;
                if (_repathTimer >= repathInterval)
                {
                    _repathTimer = 0f;
                    RequestPath();
                }
            }
        }

        /// <summary>Sets a new destination and immediately queues a path request.</summary>
        public void SetDestination(Vector3 destination)
        {
            _destination    = destination;
            _hasDestination = true;
            _repathTimer    = 0f;
            RequestPath();
        }

        public void Stop()
        {
            _hasDestination = false;
            _movement.ClearPath();
            PathfindingManager.Instance?.CancelPath(AgentId);
        }

        private void RequestPath()
        {
            if (PathfindingManager.Instance == null) return;
            PathfindingManager.Instance.RequestPath(
                transform.position,
                _destination,
                AgentId,
                OnPathFound);
        }

        private void OnPathFound(List<Vector3> path)
        {
            if (path == null || path.Count == 0) return;
            _movement.SetPath(path);
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!_hasDestination) return;
            UnityEditor.Handles.color = new Color(1f, 1f, 0f, 0.8f);
            UnityEditor.Handles.DrawWireDisc(_destination, Vector3.up, 0.4f);
            UnityEditor.Handles.DrawDottedLine(transform.position, _destination, 4f);
        }
#endif
    }
}
