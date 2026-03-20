using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding
{
    /// <summary>
    /// Executes movement along a waypoint path computed by PathfindingAgent.
    /// Runs on the main thread (no job overhead for single-agent control);
    /// for large crowds use AgentMoveJob instead.
    ///
    /// Keeps movement logic decoupled from path-request logic.
    /// </summary>
    public class AgentMovementController : MonoBehaviour
    {
        [Header("Movement")]
        [SerializeField] private float speed = 5f;
        [SerializeField] private float stoppingDistance = 0.15f;
        [SerializeField] private float rotationSpeed = 720f; // degrees per second

        [Header("Debug")]
        [SerializeField] private bool useCustomPathColor = false;
        [SerializeField] private Color customPathColor = Color.yellow;

        private List<Vector3> _path;
        private int _waypointIndex;

        public bool IsMoving      => _path != null && _waypointIndex < _path.Count;
        public int WaypointIndex  => _waypointIndex;
        public int WaypointCount  => _path?.Count ?? 0;
        public IReadOnlyList<Vector3> Path => _path;

        private void Update()
        {
            if (!IsMoving) return;

            var target    = _path[_waypointIndex];
            var direction = target - transform.position;
            direction.y = 0f;

            var dist = direction.magnitude;

            if (dist <= stoppingDistance)
            {
                _waypointIndex++;
                return;
            }

            // Translate
            transform.position += direction.normalized * (speed * Time.deltaTime);

            // Rotate toward movement direction
            if (!(direction.sqrMagnitude > 0.001f)) return;
            var targetRot = Quaternion.LookRotation(direction.normalized);
            transform.rotation  = Quaternion.RotateTowards(
                transform.rotation, targetRot, rotationSpeed * Time.deltaTime);
        }
        
        public void SetPath(List<Vector3> path)
        {
            _path          = path;
            _waypointIndex = FindClosestWaypointIndex();

            // Skip first waypoint if it's the agent's current cell
            if (_waypointIndex != 0 || _path.Count <= 1) return;
            var first = _path[0];
            first.y = transform.position.y;
            if (Vector3.Distance(transform.position, first) <= stoppingDistance)
                _waypointIndex = 1;
        }

        private int FindClosestWaypointIndex()
        {
            if (_path == null || _path.Count == 0) return 0;

            var minDist = float.MaxValue;
            var closestIndex = 0;
            var agentPos = transform.position;

            for (var i = 0; i < _path.Count; i++)
            {
                var wp = _path[i];
                wp.y = agentPos.y;
                var dist = Vector3.Distance(agentPos, wp);
                if (!(dist < minDist)) continue;
                minDist = dist;
                closestIndex = i;
            }

            // If the closest waypoint is behind the agent, advance one step
            if (closestIndex >= _path.Count - 1) return closestIndex;
            var toWp = _path[closestIndex] - agentPos;
            toWp.y = 0f;
            if (Vector3.Dot(toWp, transform.forward) < 0f)
                closestIndex++;

            return closestIndex;
        }

        public void ClearPath()
        {
            _path          = null;
            _waypointIndex = 0;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (_path == null || _path.Count == 0) return;

            var agent = GetComponent<PathfindingAgent>();
            var agentColor = useCustomPathColor
                ? customPathColor
                : agent != null
                    ? Color.HSVToRGB((agent.AgentId * 0.61803f) % 1f, 0.8f, 0.9f)
                    : Color.cyan;

            UnityEditor.Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
            UnityEditor.Handles.color = agentColor;

            for (var i = _waypointIndex; i < _path.Count - 1; i++)
            {
                var from = _path[i];
                var to   = _path[i + 1];
                UnityEditor.Handles.DrawLine(from, to);

                // Arrow head
                var dir = (to - from).normalized;
                var mid = Vector3.Lerp(from, to, 0.7f);
                var right = Vector3.Cross(Vector3.up, dir) * 0.4f;
                UnityEditor.Handles.DrawLine(mid, mid - dir * 0.6f + right);
                UnityEditor.Handles.DrawLine(mid, mid - dir * 0.6f - right);
            }

            var remaining = _path.Count - _waypointIndex;
            if (remaining > 0)
            {
                var labelPos = _path[_waypointIndex] + Vector3.up * 0.5f;
                UnityEditor.Handles.Label(labelPos, $"{remaining} wp");
            }

            UnityEditor.Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;

            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(_path[^1], 0.25f);
        }
#endif
    }
}
