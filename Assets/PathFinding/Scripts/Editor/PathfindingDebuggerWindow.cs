using UnityEditor;
using UnityEngine;
using Pathfinding;

namespace PathfindingEditor
{
    public class PathfindingDebuggerWindow : EditorWindow
    {
        [MenuItem("Pathfinding/Debugger")]
        public static void Open() => GetWindow<PathfindingDebuggerWindow>("Pathfinding Debugger");

        private Vector2 _agentScroll;
        private int     _cachedObstacleCount;
        private float   _lastCountTime = -1f;

        private void OnEnable()  => EditorApplication.update += Repaint;
        private void OnDisable() => EditorApplication.update -= Repaint;

        private void OnGUI()
        {
            var manager = FindFirstObjectByType<PathfindingManager>();
            var grid    = FindFirstObjectByType<PathfindingGrid>();
            var scanner = FindFirstObjectByType<DynamicObstacleScanner>();
            
            EditorGUILayout.LabelField("Grid Info", EditorStyles.boldLabel);

            if (grid == null)
            {
                EditorGUILayout.HelpBox("No PathfindingGrid found in scene.", MessageType.Warning);
            }
            else
            {
                EditorGUILayout.LabelField("Node Count", grid.NodeCount.ToString());
                EditorGUILayout.LabelField("Bounds", grid.WorldBounds.size.ToString("F0"));

                if (Application.isPlaying && grid.Nodes.IsCreated)
                {
                    if (Time.time > _lastCountTime + 0.5f)
                    {
                        grid.GetNodesDependency().Complete();
                        var obs = 0;
                        for (var i = 0; i < grid.NodeCount; i++)
                            if (!grid.Nodes[i].IsWalkable) obs++;
                        _cachedObstacleCount = obs;
                        _lastCountTime = Time.time;
                    }
                    EditorGUILayout.LabelField("Obstacle Nodes", _cachedObstacleCount.ToString());
                }
            }

            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Scan", EditorStyles.boldLabel);

            if (scanner == null)
            {
                EditorGUILayout.HelpBox("No DynamicObstacleScanner in scene.", MessageType.Warning);
            }
            else
            {
                var lastScan = scanner.LastScanTime < 0f ? "Never" : $"{scanner.LastScanTime:F2}s";
                EditorGUILayout.LabelField("Last Scan", lastScan);
                EditorGUILayout.LabelField("Scan Interval", $"{scanner.ScanInterval:F2}s");

                EditorGUI.BeginDisabledGroup(!Application.isPlaying || scanner.IsScanInFlight);
                if (GUILayout.Button("Force Scan"))
                    scanner.ScheduleScan();
                EditorGUI.EndDisabledGroup();
            }

            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Pathfinding", EditorStyles.boldLabel);

            if (manager == null)
            {
                EditorGUILayout.HelpBox("No PathfindingManager in scene.", MessageType.Warning);
            }
            else
            {
                EditorGUILayout.LabelField("Batch In Flight",    manager.IsBatchInFlight.ToString());
                EditorGUILayout.LabelField("Pending Requests",   manager.PendingRequestCount.ToString());
            }

            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Agents", EditorStyles.boldLabel);

            if (manager != null && manager.Agents != null)
            {
                _agentScroll = EditorGUILayout.BeginScrollView(_agentScroll, GUILayout.Height(160));

                EditorGUILayout.BeginHorizontal(EditorStyles.toolbar);
                EditorGUILayout.LabelField("ID",          GUILayout.Width(60));
                EditorGUILayout.LabelField("Destination", GUILayout.Width(120));
                EditorGUILayout.LabelField("Waypoint",    GUILayout.Width(80));
                EditorGUILayout.LabelField("Moving",      GUILayout.Width(50));
                EditorGUILayout.EndHorizontal();

                foreach (var (key, a) in manager.Agents)
                {
                    var mc = a != null ? a.GetComponent<AgentMovementController>() : null;

                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField(key.ToString(),                  GUILayout.Width(60));
                    EditorGUILayout.LabelField(a != null && a.HasDestination
                        ? a.Destination.ToString("F1") : "—",                     GUILayout.Width(120));
                    EditorGUILayout.LabelField(mc != null
                        ? $"{mc.WaypointIndex}/{mc.WaypointCount}" : "—",         GUILayout.Width(80));
                    EditorGUILayout.LabelField(mc != null && mc.IsMoving ? "Yes" : "No", GUILayout.Width(50));
                    EditorGUILayout.EndHorizontal();
                }

                EditorGUILayout.EndScrollView();
            }

            EditorGUILayout.Space();

            if (grid == null) return;
            EditorGUILayout.LabelField("Gizmo Toggles", EditorStyles.boldLabel);
            var so = new SerializedObject(grid);
            so.Update();

            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(so.FindProperty("drawGizmos"));
            EditorGUILayout.PropertyField(so.FindProperty("drawAllNodes"));
            if (!EditorGUI.EndChangeCheck()) return;
            so.ApplyModifiedProperties();
            EditorUtility.SetDirty(grid);
        }
    }
}
