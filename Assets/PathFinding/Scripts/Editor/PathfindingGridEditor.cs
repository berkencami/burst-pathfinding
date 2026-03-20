using UnityEditor;
using UnityEngine;
using Pathfinding;

namespace PathfindingEditor
{
    [CustomEditor(typeof(PathfindingGrid))]
    public class PathfindingGridEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            var grid    = (PathfindingGrid)target;
            var scanner = grid.GetComponent<DynamicObstacleScanner>();

            serializedObject.Update();
            
            EditorGUILayout.LabelField("Grid Settings", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("gridType"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("width"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("height"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("nodeSize"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("originOffset"));

            var bounds = grid.WorldBounds;
            EditorGUILayout.HelpBox(
                $"Nodes: {grid.NodeCount}  Bounds: {bounds.size.x:F0} × {bounds.size.z:F0}",
                MessageType.None);

            if (scanner == null)
                EditorGUILayout.HelpBox(
                    "No DynamicObstacleScanner found on this GameObject. Dynamic obstacles will not be detected.",
                    MessageType.Warning);

            EditorGUILayout.Space();
            
            EditorGUILayout.LabelField("Gizmos", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("drawGizmos"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("drawAllNodes"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("walkableColor"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("obstacleColor"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("showNodeIndices"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("showCostOverlay"));

            EditorGUILayout.Space();

          
            EditorGUI.BeginDisabledGroup(!Application.isPlaying);

            var scanDisabled = scanner != null && scanner.IsScanInFlight;
            EditorGUI.BeginDisabledGroup(scanDisabled);
            if (GUILayout.Button("Scan Now"))
                scanner?.ScheduleScan();
            EditorGUI.EndDisabledGroup();

            if (GUILayout.Button("Clear Walkability"))
            {
                Undo.RecordObject(grid, "Clear Walkability");
                grid.ClearWalkability();
            }

            EditorGUI.EndDisabledGroup();

            serializedObject.ApplyModifiedProperties();
        }
    }
}
