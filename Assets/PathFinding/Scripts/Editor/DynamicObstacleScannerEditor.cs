using UnityEditor;
using UnityEngine;
using Pathfinding;

namespace PathfindingEditor
{
    [CustomEditor(typeof(DynamicObstacleScanner))]
    public class DynamicObstacleScannerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            var scanner = (DynamicObstacleScanner)target;
            serializedObject.Update();

            EditorGUILayout.PropertyField(serializedObject.FindProperty("_scanInterval"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_obstacleLayer"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("_castHeight"));

            var layerProp = serializedObject.FindProperty("_obstacleLayer");
            if (layerProp.intValue == 0 || layerProp.intValue == -1)
                EditorGUILayout.HelpBox(
                    "Obstacle Layer is set to Everything or Nothing. Configure a specific layer for accurate obstacle detection.",
                    MessageType.Warning);

            if (Application.isPlaying)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Runtime Info", EditorStyles.boldLabel);

                var lastScan = scanner.LastScanTime < 0f
                    ? "Never"
                    : $"{scanner.LastScanTime:F2}s";
                EditorGUILayout.LabelField("Last Scan Time", lastScan);
                EditorGUILayout.LabelField("Scan In Flight", scanner.IsScanInFlight ? "Yes" : "No");

                EditorGUI.BeginDisabledGroup(scanner.IsScanInFlight);
                if (GUILayout.Button("Force Scan"))
                    scanner.ScheduleScan();
                EditorGUI.EndDisabledGroup();
            }

            serializedObject.ApplyModifiedProperties();

            if (Application.isPlaying) Repaint();
        }
    }
}
