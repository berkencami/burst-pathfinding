using UnityEditor;
using UnityEngine;
using Pathfinding;

namespace PathfindingEditor
{
    [CustomEditor(typeof(PathfindingAgent))]
    public class PathfindingAgentEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            var agent = (PathfindingAgent)target;
            serializedObject.Update();

            EditorGUILayout.PropertyField(serializedObject.FindProperty("agentId"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("repathInterval"));

            if (Application.isPlaying)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Runtime Info", EditorStyles.boldLabel);

                var destInfo = agent.HasDestination
                    ? $"Destination: {agent.Destination:F1}"
                    : "No destination";
                EditorGUILayout.LabelField(destInfo);

                var mc = agent.GetComponent<AgentMovementController>();
                if (mc != null)
                {
                    var moveInfo = mc.IsMoving
                        ? $"Moving — Waypoint {mc.WaypointIndex}/{mc.WaypointCount}"
                        : "Idle";
                    EditorGUILayout.LabelField(moveInfo);
                }

                if (GUILayout.Button("Stop"))
                    agent.Stop();
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}
