using Pathfinding;
using UnityEngine;

namespace TestScene
{
    public class ClickToMove : MonoBehaviour
    {
        [SerializeField] private PathfindingAgent agent;
        [SerializeField] private Camera cam;

        private void Start()
        {
            if (cam == null)
                cam = Camera.main;
        }

        private void Update()
        {
            if (!Input.GetMouseButtonDown(0)) return;

            var ray = cam.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out var hit))
                agent.SetDestination(hit.point);
        }
    }
}
