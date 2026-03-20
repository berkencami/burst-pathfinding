using Unity.Mathematics;

namespace Pathfinding
{
    /// <summary>
    /// Blittable struct representing a single cell in the pathfinding grid.
    /// Must remain blittable (no managed types) for Burst and Job compatibility.
    /// </summary>
    public struct PathNode
    {
        public int2 Position;
        public int Index;

        public float GCost;
        public float HCost;
        public float FCost => GCost + HCost;

        public bool IsWalkable;
        public int ParentIndex; // -1 means no parent

        public void ResetPathData()
        {
            GCost = float.MaxValue;
            HCost = 0f;
            ParentIndex = -1;
        }
    }
}
