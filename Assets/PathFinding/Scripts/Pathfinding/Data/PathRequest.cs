using Unity.Mathematics;

namespace Pathfinding
{
    /// <summary>
    /// Job-safe struct representing a single pathfinding request.
    /// Callbacks are handled on the main thread after job completion.
    /// </summary>
    public struct PathRequest
    {
        public int2 StartPos;
        public int2 EndPos;
        public int AgentId;
        public bool IsValid;
    }
}
