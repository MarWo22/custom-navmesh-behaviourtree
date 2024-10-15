using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Shard.NavMesh;

class NavMesh
{
        // Representation of a node within the navmesh
    public class Node
    {
        public float xMin;
        public float xMax;
        public float yMax;
        public float yMin;

        public int index;
        
        
        public Vector2 Center => new(CenterX, CenterY);
        public float CenterX => xMin + (xMax - xMin) / 2;
        public float CenterY => yMin + (yMax - yMin) / 2;
        
        public List<Edge> edges;
    }

        // Representation of an edge within the navmesh
    public struct Edge
    {
        public Node node;
        public float weight;
    }
    
        // Grid used during the baking process
    private struct BakingGrid
    {
        public float nodeSize;
        public Vector2 startIndices;

        public int xSize;
        public int ySize;
        public List<List<bool>> grid;
        public List<List<bool>> visitedNodes;
    }

        // Base class to represent agent types used during the baking to assure their collider sizes are taken into account
    public abstract class AgentType
    {
        protected Transform transform;
        protected Collider collider;
        
        protected Transform _InitTransform(float width, float height, float nodeSize)
        {
            return new Transform(null)
            {
                X = 0,
                Y = 0,
                Wid = 1,
                Ht = 1,
                Scalex = width + nodeSize,
                Scaley = height + nodeSize
            };
        }

        public abstract Collider GetColliderAtPos(float x, float y);
        public abstract void InitCollider(float nodeSize);
    }
        
        // For agents with rectangle colliders
    public class RectAgent : AgentType
    {
        private float _width;
        private float _height;

        public RectAgent(float width, float height)
        {
            _width = width;
            _height = height;
        }

        public float Width => _width;
        public float Height => _height;

        public override Collider GetColliderAtPos(float x, float y)
        {
            transform.X = x;
            transform.Y = y;
            collider.recalculate();
            return collider;
        }
        
        public override void InitCollider(float nodeSize)
        {
            transform = _InitTransform(nodeSize, _width, _height);
            collider = new ColliderRect(null, transform);
        }
    }

        // For agents with circular colliders
    public class CircleAgent : AgentType
    {
        private float _radius;
        public CircleAgent(float radius)
        {
            _radius = radius;
        }
        
        public float Radius => _radius;
        
        public override void InitCollider(float nodeSize)
        {
            transform = _InitTransform(nodeSize, _radius*2, _radius*2);
            collider = new ColliderCircle(null, transform);
        }
        
        public override Collider GetColliderAtPos(float x, float y)
        {
            transform.X = x;
            transform.Y = y;
            collider.recalculate();
            return collider;
        }
    }

    public bool drawNavMesh;
    
    private float _xMin;
    private float _xMax;
    private float _yMin;
    private float _yMax;
    private float _nodeSize;

    public float NodeSize => _nodeSize;
    public float Xmin => _xMin;
    public float Xmax => _xMax;
    public float Ymin => _yMin;
    public float Ymax => _yMax;

    private List<Node> _navMeshNodes;

    public List<Node> NavMeshNodes => _navMeshNodes;
    
    public NavMesh(float xMin, float xMax, float yMax, float yMin)
    {
        _xMin = xMin;
        _xMax = xMax;
        _yMin = yMin;
        _yMax = yMax;
        NavMeshManager.Instance.AddNavMesh(this);
    }
    
    /// <summary>
    /// Bake the environment to a usable Navigation Mesh for agents to traverse.
    /// </summary>
    /// <param name="nodeSize">The size of each grid node in world space. A lower value results in a more accurate grid, but higher computational cost during baking</param>
    /// <param name="agentType">The agent the NavMesh should be baked and adjusted for.</param>
    /// <param name="center">The starting position of the baking algorithm.</param>
    public void Bake(float nodeSize, AgentType agentType, Vector2 center)
    {
            // Initialize the grid used during baking
        var bakingGrid = _InitBakingGrid(nodeSize, center);
        
            // Create a list of all colliders and groundObjects used during baking
        _GetGameObjects(out var colliders, out var groundObjects);
        
        _Bake(bakingGrid, colliders, groundObjects, agentType);
    }
    
    /// <summary>
    /// Bake the environment to a usable Navigation Mesh for agents to traverse.
    /// </summary>
    /// <param name="nodeSize">The size of each grid node in world space. A lower value results in a more accurate grid, but higher computational cost during baking</param>
    /// <param name="agentType">The agent the NavMesh should be baked and adjusted for.</param>
    public void Bake(float nodeSize, AgentType agentType)
    {
            // Initialize the grid used during baking
        var bakingGrid = _InitBakingGrid(nodeSize, new Vector2((int)(_xMin + (_xMax - _xMin) / 2), (int)(_yMin + (_yMax - _yMin) / 2)));
        
        _GetGameObjects(out var colliders, out var groundObjects);
            
            // Locate the center to start baking from
        var center = _GetCenter((int)bakingGrid.startIndices.X, (int)bakingGrid.startIndices.Y, groundObjects, colliders, bakingGrid );
        if (center == null)
        {
            Debug.Log("Error baking NavMesh, no ground objects found");
            return;
        }
            // Reset grid
        for (int i = 0; i != bakingGrid.xSize; ++i)
            for (int j = 0; j != bakingGrid.ySize; ++j)
                bakingGrid.visitedNodes[i][j] = false;

        bakingGrid.startIndices = center.Value;
        _Bake(bakingGrid, colliders, groundObjects, agentType);
    }

    private void _Bake(BakingGrid bakingGrid, List<Collider> colliders, List<GameObject> groundObjects, AgentType agentype)
    {
        _nodeSize = bakingGrid.nodeSize;
        
            // Create the collider for the specified agent type
        agentype.InitCollider(_nodeSize);
        
            // Iteratively build the environment into a grid
        _VisitNodesIteratively(groundObjects, colliders, bakingGrid, agentype);
            
            // Convert the grid to a usable graph structure of nodes
        _CreateNodes(bakingGrid);
    }

    private void _GetGameObjects(out List<Collider> colliders, out List<GameObject> groundObjects)
    {
        groundObjects = new();
        colliders = new();

        var gameObjects = GameObjectManager.getInstance().GetGameObjectsReadonly();

            // Split into GameObjects with and without colliders
        foreach (var gameObject in gameObjects)
            if (gameObject.hasCollider())
                foreach (var collider in gameObject.MyBody.getColliders())
                {
                    collider.recalculate();
                    colliders.Add(collider);
                }
            else
                groundObjects.Add(gameObject);
    }

    private BakingGrid _InitBakingGrid(float nodeSize, Vector2 center)
    {
        var bakingGrid = new BakingGrid
        {
            nodeSize = nodeSize,
            xSize = (int)MathF.Ceiling((_xMax - _xMin) / nodeSize),
            ySize = (int)MathF.Ceiling((_yMax - _yMin) / nodeSize),
            grid = new List<List<bool>>(),
            visitedNodes = new List<List<bool>>()
        };

        bakingGrid.startIndices = _WorldPosToIndices(center.X, center.Y, bakingGrid);

        for (var i = 0; i != bakingGrid.xSize; ++i)
        {
            bakingGrid.visitedNodes.Add(new List<bool>());
            bakingGrid.grid.Add(new List<bool>());
            for (var j = 0; j != bakingGrid.ySize; ++j)
            {
                bakingGrid.visitedNodes[i].Add(false);
                bakingGrid.grid[i].Add(false);
            }
        }

        return bakingGrid;
    }

    private Vector2? _GetCenter(int startX, int startY, List<GameObject> groundObjects, List<Collider> colliders, BakingGrid bakingGrid)
    {
        var nodesToVisit = new Stack<(int, int)>();
        nodesToVisit.Push((startX, startY));

        while (nodesToVisit.Count > 0)
        {
            var (x, y) = nodesToVisit.Pop();

            if (bakingGrid.visitedNodes[x][y])
                continue;
            
            bakingGrid.visitedNodes[x][y] = true;

            GameObject cachedGroundObject = null;
            cachedGroundObject = _CheckGroundObjectOverlap(x, y, cachedGroundObject, groundObjects, bakingGrid);

            if (cachedGroundObject != null)
                return new Vector2(x, y);
        }
        
        return null;
    }
    
    private Vector2 _IndicesToWorldPos(int x, int y, BakingGrid bakingGrid)
    {
        return new Vector2(_xMin + x * bakingGrid.nodeSize, _yMin + y * bakingGrid.nodeSize);
    }

    private Vector2 _WorldPosToIndices(float x, float y, BakingGrid bakingGrid)
    {
        return new Vector2((int)((x - _xMin) / bakingGrid.nodeSize), (int)((y - _yMin) / bakingGrid.nodeSize));
    }
    
    private void _VisitNodesIteratively(List<GameObject> groundObjects, List<Collider> colliders, BakingGrid bakingGrid, AgentType agentType)
    {
            // Add the center to the stack
        var nodesToVisit = new Stack<(int, int)>();
        nodesToVisit.Push(((int)bakingGrid.startIndices.X, (int)bakingGrid.startIndices.Y));
        
        Collider cachedCollider = null;
        GameObject cachedGroundObject = null;
        
        while (nodesToVisit.Count > 0)
        {
                // Get a grid space that is not visited yet
            var (x, y) = nodesToVisit.Pop();

            if (bakingGrid.visitedNodes[x][y])
                continue;
            
            bakingGrid.visitedNodes[x][y] = true;
                
                // Check if the grid space is not on the void, and cache the object if there is overlap
            cachedGroundObject = _CheckGroundObjectOverlap(x, y, cachedGroundObject, groundObjects, bakingGrid);
            if (cachedGroundObject == null)
            {
                continue;
            }
            
                // Place the agent collider at the current grid space
            var pos = _IndicesToWorldPos(x, y, bakingGrid);
            var gridCollider = agentType.GetColliderAtPos(pos.X, pos.Y);
                
                // Check if it collides with any objects using caching, and mark it as walkable if there is no collision
            cachedCollider = _CheckCollisions(gridCollider, cachedCollider, colliders);
            if (cachedCollider == null)
            {
                bakingGrid.grid[x][y] = true;
            }
            
                // Push all neighbors to the stack
            if (x > 0 && !bakingGrid.visitedNodes[x - 1][y])
                nodesToVisit.Push((x - 1, y));

            if (x < bakingGrid.grid.Count - 1 && !bakingGrid.visitedNodes[x + 1][y])
                nodesToVisit.Push((x + 1, y));

            if (y > 0 && !bakingGrid.visitedNodes[x][y - 1])
                nodesToVisit.Push((x, y - 1));

            if (y < bakingGrid.grid[x].Count - 1 && !bakingGrid.visitedNodes[x][y + 1])
                nodesToVisit.Push((x, y + 1));

        }
    }

    private Collider _CheckCollisions(Collider gridCollider, Collider cachedCollider, List<Collider> colliders)
    {       
            // Check the cached collider
        if (cachedCollider != null && gridCollider.checkCollision(cachedCollider) != null)
        {
            return cachedCollider;
        }
            // Check against all colliders. For baking, a naive method like this is sufficient enough
        foreach (var collider in colliders)
        {
            if (gridCollider.checkCollision(collider) != null)
            {
                return collider;
            }
        }

        return null;
    }

    private GameObject _CheckGroundObjectOverlap(int x, int y, GameObject cachedGroundObject, List<GameObject> groundObjects, BakingGrid bakingGrid)
    {
            // Check if the grid is entirely overlapped by the cached Ground Object 
        if (cachedGroundObject != null && _HasGroundFullOverlap(x, y, cachedGroundObject, bakingGrid))
            return cachedGroundObject;
        
            // Check against all
        foreach (var groundObject in groundObjects)
        {
            if (_HasGroundFullOverlap(x, y, groundObject, bakingGrid))
                return groundObject;
        }

        return null;
    }

    private bool _HasGroundFullOverlap(int x, int y, GameObject groundObject, BakingGrid bakingGrid)
    {
        var pos = _IndicesToWorldPos(x, y, bakingGrid);
        // Calculate the coordinates of the corners of the first rectangle
        var x1Min = pos.X;
        var x1Max = pos.X + bakingGrid.nodeSize;
        var y1Min = pos.Y;
        var y1Max = pos.Y + bakingGrid.nodeSize;

        // Calculate the coordinates of the corners of the second rectangle
        var x2Min = groundObject.Transform.X;
        var x2Max = groundObject.Transform.X + groundObject.Transform.AbsWidth;
        var y2Min = groundObject.Transform.Y;
        var y2Max = groundObject.Transform.Y + groundObject.Transform.AbsHeight;
        
        // Check if all corners of the first rectangle are inside the second rectangle
        return x1Min >= x2Min && x1Max <= x2Max &&
               y1Min >= y2Min && y1Max <= y2Max;
    }
    
    private void _CreateNodes(BakingGrid bakingGrid)
    {
        // Create a node for each walkable grid cell
        _navMeshNodes = new List<Node>();
        for (var i = 0; i != bakingGrid.xSize; ++i)
        {
            for (var j = 0; j != bakingGrid.ySize; ++j)
            {
                if (bakingGrid.grid[i][j])
                {
                    _CreateNode(i, j, bakingGrid);
                }
            }
        }

        for (var i = 0; i != _navMeshNodes.Count; ++i)
            _navMeshNodes[i].index = i;
    }

    private void _CreateNode(int x, int y, BakingGrid bakingGrid)
    {
            // Get the size of the node 
        var size = _GetSquareSize(x, y, bakingGrid);
            
            // Disable the grid for the new node
        for (int i = x; i != x + size; ++i)
        {
            for (int j = y; j != y + size; ++j)
                bakingGrid.grid[i][j] = false;
        }
    
            // Create the node and set dimensions
        var node = new Node();
        node.index = _navMeshNodes.Count;
        var pos1 = _IndicesToWorldPos(x, y, bakingGrid);
        var pos2 = _IndicesToWorldPos(x + size, y + size, bakingGrid);
        node.xMin = pos1.X;
        node.yMin = pos1.Y;
        node.xMax = pos2.X;
        node.yMax = pos2.Y;
        
            // Initialize edge connections
        _InitNodeNeighbors(node, bakingGrid);
        _navMeshNodes.Add(node);
        
            // Try and merge with neighbors to reduce the node quantity
        _AttemptMergeNeighbors(node, bakingGrid);
    }

    private void _InitNodeNeighbors(Node node, BakingGrid bakingGrid)
    {
            // For each node in the navmesh, check if it is to the left or top of the current node
        node.edges = new List<Edge>();
        var tolerance = bakingGrid.nodeSize * 0.25f;
        foreach (var currNode in _navMeshNodes)
        {
            var isLeftNeighbor = ((currNode.yMax <= node.yMin && currNode.yMin >= node.yMax) ||
                                 (currNode.yMin <= node.yMax && currNode.yMax >= node.yMin)) &&
                                 Math.Abs(currNode.xMax - node.xMin) < tolerance;
            
            var isTopNeighbor = ((currNode.xMax <= node.xMin && currNode.xMin >= node.xMax) ||
                                 (currNode.xMin <= node.xMax && currNode.xMax >= node.xMin)) &&
                                Math.Abs(currNode.yMax - node.yMin) < tolerance;
            
            if (isLeftNeighbor || isTopNeighbor)
            {
                var distance = Vector2.Distance(currNode.Center, node.Center);
                node.edges.Add(new Edge{node = currNode, weight = distance});
                currNode.edges.Add(new Edge{node = node, weight = distance});
            }
        }
    }

    private void _AttemptMergeNeighbors(Node node, BakingGrid bakingGrid)
    {
            // If two equally sizes nodes are neighboring, merge them, and recursively call merge on the new node
        var tolerance = bakingGrid.nodeSize * 0.25f;
        foreach (var edge in node.edges)
        {
            if (Math.Abs(edge.node.xMax - node.xMin) < tolerance &&
                Math.Abs(edge.node.yMax - node.yMax) < tolerance &&
                Math.Abs(edge.node.yMin - node.yMin) < tolerance)
            {
                edge.node.xMax = node.xMax;
                _MergeEdges(edge.node, node);
                _navMeshNodes.Remove(node);
                _AttemptMergeNeighbors(edge.node, bakingGrid);
                break;
            }
            
            if (Math.Abs(edge.node.yMax - node.yMin) < tolerance &&
                Math.Abs(edge.node.xMax - node.xMax) < tolerance &&
                Math.Abs(edge.node.xMin - node.xMin) < tolerance)
            {
                edge.node.yMax = node.yMax;
                _MergeEdges(edge.node, node);
                _navMeshNodes.Remove(node);
                _AttemptMergeNeighbors(edge.node, bakingGrid);
                break;
            }
        }
    }

    private void _MergeEdges(Node updatedNode, Node connectedNode)
    {
        var newEdges = new List<Edge>();
        
            // Add all edges from the new node
        foreach (var edge in updatedNode.edges)
        {
            if (edge.node == connectedNode)
                continue;
            
            if (newEdges.All(e => e.node != edge.node))
                newEdges.Add(new Edge{node = edge.node, weight = Vector2.Distance(updatedNode.Center, edge.node.Center)});
        }
        
            // Add all unique edges from the old node
        foreach (var edge in connectedNode.edges)
        {
            if (edge.node == updatedNode)
                continue;
            
            if (newEdges.All(e => e.node != edge.node))
                newEdges.Add(new Edge{node = edge.node, weight = Vector2.Distance(updatedNode.Center, edge.node.Center)});
        }

        updatedNode.edges = newEdges;
            
            // Update the edges within the neighbors
        foreach (var edge in updatedNode.edges)
        {
            edge.node.edges.RemoveAll(e => e.node == connectedNode || e.node == updatedNode);
            edge.node.edges.Add(new Edge{node = updatedNode, weight = Vector2.Distance(updatedNode.Center, edge.node.Center)});
        }
    }
    
    private int _GetSquareSize(int x, int y, BakingGrid bakingGrid)
    {
            // Grow a square to the maximum size possible within the grid, to reduce the quantity of nodes
        var size = 1;
        var isValid = true;
        while (true)
        {
            if (x + size >= bakingGrid.xSize || y + size >= bakingGrid.ySize)
            {
                break;
            }

            for (int i = x; i != x + size; ++i)
            {
                if (!bakingGrid.grid[i][y + size])
                {
                    isValid = false;
                    break;
                }
            }

            if (!isValid)
                break;

            for (int i = y; i != y + size; ++i)
            {
                if (!bakingGrid.grid[x + size][i])
                {
                    isValid = false;
                    break;
                }
            }

            if (!isValid)
                break;

            if (!bakingGrid.grid[x + size][y + size])
                break;

            ++size;
        }

        return size;
    }
}