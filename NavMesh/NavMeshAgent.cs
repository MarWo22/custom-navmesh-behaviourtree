using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Numerics;

namespace Shard.NavMesh;

class NavMeshAgent
{
    private readonly NavMesh _navMesh;
    private readonly GameObject _parent;
    private List<Vector2> _path;
    private Vector2 _destination;
    private NavMesh.Node _destinationNode;
    private NavMesh.Node _currentNode;
    private bool _inProgress;
    private bool _hasDestination;
    private readonly bool[] _visited;
    private readonly PriorityQueue<NavMesh.Node, float> _openNodes;
    private readonly NavMesh.Node[] _preceedingNodes;
    private readonly float[] _cheapestWeights;

    public Vector2 NextWaypoint => _path[0];
    public List<Vector2> Path => _path;
    public Vector2 Destination => _destination;
    public Vector2 CurrentPosition => new Vector2(_parent.Transform.X, _parent.Transform.Y);
    
    public bool enabled = true;
    public bool drawPath = false;
    public bool Enabled => enabled && !_parent.ToBeDestroyed;
    
    

    public NavMeshAgent(GameObject parent, NavMesh navMesh)
    {
        _parent = parent;
        _navMesh = navMesh;
        _visited = new bool[navMesh.NavMeshNodes.Count];
        _preceedingNodes = new NavMesh.Node[navMesh.NavMeshNodes.Count];
        _cheapestWeights = new float[navMesh.NavMeshNodes.Count];
        _openNodes = new PriorityQueue<NavMesh.Node, float>();
        _path = new List<Vector2>();
        NavMeshManager.Instance.AddAgent(this);
    }
    
    /// <summary>
    /// Sets a destination for the agent without directly calculating the path
    /// </summary>
    /// <param name="destination">The destination of the agent</param>
    /// <returns> True if destination is present on the NavMesh</returns>
    public bool SetDestination(Vector2 destination)
    {
            // Check if the destination already exists
        if (_destination == destination)
            return true;
        
            // Find the node the destination is on
        var node = _FindPointOnNavmesh(destination);
        if (node == null)
            return false;
        _destination = destination;
        _destinationNode = node;
        _hasDestination = true;
        return true;
    }
    
    /// <summary>
    /// Sets a destination for the agent and directly calculates the path, should be used with care for performance reasons.
    /// </summary>
    /// <param name="destination">The destination of the agent</param>
    /// <returns> True if destination is present on the NavMesh and a path exists</returns>
    public bool CalculatePath(Vector2 destination)
    {
            // Sets the destination
        var success = SetDestination(destination);
        if (!success)
            return false;
        
        return RecalculatePath();
    }

    /// <summary>
    /// Recalculates the path for the current destination.
    /// </summary>
    /// <returns> True if a path exists</returns>
    public bool RecalculatePath()
    {
            // Remove from the manager if the parent GameObject is destroyed
        if (_parent.ToBeDestroyed)
        {
            NavMeshManager.Instance.RemoveAgent(this);
            return false;
        }
            
            // Abort if there is no destination, the agent is disabled, or there are no more resources to use for this update cycle
        if (!_hasDestination || !enabled || !NavMeshManager.Instance.CanTick())
            return false;
        
            // Get the node the agent is currently on
        _currentNode = _GetCurrentNode();
        if (_currentNode == null)
        {
            Debug.Log("Agent not placed on NavMesh");
            return false;
        }
        
            // Calculate the path using the A* algorithm
        return _PerformAStar();
    }

    private bool _PerformAStar()
    {
            // Reset all data if the previous cycle was completed successfully
        if (!_inProgress)
        {
                // If the agent is already in the destination node, it's path exists of only the destination
            if (_currentNode == _destinationNode)
            {
                _path.Clear();
                _path.Add(new Vector2(_destination.X, _destination.Y));
                return true;
            }
            
                // Reset weights
            for (var i = 0; i != _navMesh.NavMeshNodes.Count; ++i)
            {
                _cheapestWeights[i] = float.PositiveInfinity;
                _visited[i] = false;
            }

                // Add neighboring nodes to the prioirity queue
            _openNodes.Clear();
            foreach (var edge in _currentNode.edges)
            {
                var distanceToNode = Vector2.Distance(new Vector2(_parent.Transform.X, _parent.Transform.Y),
                    edge.node.Center);
                
                _cheapestWeights[edge.node.index] = distanceToNode;
                _openNodes.Enqueue(edge.node, distanceToNode + Vector2.Distance(edge.node.Center, _destinationNode.Center));
                _visited[edge.node.index] = true;
                _preceedingNodes[edge.node.index] = _currentNode;
            }

            _cheapestWeights[_currentNode.index] = 0;
            _visited[_currentNode.index] = true;
        }

        _inProgress = true;

        while (_openNodes.Count > 0)
        {
                // Abort if there are no more resources
            if (!NavMeshManager.Instance.Tick())
                break;

            var current = _openNodes.Dequeue();
        
                // If it arrived at the destination node, reconstruct the path
            if (current == _destinationNode)
            {
                _inProgress = false;
                _ReconstructPath();
                return true;
            }

            foreach (var edge in current.edges)
            {
                    // The current cost to get to the node
                var tentativeCost = _cheapestWeights[current.index] + edge.weight;
                
                    // Update if it is cheaper than the previous allocated path
                if (tentativeCost < _cheapestWeights[edge.node.index])
                {
                        // Keep track of the preceding nodes to recreate the path later
                    _preceedingNodes[edge.node.index] = current;
                    _cheapestWeights[edge.node.index] = tentativeCost;
                    
                        // Add all unvisited neighbors to the priority queue, using euclidean distance to the destination as the heuristic
                    if (!_visited[edge.node.index])
                    {
                        var priority = tentativeCost + Vector2.Distance(current.Center, _destinationNode.Center);
                        _visited[edge.node.index] = true;
                        _openNodes.Enqueue(edge.node, priority);
                    }
                }
            }
                
        }
        _inProgress = false;
        return false;
    }

    private void _ReconstructPath()
    {
        _path.Clear();
            // Add the destination 
        _path.Add(_destination);
        var previousNode = _destinationNode;
            // Return if we are already at the destination node
        if (_destinationNode == _currentNode)
            return;
        var current = _preceedingNodes[_destinationNode.index];
            // Insert the waypoint that connects the two nodes
        while (current != _currentNode)
        {
            _InsertConnectingWaypoint(current, previousNode, _path[0]);
            previousNode = current;
            current = _preceedingNodes[current.index];
        }
        
        _InsertConnectingWaypoint(current, previousNode, _path[0]);
    }
    
    private static Vector2 _FindClosestPoint(Vector2 givenPoint, Vector2 linePoint1, Vector2 linePoint2)
    {
            // Vector representing the line
        Vector2 lineVector = linePoint2 - linePoint1;

            // Vector from one of the line points to the given point
        Vector2 pointToLineStart = givenPoint - linePoint1;

            // Calculate the dot product
        float dotProduct = Vector2.Dot(pointToLineStart, lineVector);

            // Calculate the length of the line
        float lineLengthSquared = lineVector.LengthSquared();

            // Parametric value of the closest point on the line
        float t = dotProduct / lineLengthSquared;

            // Clamp t to ensure the closest point is within the line segment
        t = Math.Max(0, Math.Min(1, t));

            // Calculate the closest point on the line
        Vector2 closestPoint = linePoint1 + t * lineVector;

        return closestPoint;
    }

    private void _ReduceWaypoints()
    {
        
    }

    private void _InsertConnectingWaypoint(NavMesh.Node currentNode, NavMesh.Node previousNode, Vector2 previousPoint)
    {
            // Find the point on the edge that connects the two nodes, which is closest to the previous waypoint, and add it to the list of waypoints
        var currentPosition = CurrentPosition;
        var tolerance = 0.25f * _navMesh.NodeSize;
        if (Math.Abs(currentNode.xMax - previousNode.xMin) < tolerance)
        {
            var max = Math.Min(currentNode.yMax, previousNode.yMax);
            var min = Math.Max(currentNode.yMin, previousNode.yMin);
            var point = _FindClosestPoint(previousPoint, new Vector2(currentNode.xMax, min), new Vector2(currentNode.xMax, max));
            if (Vector2.Distance(point, currentPosition) > 0.5f)
                _path.Insert(0, point);
        }
        else if (Math.Abs(currentNode.xMin - previousNode.xMax) < tolerance)
        {
            var max = Math.Min(currentNode.yMax, previousNode.yMax);
            var min = Math.Max(currentNode.yMin, previousNode.yMin);
            var point = _FindClosestPoint(previousPoint, new Vector2(currentNode.xMin, min), new Vector2(currentNode.xMin, max));
            if (Vector2.Distance(point, currentPosition) > 0.5f)    
                _path.Insert(0, point);
        }
        else if (Math.Abs(currentNode.yMax - previousNode.yMin) < tolerance)
        {
            var max = Math.Min(currentNode.xMax, previousNode.xMax);
            var min = Math.Max(currentNode.xMin, previousNode.xMin);
            var point = _FindClosestPoint(previousPoint, new Vector2(min, currentNode.yMax), new Vector2(max, currentNode.yMax));
            if (Vector2.Distance(point, currentPosition) > 0.5f)
                _path.Insert(0, point);
        }
        else if (Math.Abs(currentNode.yMin - previousNode.yMax) < tolerance)
        {
            var max = Math.Min(currentNode.xMax, previousNode.xMax);
            var min = Math.Max(currentNode.xMin, previousNode.xMin);
            var point = _FindClosestPoint(previousPoint, new Vector2(min, currentNode.yMin), new Vector2(max, currentNode.yMin));
            if (Vector2.Distance(point, currentPosition) > 0.5f)
                _path.Insert(0, point);
        }
    }
    
    private NavMesh.Node _GetCurrentNode()
    {
            // Check the cached node whether the player is still on it
        var position = new Vector2(_parent.Transform.X, _parent.Transform.Y);
        if (_currentNode != null && _IsPointOnNode(_currentNode, position))
            return _currentNode;
            
            // If not, loop over all and check each one
        return _FindPointOnNavmesh(position);
    }

    private NavMesh.Node _FindPointOnNavmesh(Vector2 point)
    {
        foreach (var node in _navMesh.NavMeshNodes)
        {
            if (!NavMeshManager.Instance.Tick())
            {
                break;
            }
            if (_IsPointOnNode(node, point))
                return node;
        }

        return null;
    }

    private bool _IsPointOnNode(NavMesh.Node node, Vector2 point)
    {
        return point.X >= node.xMin && point.X <= node.xMax && point.Y >= node.yMin && point.Y <= node.yMax;
    }
}