using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;

namespace Shard.NavMesh;

class NavMeshManager
{
    private static NavMeshManager _instance;
    private List<NavMesh> _navMeshes;
    private Queue<NavMeshAgent> _agentUpdateQueue;
    private Queue<NavMeshAgent> _doubleBuffer;

    private int _currentIteration;
    
    public int maxIterations = 5000;
    public long updateIntervalMilliseconds = 20;
    private long _lastUpdate = 0;
    private bool _isActive = false;
    
    public static NavMeshManager Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = new NavMeshManager();
            }

            return _instance;
        }
    }

    private NavMeshManager()
    {
        _navMeshes = new List<NavMesh>();
        _agentUpdateQueue = new Queue<NavMeshAgent>();
        _doubleBuffer = new Queue<NavMeshAgent>();
    }

    /// <summary>
    /// Add a NavMesh agent to the manager
    /// </summary>
    /// <param name="agent">The agent to add</param>
    public void AddAgent(NavMeshAgent agent)
    {
        _agentUpdateQueue.Enqueue(agent);
    }

    /// <summary>
    /// Remove a NavMesh agent from the manager
    /// </summary>
    /// <param name="agent">The agent to remove</param>
    public void RemoveAgent(NavMeshAgent agent)
    {
        var newQueue = new Queue<NavMeshAgent>();
        foreach(var currAgent in _agentUpdateQueue)
            if (agent != currAgent)
                newQueue.Enqueue(currAgent);
        
        _agentUpdateQueue = newQueue;
    }

    /// <summary>
    /// Add a NavMesh to the manager
    /// </summary>
    /// <param name="navMesh">The NavMesh to add</param>
    public void AddNavMesh(NavMesh navMesh)
    {
        _navMeshes.Add(navMesh);
    }

    /// <summary>
    /// Remove a NavMesh from the manager
    /// </summary>
    /// <param name="navMesh">The NavMesh to remove</param>
    public void RemoveNavMesh(NavMesh navMesh)
    {
        _navMeshes.Remove(navMesh);
    }

    /// <summary>
    /// Draws all paths and NavMeshes which have the setting enabled, and calculate all paths up to the specified maximum iterations.
    /// </summary>
    public void Update()
    {
        _isActive = true;
        _currentIteration = 0;
            // Draw the navmesh overlays
        _DrawNavMeshes();
            // Draw the agent paths
        _DrawAgentPaths();
        
            // Runs at 50 updates per second standard
        if (Bootstrap.getCurrentMillis() - _lastUpdate > updateIntervalMilliseconds)
        {
            _lastUpdate = Bootstrap.getCurrentMillis();
                // Update agent paths
            _UpdateAgents();
        }

        _isActive = false;
    }

    private void _DrawAgentPaths()
    {
        while (_agentUpdateQueue.TryPeek(out var agent))
        {
            _DrawPath(agent);
            _doubleBuffer.Enqueue(_agentUpdateQueue.Dequeue());
        }
        
        (_agentUpdateQueue, _doubleBuffer) = (_doubleBuffer, _agentUpdateQueue);
    }

    public bool Tick()
    {
            // Check if the there are still resources available, or the manager is not running
        if (!CanTick())
            return false;

            // Update the iteration
        ++_currentIteration;
        return true;
    }

    public bool CanTick()
    {
        if (!_isActive)
            return true;
        return _currentIteration < maxIterations;
    }

    private void _UpdateAgents()
    {
        while (_agentUpdateQueue.TryPeek(out var agent))
        {
                // Calculate the path
            agent.RecalculatePath();

            if (_agentUpdateQueue.TryPeek(out var tempAgent))
                _doubleBuffer.Enqueue(_agentUpdateQueue.Dequeue());
        }
        
        (_agentUpdateQueue, _doubleBuffer) = (_doubleBuffer, _agentUpdateQueue);
    }

    private void _DrawPath(NavMeshAgent agent)
    {
        if (!agent.drawPath || agent.Path.Count == 0 || !agent.Enabled)
            return;
        
        var display = Bootstrap.getDisplay();

        foreach (var waypoint in agent.Path)
            display.drawFilledCircle((int)waypoint.X, (int)waypoint.Y, 4, Color.Red);
        
        var currentPos = agent.CurrentPosition;
        
        display.drawLine((int)currentPos.X, (int)currentPos.Y, (int)agent.Path[0].X, (int)agent.Path[0].Y, Color.Red);
        
        for (var i = 1; i != agent.Path.Count; ++i)
            display.drawLine((int)agent.Path[i-1].X, (int)agent.Path[i-1].Y, (int)agent.Path[i].X, (int)agent.Path[i].Y, Color.Red);

    }

    private void _DrawNavMeshes()
    {
        foreach(var navMesh in _navMeshes)
            if (navMesh.drawNavMesh)
                _DrawNavMesh(navMesh);
    }
    
    private void _DrawNavMesh(NavMesh navMesh)
    {
        var display = Bootstrap.getDisplay();

        foreach (var node in navMesh.NavMeshNodes)
        {
            display.drawLine((int)node.xMin, (int)node.yMax, (int)node.xMax, (int)node.yMax, 255, 255, 255, 40);
            display.drawLine((int)node.xMin, (int)node.yMin, (int)node.xMax, (int)node.yMin, 255, 255, 255, 40);
            display.drawLine((int)node.xMin, (int)node.yMax, (int)node.xMin, (int)node.yMin, 255, 255, 255, 40);
            display.drawLine((int)node.xMax, (int)node.yMax, (int)node.xMax, (int)node.yMin, 255, 255, 255, 40);
            
            display.drawFilledCircle((int)node.CenterX, (int)node.CenterY, 2, 0, 255, 0, 128);
        }
        
        display.drawLine((int)navMesh.Xmin, (int)navMesh.Ymax, (int)navMesh.Xmax, (int)navMesh.Ymax, 0, 0, 0, 255);
        display.drawLine((int)navMesh.Xmin, (int)navMesh.Ymin, (int)navMesh.Xmax, (int)navMesh.Ymin, 0, 0, 0, 255);
        display.drawLine((int)navMesh.Xmin, (int)navMesh.Ymax, (int)navMesh.Xmin, (int)navMesh.Ymin, 0, 0, 0, 255);
        display.drawLine((int)navMesh.Xmax, (int)navMesh.Ymax, (int)navMesh.Xmax, (int)navMesh.Ymin, 0, 0, 0, 255);
    }
}