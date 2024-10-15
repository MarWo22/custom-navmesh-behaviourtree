using System.Collections.Generic;

namespace Shard.BehaviourTreeSystem.Core
{
    public enum NodeState
    {
        Running,
        Success,
        Failure
    }

    public class Node
    {
        // Current state of the node
        public NodeState state { get; protected set; }
       
        // List of nodes of the children
        protected List<Node> children { get; } = new List<Node>();

        // Blackboard to share data between nodes
        public readonly BlackBoard blackboard;

        protected Node()
        {}
        
        protected Node(BlackBoard blackboard)
        {
            this.blackboard = blackboard;
        }

        protected Node(List<Node> childNodes)
        {      
            _AttachChildren(childNodes);
        }

        protected Node(Node child)
        {
            _AttachChild(child);
        }
        
        private void _AttachChildren(List<Node> childNodes)
        {
            foreach (Node child in childNodes)
                _AttachChild(child);
        }
        
        private void _AttachChild(Node node)
        {
            children.Add(node);
        }
        
        /// <summary>
        /// Executes the node
        /// </summary>
        public NodeState Execute()
        {
            state = ExecuteTask();
            return state;
        }

        /// <summary>
        /// Should be overriden to perform code
        /// </summary>
        protected virtual NodeState ExecuteTask()
        {
            return NodeState.Failure;
        }

        /// <summary>
        /// Inserts a variable into the blackboard
        /// <param name="key">The key of the variable to insert</param>
        /// <param name="value">The value of the variable to insert</param>
        /// </summary>
        protected void SetData<T>(string key, T value)
        {
            blackboard.Set(key, value);
        }

        /// <summary>
        /// Extracts a variable from the blackboard
        /// <param name="key">The key of the variable to retrieve</param>
        /// </summary>
        protected virtual T GetData<T>(string key)
        {
            return blackboard.Get<T>(key);
        }
    }
}
