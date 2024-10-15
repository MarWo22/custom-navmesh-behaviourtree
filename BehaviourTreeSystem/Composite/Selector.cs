using Shard.BehaviourTreeSystem.Core;
using System.Collections.Generic;

namespace Shard.BehaviourTreeSystem.Composite
{
    public class Selector : Composite
    {
        public Selector(List<Node> children) : base(children) 
        { }


        protected override NodeState ExecuteTask()
        {
            foreach (var child in children)
            {
                switch (child.Execute())
                {
                    case NodeState.Failure:
                        continue;
                    case NodeState.Running:
                        return NodeState.Running;
                    case NodeState.Success:
                        return NodeState.Success;
                    default:
                        continue;
                }
            }

            return NodeState.Failure;
        }
    }
}
