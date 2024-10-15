using Shard.BehaviourTreeSystem.Core;
using System.Collections.Generic;

namespace Shard.BehaviourTreeSystem.Composite
{
    public class Sequence : Composite
    {
        public Sequence(List<Node> children) : base(children)
        { }

        protected override NodeState ExecuteTask()
        {
            foreach (var child in children)
            {
                switch (child.Execute())
                {
                    case NodeState.Failure:
                        return NodeState.Failure;
                    case NodeState.Success:
                        continue;
                    case NodeState.Running:
                        return NodeState.Running;
                    default:
                        continue;
                }
            }

            return NodeState.Success;
        }
    }
}
