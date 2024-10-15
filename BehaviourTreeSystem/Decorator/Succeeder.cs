using Shard.BehaviourTreeSystem.Core;

namespace Shard.BehaviourTreeSystem.Decorator
{
    public class Succeeder : Decorator
    {
        public Succeeder(Node child) : base(child)
        { }

        protected override NodeState ExecuteTask()
        {
            children[0].Execute();
            return NodeState.Success;
        }
    }
}