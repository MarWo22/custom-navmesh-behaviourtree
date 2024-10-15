using Shard.BehaviourTreeSystem.Core;

namespace Shard.BehaviourTreeSystem.Decorator
{
    public class Inverter : Decorator
    {
        private bool _interpretRunningAsFailure;

        public Inverter(Node child, bool interpretRunningAsFailure = false) : base(child)
        {
            _interpretRunningAsFailure = interpretRunningAsFailure;
        }

        protected override NodeState ExecuteTask()
        {
            var result = children[0].Execute();

            return result switch
            {
                NodeState.Failure => NodeState.Success,
                NodeState.Success => NodeState.Failure,
                NodeState.Running => _interpretRunningAsFailure ? NodeState.Failure : NodeState.Running,
                _ => NodeState.Failure
            };
        }
    }
}