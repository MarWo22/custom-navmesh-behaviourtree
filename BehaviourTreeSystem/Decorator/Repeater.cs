using Shard.BehaviourTreeSystem.Core;

namespace Shard.BehaviourTreeSystem.Decorator
{
    public class Repeater : Decorator
    {
        private int _repeatCount;
        private bool _quitOnFailure;
        public Repeater(Node child, int repeatCount, bool quitOnFailure) : base(child)
        {
            _repeatCount = repeatCount;
            _quitOnFailure = quitOnFailure;
        }

        protected override NodeState ExecuteTask()
        {
            for (var i = 0; i != _repeatCount; ++i)
            {
                var result = children[0].Execute();
                if (_quitOnFailure && result != NodeState.Success)
                    return result;
            }

            return NodeState.Success;
        }
    }
}