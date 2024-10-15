using Shard.BehaviourTreeSystem.Core;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace Shard.BehaviourTreeSystem.Composite
{
    public class Parallel : Composite
    {
        private int _requiredSuccess;
        private int _requiredFailure;
        
        public Parallel(int requiredSuccess, int requiredFailure, List<Node> children) : base(children)
        {
            _requiredSuccess = requiredSuccess;
            _requiredFailure = requiredFailure;
        }

        protected override NodeState ExecuteTask()
        {

            var successCount = 0;
            var failureCount = 0;

            var tasks = new List<Task<NodeState>>();

            foreach (var child in children)
            {
                tasks.Add(Task.Run(() => child.Execute()));
            }

            Task.WaitAll(tasks.ToArray());

            foreach (var task in tasks)
            {
                var result = task.Result;

                if (result == NodeState.Success)
                    ++successCount;
                else if (result == NodeState.Failure)
                    ++failureCount;
            }

            if (successCount >= _requiredSuccess)
                return NodeState.Success;
            if (failureCount >= _requiredFailure)
                return NodeState.Failure;

            return NodeState.Running;
        }
    }
}