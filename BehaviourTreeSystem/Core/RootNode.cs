namespace Shard.BehaviourTreeSystem.Core
{
    public class RootNode : Node
    {

        public RootNode(Node child) : base(child)
        { }

        protected override NodeState ExecuteTask()
        {
            return children[0].Execute();
        }
    }
}
