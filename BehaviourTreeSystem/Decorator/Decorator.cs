using Shard.BehaviourTreeSystem.Core;

namespace Shard.BehaviourTreeSystem.Decorator;

public class Decorator : Node
{
    protected Decorator(Node child) : base(child)
    {}
}