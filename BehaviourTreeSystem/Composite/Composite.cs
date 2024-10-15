using System.Collections.Generic;
using Shard.BehaviourTreeSystem.Core;

namespace Shard.BehaviourTreeSystem.Composite;

public class Composite : Node
{
    protected Composite(List<Node> children) : base(children)
    {}
    
}