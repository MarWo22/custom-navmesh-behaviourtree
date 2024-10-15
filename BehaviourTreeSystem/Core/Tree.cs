using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using System.Xml;
using Shard.BehaviourTreeSystem.Decorator;
using Shard.BehaviourTreeSystem.Leaves;

namespace Shard.BehaviourTreeSystem.Core
{
    abstract class Tree 
    {
        private RootNode _root;

        // Blackboard to share data between nodes
        private BlackBoard _blackboard = new BlackBoard();

        protected List<string> nodeNamespaces = new();
        
        public BlackBoard blackboard => _blackboard;

        private bool _blackboardInitialized;

        protected Tree()
        {
                
            nodeNamespaces.Add("Shard.BehaviourTreeSystem.Composite");
            nodeNamespaces.Add("Shard.BehaviourTreeSystem.Decorator");
            nodeNamespaces.Add("Shard.BehaviourTreeSystem.Leaves");
            nodeNamespaces.Add("Shard.BehaviourTreeSystem.Core");

            _root = SetupTree();
        }

        /// <summary>
        /// Executes the behaviour tree
        /// </summary>
        public void Update()
        {
            if (!_blackboardInitialized)
            {
                _blackboardInitialized = true;
                SetupBlackBoard();
            }

            _root?.Execute();
        }

        /// <summary>
        /// Should contain all code to setup the tree
        /// </summary>
        protected abstract RootNode SetupTree();
        
        /// <summary>
        /// Should contain all code to setup the blackboard
        /// </summary>
        protected abstract void SetupBlackBoard();


        private struct TreeStructure
        {
            public string className;
            public string[] parameters;
            public List<string> edges;
        }
        
        // Reads in an XML file in the format exported from the online flowchart editor draw.io: https://app.diagrams.net/
        // The user should create the tree vertically, and use nodes to represent nodes in the tree, with the value being the class name,
        // and parameter list followed by comma, if needed. i.e. 'Repeater, 4, true'. Class names are case sensitive.
        // The XML file is converted to a tree.
        // Relatively expensive function call due to reflection, but only needs to be ran once per intantation.
        // It is suggested to save the instance in any derived tree that is created multiple times as a static variable, and
        // load it from there after initial creation.
        // For future implementations, serialization can be used to save the tree to disk in a c# friendly manner, reducing load times
        protected RootNode CreateFromXML(string fileName)
        {
                // Load xml file
            var doc = new XmlDocument();
            doc.Load(Bootstrap.getAssetManager().getAssetPath(fileName));
                
                // Extract nodes and edges
            var nodes = doc.SelectNodes("//mxCell[@value]");
            var edges = doc.SelectNodes("//mxCell[@source]");
            
                // Use a dictionary to hold edges
            string rootId = null;
            var tree = new Dictionary<string, TreeStructure>();
            
            foreach (XmlNode node in nodes)
            {
                var id = node.Attributes["id"].Value;
                var classData = node.Attributes["value"].Value;
                    // Extract parameters
                var parameters = classData.Replace(" ", "").Split(",");
                    // Class name is the first parameter
                var className = parameters[0];
                string[] structParameters = null;
                    // Extract constructor parameters if they are present
                if (parameters.Length > 1)
                {
                    structParameters = parameters.Skip(1).ToArray();
                }
                    // Save the root node
                if (className == "RootNode")
                    rootId = id;
                    
                    // Add the node to the dictionary
                tree.Add(id, new TreeStructure { className = className, edges = new List<string>(), parameters = structParameters});
            }

                // Add edges to the dictionary
            foreach (XmlNode edge in edges)
            {
                var source = edge.Attributes["source"].Value;
                var target = edge.Attributes["target"].Value;
                tree[source].edges.Add(target);
            }
            
                // Initialize the tree recursively
            if (rootId != null)
            {
                var rootNode = _InitializeRecursively(rootId, tree);
                Debug.Log(Environment.TickCount.ToString());
                if (rootNode != null)
                    return (RootNode)rootNode;
            }
            Debug.Log("Warning! empty behaviour tree loaded");
            return null;
        }

        // Recursively goes through the tree to initialize the classes to create the overall tree structure
        private Node _InitializeRecursively(string currentId, Dictionary<string, TreeStructure> dict)
        {
            List<Node> children = new List<Node>();
                // Check if there are outgoing connectiosn
            if (dict[currentId].edges.Count > 0)
            {
                    // Initialize the node for each outgoing connection, i.e. child
                foreach (var edge in dict[currentId].edges)
                {
                    var node = _InitializeRecursively(edge, dict);
                    children.Add(node);
                }
            }

            Type type = null;
                
                // Iterate through the namespaces until the type with the className is found
            foreach (var nameSpace in nodeNamespaces)
            {
                type = Type.GetType(nameSpace + "." + dict[currentId].className);
                if (type != null)
                    break;
            }
            
                // Make sure a type is found
            if (type == null)
                throw new Exception("class name '" + dict[currentId].className +
                                    " 'can not be found. If the class is using a different namespace, make sure to add it to the protected 'nodeNamespaces' property.");
                
                // Initialize said type
            return _InitializeClass(type, dict[currentId].parameters, children);
        }
        
        
        // Creates an instance of the class 'type' with constructor parameters 'parameters'.
        // 'children' are added as children to the new class.
        // Will throw an exception if the creation of the instance failed.
        private Node _InitializeClass(Type type, string[] parameters, List<Node> children)
        {
                // Leaf takes in blackboard as a parameter
            if (type.IsAssignableTo(typeof(Leaf)))
                return _ConstructNode(type, new[] { typeof(BlackBoard) }, new object[] { _blackboard });
            
            if (type.IsAssignableTo(typeof(Composite.Composite)))
            {       
                    // Parallel takes in two parameters: successCount and failureCount, both ints
                if (type.IsAssignableTo(typeof(Parallel)))
                {
                    if (parameters == null || parameters.Length < 2)
                        throw new Exception(
                            "Node type 'Parallel' requires two parameters: requiredSuccess(int), requiredFailure(int)");
                    var successCount = Convert.ToInt32(parameters[0]);
                    var failureCount = Convert.ToInt32(parameters[1]);
                    return _ConstructNode(type, new[] { typeof(List<Node>), typeof(int), typeof(int) },
                        new object[] { children, successCount, failureCount });
                }
                    // All other composites don't take extra parameters
                return _ConstructNode(type, new[] { typeof(List<Node>) }, new object[] { children });
            }
                
                // Root node doesn't take extra parameters
            if (type.IsAssignableTo(typeof(RootNode)))
                return _ConstructNode(type, new[] { typeof(Node) },
                    new object[] { children.Count > 0 ? children[0] : null });
            
            if (type.IsAssignableTo(typeof(Decorator.Decorator)))
            {
                    // Inverter takes in a boolean parameter 'convertRunning'
                if (type.IsAssignableTo(typeof(Inverter)))
                {
                    if (parameters == null || parameters.Length < 1)
                    {
                        throw new Exception(
                            "Node type 'Inverter' requires one parameters: interpretRunningAsFailure(bool)");
                    }

                    var convertRunning = Convert.ToBoolean(parameters[0]);
                    return _ConstructNode(type, new[] { typeof(Node), typeof(bool) },
                        new object[] { children.Count > 0 ? children[0] : null, convertRunning });
                }
                
                    // Repeater takes two arguments, int repeatCount, bool exitOnFailure
                if (type.IsAssignableTo(typeof(Repeater)))
                {
                    var repeatCount = Convert.ToInt32(parameters[0]);
                    var exitOnFailure = Convert.ToBoolean(parameters[1]);
                    return _ConstructNode(type, new[] { typeof(Node), typeof(int), typeof(bool) },
                        new object[] { children.Count > 0 ? children[0] : null, repeatCount, exitOnFailure });
                }
                    // Succeeder does not take any extra arguments
                return _ConstructNode(type, new[] { typeof(Node) },
                    new object[] { children.Count > 0 ? children[0] : null });
            }
                // Throw if the type is none of the aforementioned
            throw new Exception("type '" + type +
                                "' is not derived from any Node type. If using custom node, make sure it is inherited from the appropriate node type, and not base 'Node'.");
        }

        // Calls the constructor of 'type' to create an instance of the class.
        // The constructor is called with the types 'parameterTypes' and values 'parameters' as arguments.
        private Node _ConstructNode(Type type, Type[] parameterTypes, object[] parameters)
        {
                // Extract the constructor based on the types the function takes as parameters
            var constructor = type.GetConstructor(parameterTypes);
                // Make sure a constructor was found
            if (constructor == null)
                throw new Exception("No constructor found for type '" + type + "' with parameters {" + string.Join(",",
                    parameterTypes.Select(x => x.ToString()).ToArray()) + "}.");
                // Call the constructor with the given parameters
            var node = constructor.Invoke(parameters);

            return (Node)node;
        }
        
    }
}
