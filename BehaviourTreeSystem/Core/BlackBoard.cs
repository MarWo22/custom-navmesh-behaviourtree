using System.Collections.Generic;

namespace Shard.BehaviourTreeSystem.Core
{
    public class BlackBoard
    {
        // Data is stored as a string-object dictionary
        private readonly Dictionary<string, object> _data = new Dictionary<string, object>();

        // Attempts to retrieve the key value with type T, will return default value of type T if failed
        public T Get<T>(string key)
        {
            if (_data.TryGetValue(key, out var value) && value is T casted)
                return casted;
            return default;
        }

        // Sets the key with value of type T
        public void Set<T>(string key, T value)
        {
            Debug.Log(value.ToString());
            _data[key] = value;
        }

        // Deletion is never necessary for blackboards, thus the implementation is omitted.
    }
}
