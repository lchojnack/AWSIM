using UnityEngine;
using System;

namespace AWSIM.TrafficSimulationECS
{
    [Serializable]
    public struct RandomTrafficSimulatorConfiguration
    {
        /// <summary>
        /// Available NPC prefabs
        /// </summary>
        [Tooltip("NPCs to be spawned.")]
        public GameObject[] npcPrefabs;

        [Tooltip("TrafficLanes where NPC vehicles can spawn.")]
        public AWSIM.TrafficSimulation.TrafficLane[] spawnableLanes;

        [Tooltip("Describes the lifetime of a traffic simulator instance by specifying how many vehicles this traffic simulator will spawn. Setting it makes the spawner live longer or shorter, while it can also be set to infinity if needed (endless lifetime).")]
        public int maximumSpawns;

        [Tooltip("Is this traffic simulation enabled.")]
        public bool enabled;
    }

}
