using Unity.Entities;
using UnityEngine;
using System.Collections.Generic;

using Unity.Collections;
using Unity.Mathematics;

namespace AWSIM.TrafficSimulationECS
{

    public class TrafficManagerECS : MonoBehaviour
    {
        [SerializeField, Tooltip("Seed value for random generator.")]
        public int seed;
        // [Header("NPC Vehicle Settings")]
        // [SerializeField] private NPCVehicleConfig vehicleConfig = NPCVehicleConfig.Default();

        // [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        // private LayerMask vehicleLayerMask;

        // [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        // private LayerMask groundLayerMask;

        [SerializeField, Tooltip("A maximum number of vehicles that can simultaneously live in the scene. Lowering this value results in less dense traffic but improves the simulator's performance.")]
        public int maxVehicleCount = 100;
        public int targetVehicleCount = 10;

        // [SerializeField, Tooltip("Ego vehicle handler. If not set, the manager creates a dummy ego. This reference is also set automatically when the Ego spawns via the traffic simulator.")]
        // private GameObject _egoVehicle;

        // public GameObject egoVehicle
        // {
        //     get => _egoVehicle;
        //     set
        //     {
        //         _egoVehicle = value;
        //         if (_egoVehicle != null)
        //         {
        //             NpcVehicleSimulator.RegisterEgo(value);
        //         }
        //         else
        //         {
        //             NpcVehicleSimulator.UnregisterEgo();
        //             _egoVehicle = _dummyEgo;
        //         }
        //     }
        // }

        [Header("Debug")]
        [SerializeField] protected bool showGizmos = false;
        [SerializeField] protected bool showYieldingPhase = false;
        [SerializeField] protected bool showObstacleChecking = false;
        [SerializeField] protected bool showSpawnPoints = false;
        public RandomTrafficSimulatorConfiguration[] randomTrafficSims;
        // public RouteTrafficSimulatorConfiguration[] routeTrafficSims;
        // public NPCVehicleSimulator NpcVehicleSimulator;
        // private List<ITrafficSimulator> _trafficSimulatorNodes;
        // private Dictionary<NPCVehicleSpawnPoint, Dictionary<ITrafficSimulator, GameObject>> _spawnLanes;
        private GameObject _dummyEgo;

        // public List<TrafficLaneComponent> TrafficLaneComponents;
    }

    public class TrafficManagerECSBaker : Baker<TrafficManagerECS>
    {
        public override void Bake(TrafficManagerECS authoring)
        {

            // var spawnLane = toTrafficLaneComponent(authoring.randomTrafficSims[0].spawnableLanes[0]);
            var spawner = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: "NpcSpawner");
            AddComponent(spawner, new NPCVehicleSpawnerComponent
            {
                seed = authoring.seed,
                maxVehicleCount = authoring.maxVehicleCount,
                npcPrefab = GetEntity(authoring.randomTrafficSims[0].npcPrefabs[0], TransformUsageFlags.Dynamic),
                // spawnLane = spawnLane,
            });
            AddBuffer<SpawnLanes>(spawner);
            foreach(var spawnLane in authoring.randomTrafficSims[0].spawnableLanes)
            {
                AppendToBuffer(spawner, new SpawnLanes { Value = toTrafficLaneComponent(spawnLane)});
            }
            AddBuffer<NpcPrefabs>(spawner);
            foreach(var npcPrefab in authoring.randomTrafficSims[0].npcPrefabs)
            {
                AppendToBuffer(spawner, new NpcPrefabs { Value = GetEntity(npcPrefab, TransformUsageFlags.Dynamic)});
            }

            var allTrafficLanes = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.TrafficLane>();
            foreach (var trafficLane in allTrafficLanes)
            {
                var tlEntity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: trafficLane.name);
                AddComponent(tlEntity, toTrafficLaneComponent(trafficLane));
                AddBuffer<Waypoints>(tlEntity);
                foreach(var waypoint in trafficLane.Waypoints)
                {
                    AppendToBuffer(tlEntity, new Waypoints { Value = waypoint});
                }
                AddBuffer<NextLanes>(tlEntity);
                foreach(var nextLane in trafficLane.NextLanes)
                {
                    AppendToBuffer(tlEntity, new NextLanes { Value = toTrafficLaneComponent(nextLane)});
                }
            }


        }

        private TrafficLaneComponent toTrafficLaneComponent(AWSIM.TrafficSimulation.TrafficLane trafficLane)
        {
            if(trafficLane != null)
            {
                return new TrafficLaneComponent {
                    trafficLaneId = toID(trafficLane),
                    turnDirection = (int)trafficLane.TurnDirection,
                    startPoint = trafficLane.Waypoints[0],
                    endPoint = trafficLane.Waypoints[trafficLane.Waypoints.Length-1],
                    // nextLane = getRandomNextLane(trafficLane.NextLanes),
                    speedLimit = trafficLane.SpeedLimit,
                    intersectionLane = trafficLane.intersectionLane
                };
            }
            return new TrafficLaneComponent{trafficLaneId = -1};
        }

        private int getRandomNextLane(List<AWSIM.TrafficSimulation.TrafficLane> trafficLanes)
        {
            int id = -1;
            if(trafficLanes.Count > 0)
            {
                id = toID(trafficLanes[0]);
            }
            return id;
        }

        private int toID(AWSIM.TrafficSimulation.TrafficLane tl)
        {
            int id = -1;
            if(tl != null)
            {
                var name = tl.name;
                string[] splitArray =  name.Split(char.Parse("."));
                var idStr = splitArray[1];
                id = System.Convert.ToInt32(idStr);
            }
            return id;
        }
    }

}