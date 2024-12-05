using Unity.Entities;
using UnityEngine;
using System.Collections.Generic;
using AWSIM;

using Unity.Collections;
using Unity.Mathematics;

namespace AWSIM.TrafficSimulationECS
{
    public class TrafficManagerECS : MonoBehaviour, ITrafficManagerTest
    {
        public bool debugMode = false;

        [SerializeField, Tooltip("Seed value for random generator.")]
        public int seed;
        [Header("NPC Vehicle Settings")]
        [SerializeField] public AWSIM.TrafficSimulation.NPCVehicleConfig vehicleConfig = AWSIM.TrafficSimulation.NPCVehicleConfig.Default();

        // [SerializeField, Tooltip("Vehicle layer for raytracing the collision distances.")]
        // private LayerMask vehicleLayerMask;

        // [SerializeField, Tooltip("Ground layer for raytracing the collision distances.")]
        // private LayerMask groundLayerMask;

        [SerializeField, Tooltip("A maximum number of vehicles that can simultaneously live in the scene. Lowering this value results in less dense traffic but improves the simulator's performance.")]
        public int maxVehicleCount = 100;
        public int targetVehicleCount = 10;
        public int currentVehicleCount = 0;


        public RandomTrafficSimulatorConfiguration[] randomTrafficSims;

        public void RestartTraffic()
        {

        }

        public void setMaxVehicleCount(int max)
        {
            maxVehicleCount = max;
        }
        public void setTargetVehicleCount(int target)
        {
            targetVehicleCount = target;
        }
        public int getCurrentVehicleCount()
        {
            return currentVehicleCount;
        }
    }

    public class TrafficManagerECSBaker : Baker<TrafficManagerECS>
    {
        public override void Bake(TrafficManagerECS authoring)
        {
            foreach (var randomTrafficSim in authoring.randomTrafficSims)
            {
                var allTrafficLanes = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.TrafficLane>();
                var trafficLanesDatabase = new Dictionary<string, Unity.Entities.Entity>();

                foreach (var trafficLane in allTrafficLanes)
                {
                    var tlEntity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: trafficLane.name);
                    trafficLanesDatabase.Add(trafficLane.name, tlEntity);
                    AddComponent(tlEntity, toTrafficLaneComponent(trafficLane));
                    AddBuffer<Waypoints>(tlEntity);
                    foreach(var waypoint in trafficLane.Waypoints)
                    {
                        AppendToBuffer(tlEntity, new Waypoints { Value = waypoint});
                    }
                }

                foreach (var trafficLane in allTrafficLanes)
                {
                    var tlEntity = trafficLanesDatabase[trafficLane.name];
                    AddBuffer<NextLanes>(tlEntity);
                    foreach(var nextLane in trafficLane.NextLanes)
                    {
                        if(toID(nextLane) == -1)
                        {
                            continue;
                        }
                        AppendToBuffer(tlEntity, new NextLanes { 
                            Entity = trafficLanesDatabase[nextLane.name]
                        });
                    }
                }

                var spawner = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: "NpcSpawner");
                AddComponent(spawner, new NPCVehicleSpawnerComponent
                {
                    seed = authoring.seed,
                    maxVehicleCount = authoring.maxVehicleCount,
                    targetVehicleCount = authoring.targetVehicleCount,
                    currentVehicleCount = 0,
                });
                AddComponent(spawner, new NPCVehicleConfigComponent
                {
                    acceleration = authoring.vehicleConfig.Acceleration,
                    deceleration = authoring.vehicleConfig.Deceleration,
                    suddenDeceleration = authoring.vehicleConfig.SuddenDeceleration,
                    absoluteDeceleration = authoring.vehicleConfig.AbsoluteDeceleration,
                    yawSpeedMultiplier = AWSIM.TrafficSimulation.NPCVehicleConfig.YawSpeedMultiplier,
                    yawSpeedLerpFactor = AWSIM.TrafficSimulation.NPCVehicleConfig.YawSpeedLerpFactor,
                    slowSpeed = AWSIM.TrafficSimulation.NPCVehicleConfig.SlowSpeed,
                    debugMode = authoring.debugMode
                });

                AddBuffer<SpawnLanes>(spawner);
                foreach(var spawnLane in randomTrafficSim.spawnableLanes)
                {
                    AppendToBuffer(spawner, new SpawnLanes {
                        Entity = trafficLanesDatabase[spawnLane.name]
                    });
                }
                AddBuffer<NpcPrefabs>(spawner);
                foreach(var npcPrefab in randomTrafficSim.npcPrefabs)
                {
                    AppendToBuffer(spawner, new NpcPrefabs { 
                        Entity = GetEntity(npcPrefab, TransformUsageFlags.Dynamic),
                        BoundsCenter = npcPrefab.GetComponent<NPCVehicle>().Bounds.center,
                        BoundsExtents = npcPrefab.GetComponent<NPCVehicle>().Bounds.extents,
                        BoundsMax = npcPrefab.GetComponent<NPCVehicle>().Bounds.max,
                        BoundsMin = npcPrefab.GetComponent<NPCVehicle>().Bounds.min,
                        BoundsSize = npcPrefab.GetComponent<NPCVehicle>().Bounds.size,
                    });
                }

            }


        }

        private TrafficLaneComponent toTrafficLaneComponent(AWSIM.TrafficSimulation.TrafficLane trafficLane)
        {
            if(trafficLane != null)
            {
                return new TrafficLaneComponent {
                    trafficLaneId = toID(trafficLane),
                    turnDirection = toTurnDirectionType(trafficLane.TurnDirection),
                    speedLimit = trafficLane.SpeedLimit,
                    intersectionLane = trafficLane.intersectionLane
                };
            }
            return new TrafficLaneComponent{trafficLaneId = -1};
        }

        private TurnDirectionType toTurnDirectionType(AWSIM.TrafficSimulation.TrafficLane.TurnDirectionType turnDirection)
        {
            switch(turnDirection)
            {
                case(AWSIM.TrafficSimulation.TrafficLane.TurnDirectionType.STRAIGHT):
                    return TurnDirectionType.STRAIGHT;
                case(AWSIM.TrafficSimulation.TrafficLane.TurnDirectionType.LEFT):
                    return TurnDirectionType.LEFT;
                case(AWSIM.TrafficSimulation.TrafficLane.TurnDirectionType.RIGHT):
                    return TurnDirectionType.RIGHT;
                case(AWSIM.TrafficSimulation.TrafficLane.TurnDirectionType.NULL):
                    return TurnDirectionType.NULL;
            }

            return TurnDirectionType.NULL;
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