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

        private void OnDrawGizmos()
        {
            var allTrafficLanes = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.TrafficLane>();
            foreach (var trafficLane in allTrafficLanes)
            {
                DrawGizmoNonSelected(trafficLane);
            }
            // var allStopLines = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.StopLine>();
            // foreach (var stopLine in allStopLines)
            // {
            //     DrawGizmo(stopLine);
            // }
        }

        private static void DrawGizmoNonSelected(AWSIM.TrafficSimulation.TrafficLane trafficLane)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(trafficLane.Waypoints[0], 0.3f);
            for (int i = 1; i < trafficLane.Waypoints.Length; ++i)
            {
                Gizmos.DrawLine(trafficLane.Waypoints[i - 1], trafficLane.Waypoints[i]);
                Gizmos.DrawSphere(trafficLane.Waypoints[i], 0.3f);
            }

            Gizmos.color = Color.yellow;

            foreach (var lane in trafficLane.RightOfWayLanes)
            {
                if (lane == null)
                {
                    Debug.LogWarning("NullReferenceException! Please check TrafficLaneEditor.cs's RightOfWay fields for empty values.");
                    continue;
                }

                Gizmos.DrawSphere(lane.Waypoints[0], 0.4f);
                for (int i = 1; i < lane.Waypoints.Length; ++i)
                {
                    Gizmos.DrawLine(lane.Waypoints[i - 1], lane.Waypoints[i]);
                    Gizmos.DrawSphere(lane.Waypoints[i], 0.4f);
                }
            }
        }

        private static void DrawGizmo(AWSIM.TrafficSimulation.StopLine stopLine)
        {
            var matCache = Gizmos.matrix;
            var colorCache = Gizmos.color;

            var direction = stopLine.Points[1] - stopLine.Points[0];
            var center = (stopLine.Points[0] + stopLine.Points[1]) / 2;
            var rotation = Quaternion.LookRotation(direction);
            Gizmos.matrix = Matrix4x4.TRS(center, rotation, Vector3.one);
            Gizmos.color = stopLine.HasStopSign ? Color.red : Color.white;
            var size = new Vector3(0.3f, 1f, direction.magnitude);
            Gizmos.DrawCube(Vector3.zero, size);

            Gizmos.matrix = matCache;
            Gizmos.color = colorCache;
        }
    }

    public class TrafficManagerECSBaker : Baker<TrafficManagerECS>
    {
        public override void Bake(TrafficManagerECS authoring)
        {
            foreach (var randomTrafficSim in authoring.randomTrafficSims)
            {
                var allTrafficLights = GameObject.FindObjectsOfType<AWSIM.TrafficLight>();
                var trafficLightEntities = new Dictionary<string, Unity.Entities.Entity>();
                foreach (var trafficLight in allTrafficLights)
                {
                    var entity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: trafficLight.name);
                    trafficLightEntities.Add(trafficLight.name, entity);
                    // AddComponent(entity, new TrafficLightComponent{});
                    AddBuffer<BulbData>(entity);
                    AppendToBuffer(entity, new BulbData { 
                        type = BulbType.RED_BULB,
                        color = BulbColor.RED,
                        status = BulbStatus.SOLID_ON,
                    });
                }

                var allStopLines = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.StopLine>();
                var stopLinesEntities = new Dictionary<string, Unity.Entities.Entity>();
                foreach (var stopLine in allStopLines)
                {
                    var entity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: stopLine.name);
                    stopLinesEntities.Add(stopLine.name, entity);
                    var stopLineComponent = toStopLineComponent(stopLine);
                    if(stopLine.TrafficLight != null)
                    {
                        stopLineComponent.trafficLight = trafficLightEntities[stopLine.TrafficLight.name];
                    }
                    AddComponent(entity, stopLineComponent);
                    AddBuffer<Points>(entity);
                    foreach(var point in stopLine.Points)
                    {
                        AppendToBuffer(entity, new Points { Value = point});
                    }
                }

                var allTrafficLanes = GameObject.FindObjectsOfType<AWSIM.TrafficSimulation.TrafficLane>();
                var trafficLanesEntities = new Dictionary<string, Unity.Entities.Entity>();

                foreach (var trafficLane in allTrafficLanes)
                {
                    var tlEntity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, entityName: trafficLane.name);
                    trafficLanesEntities.Add(trafficLane.name, tlEntity);
                    var trafficLaneComponent = toTrafficLaneComponent(trafficLane);
                    if(trafficLane.StopLine != null)
                    {
                        trafficLaneComponent.stopLine = stopLinesEntities[trafficLane.StopLine.name];
                    }
                    AddComponent(tlEntity, trafficLaneComponent);
                    AddBuffer<Waypoints>(tlEntity);
                    foreach(var waypoint in trafficLane.Waypoints)
                    {
                        AppendToBuffer(tlEntity, new Waypoints { Value = waypoint});
                    }
                }

                foreach (var trafficLane in allTrafficLanes)
                {
                    var tlEntity = trafficLanesEntities[trafficLane.name];
                    AddBuffer<NextLanes>(tlEntity);
                    foreach(var nextLane in trafficLane.NextLanes)
                    {
                        if(toID(nextLane) == -1)
                        {
                            continue;
                        }
                        AppendToBuffer(tlEntity, new NextLanes { 
                            Entity = trafficLanesEntities[nextLane.name]
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
                        Entity = trafficLanesEntities[spawnLane.name]
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
                    turnDirection = toTurnDirectionType(trafficLane.TurnDirection),
                    speedLimit = trafficLane.SpeedLimit,
                    intersectionLane = trafficLane.intersectionLane
                };
            }
            return new TrafficLaneComponent{};
        }

        private StopLineComponent toStopLineComponent(AWSIM.TrafficSimulation.StopLine stopLine)
        {
            if(stopLine != null)
            {
                return new StopLineComponent {
                    hasStopSign = stopLine.HasStopSign,
                    centerPoint = (stopLine.Points[0] + stopLine.Points[1]) / 2f
                };
            }
            return new StopLineComponent{};
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