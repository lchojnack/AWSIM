using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;

namespace AWSIM.TrafficSimulationECS
{

    [BurstCompile]
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct NPCVehicleSpawnerSystem : ISystem
    {

        private EntityQuery _roadNodeGroup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NPCVehicleSpawnerComponent>();   
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // state.Enabled = false;
            var spawnerEntity = SystemAPI.GetSingletonEntity<NPCVehicleSpawnerComponent>();
            var spawner = SystemAPI.GetComponentRW<NPCVehicleSpawnerComponent>(spawnerEntity);
            var config = SystemAPI.GetComponentRW<NPCVehicleConfigComponent>(spawnerEntity);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            // for (var i = 0; i < spawner.ValueRO.maxVehicleCount; i++)
            if (spawner.ValueRW.spawnedVehicle < spawner.ValueRO.maxVehicleCount)
            {
                var npcPrefabs = state.EntityManager.GetBuffer<NpcPrefabs>(spawnerEntity);
                var npcPrefab = npcPrefabs[UnityEngine.Random.Range(0, npcPrefabs.Length)];
                if (config.ValueRO.debugMode)
                {
                    npcPrefab = npcPrefabs[0];
                }
                var spawnLanes = state.EntityManager.GetBuffer<SpawnLanes>(spawnerEntity);
                var spawnLane = spawnLanes[UnityEngine.Random.Range(0, spawnLanes.Length)].Value;
                if(config.ValueRO.debugMode)
                {
                    spawnLane = spawnLanes[0].Value;
                }
                var waypoints = getWaypoints(ref state, spawnLane);
                Quaternion rotation = Quaternion.LookRotation(Forward(waypoints),  Vector3.up);
                var isSpawnable = IsSpawnable(ref state, waypoints[0].Value, npcPrefab.BoundsMax);
                if(isSpawnable)
                {
                    Unity.Entities.Entity newEntity = ecb.Instantiate(npcPrefab.Entity);
                    ecb.AddComponent(newEntity, new NPCVehicleComponent{
                        currentTrafficLane = spawnLane,
                        position = waypoints[0].Value,
                        yaw = rotation.eulerAngles.y,
                        waypointIndex = 0,
                        targetPoint = waypoints[1].Value,
                        width = npcPrefab.BoundsSize.x,
                        frontCenterLocalPosition = new float3 {x = 0f, y = 0f, z = npcPrefab.BoundsMax.z},
                        backCenterLocalPosition = new float3 {x = 0f, y = 0f, z = npcPrefab.BoundsMin.z},
                        config = config.ValueRO,
                        yieldPhase = NPCVehicleYieldPhase.NONE,
                        distanceToFrontVehicle = float.MaxValue,
                    });
                    ecb.SetComponent(newEntity, LocalTransform.FromPositionRotation(waypoints[0].Value, rotation));

                    spawner.ValueRW.spawnedVehicle += 1;
                }
            }
            ecb.Playback(state.EntityManager);
        }

        public static bool IsSpawnable(ref SystemState state, float3 spawnPoint, float3 bounds)
        {
            NativeArray<Unity.Entities.Entity> entities = state.EntityManager.GetAllEntities(Allocator.Temp);
            var isSpawnable = true;

            foreach (Unity.Entities.Entity entity in entities)
            {
                if(state.EntityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = state.EntityManager.GetComponentData<NPCVehicleComponent>(entity);
                    var distanceToCurrentWaypoint = GeometryUtility.Distance2D(spawnPoint, npc.position);
                    var isClose = distanceToCurrentWaypoint <= 2f + bounds.z;
                    if(isClose)
                    {
                        return false;
                    }
                }
            }

            return isSpawnable;
        }

        private float3 Forward(DynamicBuffer<Waypoints> waypoints)
        {
            var waypointIndex = 0;
            var Position = waypoints[waypointIndex].Value;
            var rotation = waypointIndex == waypoints.Length - 1
                ? Position - waypoints[waypointIndex - 1].Value
                : waypoints[waypointIndex + 1].Value - Position;
            return rotation;
        } 

        private DynamicBuffer<Waypoints> getWaypoints(ref SystemState state, TrafficLaneComponent trafficLane)
        {
            EntityManager entityManager = state.EntityManager;
            foreach (Unity.Entities.Entity entityTL in entityManager.GetAllEntities(Allocator.Temp))
            {
                if(entityManager.HasComponent<TrafficLaneComponent>(entityTL))
                {
                    var tl = entityManager.GetComponentData<TrafficLaneComponent>(entityTL);
                    if(tl.trafficLaneId == trafficLane.trafficLaneId)
                    {
                        var waypoints = entityManager.GetBuffer<Waypoints>(entityTL);
                        return waypoints;
                    }
                }
            }
            return new DynamicBuffer<Waypoints>();
        }
    }
}
