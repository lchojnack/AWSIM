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
            var spawnerEntity = SystemAPI.GetSingletonEntity<NPCVehicleSpawnerComponent>();
            var spawner = SystemAPI.GetComponentRW<NPCVehicleSpawnerComponent>(spawnerEntity);
            var config = SystemAPI.GetComponentRW<NPCVehicleConfigComponent>(spawnerEntity);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            if (spawner.ValueRW.currentVehicleCount < spawner.ValueRO.maxVehicleCount)
            {
                if(spawner.ValueRW.currentVehicleCount >= spawner.ValueRO.targetVehicleCount)
                {
                    return;
                }
                var npcPrefabs = state.EntityManager.GetBuffer<NpcPrefabs>(spawnerEntity);
                var npcPrefab = npcPrefabs[UnityEngine.Random.Range(0, npcPrefabs.Length)];
                if (config.ValueRO.debugMode)
                {
                    npcPrefab = npcPrefabs[0];
                }
                var spawnLanes = state.EntityManager.GetBuffer<SpawnLanes>(spawnerEntity);
                var randomIndex = UnityEngine.Random.Range(0, spawnLanes.Length);
                if(config.ValueRO.debugMode)
                {
                    randomIndex = 0;
                }
                var spawnLaneEntity = spawnLanes[randomIndex].Entity;
                var waypoints = state.EntityManager.GetBuffer<Waypoints>(spawnLaneEntity);
                Quaternion rotation = Quaternion.LookRotation(Forward(waypoints),  Vector3.up);
                var isSpawnable = IsSpawnable(ref state, waypoints[0].Value, npcPrefab.BoundsMax);
                if(isSpawnable)
                {
                    Unity.Entities.Entity newEntity = ecb.Instantiate(npcPrefab.Entity);
                    ecb.AddComponent(newEntity, new NPCVehicleComponent{
                        currentTrafficLane = spawnLaneEntity,
                        position = waypoints[0].Value,
                        yaw = rotation.eulerAngles.y,
                        waypointIndex = 1,
                        targetPoint = waypoints[1].Value,
                        width = npcPrefab.BoundsSize.x,
                        frontCenterLocalPosition = new float3 {x = 0f, y = 0f, z = npcPrefab.BoundsMax.z},
                        backCenterLocalPosition = new float3 {x = 0f, y = 0f, z = npcPrefab.BoundsMin.z},
                        config = config.ValueRO,
                        yieldPhase = NPCVehicleYieldPhase.NONE,
                        distanceToFrontVehicle = float.MaxValue,
                    });
                    ecb.SetComponent(newEntity, LocalTransform.FromPositionRotation(waypoints[0].Value, rotation));

                    spawner.ValueRW.currentVehicleCount += 1;
                }
            }
            ecb.Playback(state.EntityManager);
        }

        public bool IsSpawnable(ref SystemState state, float3 spawnPoint, float3 bounds)
        {
            NativeArray<Unity.Entities.Entity> entities = state.EntityManager.GetAllEntities(Allocator.Temp);
            var isSpawnable = true;

            foreach (Unity.Entities.Entity entity in entities)
            {
                if(state.EntityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = state.EntityManager.GetComponentData<NPCVehicleComponent>(entity);
                    var distanceToCurrentWaypoint = GeometryUtility.Distance2D(spawnPoint, npc.position);
                    // var isClose = distanceToCurrentWaypoint <= (2.0f*bounds.z);
                    var isClose = distanceToCurrentWaypoint <= 0.1;
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
    }
}
