using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

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
            state.Enabled = false;
            var spawnerEntity = SystemAPI.GetSingletonEntity<NPCVehicleSpawnerComponent>();
            var spawner = SystemAPI.GetComponentRW<NPCVehicleSpawnerComponent>(spawnerEntity);
            var config = SystemAPI.GetComponentRW<NPCVehicleConfigComponent>(spawnerEntity);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            for (var i = 0; i < spawner.ValueRO.maxVehicleCount; i++)
            {
                var npcPrefabs = state.EntityManager.GetBuffer<NpcPrefabs>(spawnerEntity);
                // TODO make it random

                var npcPrefab = npcPrefabs[0];

                Unity.Entities.Entity newEntity = ecb.Instantiate(npcPrefab.Entity);
                FixedString64Bytes newName = $"NPC.{i}";
                ecb.SetName(newEntity, newName);

                var spawnLanes = state.EntityManager.GetBuffer<SpawnLanes>(spawnerEntity);
                // TODO make it random
                var spawnLane = spawnLanes[0].Value;
                var waypoints = getWaypoints(ref state, spawnLane);
                Quaternion rotation = Quaternion.LookRotation(Forward(waypoints),  Vector3.up);
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
                });
                ecb.SetComponent(newEntity, LocalTransform.FromPositionRotation(waypoints[0].Value, rotation));
            }
            ecb.Playback(state.EntityManager);
        }

        private float3 Forward(DynamicBuffer<Waypoints> waypoints)
        {
            var waypointIndex = 0;
            var Position = waypoints[waypointIndex].Value;
            var rotation = waypointIndex == waypoints.Length - 1
                ? Position - waypoints[waypointIndex - 1].Value
                : waypoints[waypointIndex + 1].Value - Position;
            FixedString64Bytes txt = $"{rotation.x} {rotation.y} {rotation.z}";
            Debug.Log(txt);
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
