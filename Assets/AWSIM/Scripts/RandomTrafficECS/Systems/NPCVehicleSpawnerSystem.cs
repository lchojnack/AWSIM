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
            // state.Enabled = false;
            var spawnerEntity = SystemAPI.GetSingletonEntity<NPCVehicleSpawnerComponent>();
            var spawner = SystemAPI.GetComponentRW<NPCVehicleSpawnerComponent>(spawnerEntity);
            var config = SystemAPI.GetComponentRW<NPCVehicleConfigComponent>(spawnerEntity);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            // for (var i = 0; i < spawner.ValueRO.maxVehicleCount; i++)
            if (spawner.ValueRW.spawnedVehicle < spawner.ValueRO.maxVehicleCount)
            {
                var npcPrefabs = state.EntityManager.GetBuffer<NpcPrefabs>(spawnerEntity);
                // TODO make it random
                var npcPrefab = npcPrefabs[UnityEngine.Random.Range(0, npcPrefabs.Length)];
                // var npcPrefab = npcPrefabs[0];

                // FixedString64Bytes newName = $"NPC.{i}";
                // ecb.SetName(newEntity, newName);

                var spawnLanes = state.EntityManager.GetBuffer<SpawnLanes>(spawnerEntity);
                // TODO make it random
                var spawnLane = spawnLanes[UnityEngine.Random.Range(0, spawnLanes.Length)].Value;
                // var spawnLane = spawnLanes[0].Value;
                var waypoints = getWaypoints(ref state, spawnLane);
                Quaternion rotation = Quaternion.LookRotation(Forward(waypoints),  Vector3.up);
                var isSpawnable = IsSpawnable(ref state, spawnLane);
                // var isSpawnable = IsSpawnable(npcPrefab.BoundsCenter, npcPrefab.BoundsExtents, Forward(waypoints) ,waypoints[0].Value);
                // FixedString64Bytes newName = $"isSpawnable {isSpawnable}";
                // Debug.Log(newName);
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

        public static bool IsSpawnable(ref SystemState state, TrafficLaneComponent spawnLane)
        {
            NativeArray<Unity.Entities.Entity> entities = state.EntityManager.GetAllEntities(Allocator.Temp);
            var isSpawnable = true;

            foreach (Unity.Entities.Entity entity in entities)
            {
                if(state.EntityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = state.EntityManager.GetComponentData<NPCVehicleComponent>(entity);
                    if(npc.currentTrafficLane.trafficLaneId == spawnLane.trafficLaneId)
                    {
                        return false;
                    }
                }
            }

            return isSpawnable;
        }

        public static bool IsSpawnable(float3 BoundsCenter, float3 BoundsExtents, float3 npcVehicleSpawnPointForward, float3 npcVehicleSpawnPointPosition)
        {
            var npcPos = new Vector3{x = npcVehicleSpawnPointPosition.x, y = npcVehicleSpawnPointPosition.y, z = npcVehicleSpawnPointPosition.z};
            var rotation = Quaternion.LookRotation(npcVehicleSpawnPointForward);
            var center = rotation * BoundsCenter + npcPos;
            // FixedString64Bytes mask = "Ground";
            // var ignoreGroundLayerMask = ~LayerMask.GetMask(mask);
            var ignoreGroundLayerMask = -129;
            return !Physics.CheckBox(
                center,
                BoundsExtents,
                rotation,
                ignoreGroundLayerMask,
                QueryTriggerInteraction.Ignore);
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
