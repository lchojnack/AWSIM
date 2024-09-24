using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Burst.Intrinsics;
using UnityEngine;

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

        EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

        for (var i = 0; i < spawner.ValueRO.maxVehicleCount; i++)
        {
            var npcPrefabs = state.EntityManager.GetBuffer<NpcPrefabs>(spawnerEntity);
            // TODO make it random
            var npcPrefab = npcPrefabs[0].Value;

            Entity newEntity = ecb.Instantiate(npcPrefab);
            // FixedString64Bytes newName = $"NPC.{i}";
            // ecb.SetName(newEntity, newName);

            var spawnLanes = state.EntityManager.GetBuffer<SpawnLanes>(spawnerEntity);
            // TODO make it random
            var spawnLane = spawnLanes[0].Value;
            // var waypoints = state.EntityManager.GetBuffer<Waypoints>(spawnLane);
            ecb.AddComponent(newEntity, new NPCVehicleComponent{
                currentTrafficLane = spawnLane,
                goToPosition = spawnLane.endPoint,
                moveSpeed = spawnLane.speedLimit
            });
            ecb.SetComponent(newEntity, new Unity.Transforms.LocalTransform{
                Position = spawnLane.startPoint,
                Scale = 1,
                Rotation = Quaternion.LookRotation(spawnLane.startPoint - spawner.ValueRO.spawnLane.endPoint)
            });
        }
        ecb.Playback(state.EntityManager);
    }
}
