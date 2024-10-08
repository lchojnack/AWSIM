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
    public partial struct NPCVehicleDespawnerSystem : ISystem
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

            NativeArray<Unity.Entities.Entity> entities = state.EntityManager.GetAllEntities(Allocator.Temp);
            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (Unity.Entities.Entity entity in entities)
            {
                if(state.EntityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = state.EntityManager.GetComponentData<NPCVehicleComponent>(entity);
                    if(npc.shouldDespawn)
                    {
                        ecb.DestroyEntity(entity);
                        spawner.ValueRW.spawnedVehicle -= 1;
                    }
                }
            }
            ecb.Playback(state.EntityManager);
        }

    }
}
