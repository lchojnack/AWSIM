using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Physics;

namespace AWSIM.TrafficSimulationECS
{

    [BurstCompile]
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct NPCVehicleMeshColliderSystem : ISystem
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
            NativeArray<Unity.Entities.Entity> entities = state.EntityManager.GetAllEntities(Allocator.Temp);
            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
            foreach (Unity.Entities.Entity entity in entities)
            {
                if(state.EntityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = state.EntityManager.GetComponentData<NPCVehicleComponent>(entity);

                    if(npc.meshColliderComponent == Unity.Entities.Entity.Null)
                    {
                        var links = state.EntityManager.GetBuffer<LinkedEntityGroup>(entity);
                        foreach(var link in links)
                        {
                            if(state.EntityManager.HasComponent<PhysicsCollider>(link.Value))
                            {
                                npc.meshColliderComponent = link.Value;
                                state.EntityManager.SetComponentData<NPCVehicleComponent>(entity, npc);
                            }
                        }
                    }
                }
            }
            ecb.Playback(state.EntityManager);
        }

    }
}
