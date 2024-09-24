using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;
using System.Collections.Generic;


[BurstCompile]
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial struct NPCVehicleSystem : ISystem
{
    // private Dictionary<int, float3[]> waypoints;
    // Dictionary<int, DynamicBuffer<NextLanes>> nextLanes;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<NPCVehicleComponent>();
        state.RequireForUpdate<TrafficLaneComponent>();
        // cache TrafficLightsComponentsData

        // trafficLanes = state.EntityManager.GetBuffer<NextLanes>();
        // EntityManager entityManager = state.EntityManager;
        // NativeArray<Entity> entities = entityManager.GetAllEntities(Allocator.Temp);
        // NativeArray<Entity> entities = state.entityManager.GetAllEntities(Allocator.Temp);

        // foreach (Entity entity in entities)
        // {
        //     if(state.entityManager.HasComponent<TrafficLaneComponent>(entity))
        //     {
        //         TrafficLaneComponent tl = state.entityManager.GetComponentData<TrafficLaneComponent>(entity);
        //         var waypoints = state.entityManager.GetBuffer<Waypoints>(entity);

        //         waypoints.Add(tl.trafficLaneId, waypoints);
        //     }
        // }
        
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        EntityManager entityManager = state.EntityManager;
        NativeArray<Entity> entities = entityManager.GetAllEntities(Allocator.Temp);

        foreach (Entity entity in entities)
        {
            if(entityManager.HasComponent<NPCVehicleComponent>(entity))
            {
                NPCVehicleComponent npc = entityManager.GetComponentData<NPCVehicleComponent>(entity);
                LocalTransform localTransform = entityManager.GetComponentData<LocalTransform>(entity);

                // npc movement 
                npc.moveSpeed = npc.currentTrafficLane.speedLimit / 2;

                float3 moveDirection = localTransform.Forward() * SystemAPI.Time.DeltaTime * npc.moveSpeed;
                var newPosition = localTransform.Position + moveDirection;

                var distance2D = GeometryUtility.Distance2D(npc.goToPosition, newPosition);
                if(distance2D < 1)
                {
                    var nextTrafficLane = getNextTrafficLane(entityManager, entities, npc);
                    if(nextTrafficLane.trafficLaneId == npc.currentTrafficLane.trafficLaneId)
                    {
                        // TODO despawn entity
                    }
                    npc.currentTrafficLane = nextTrafficLane;
                    npc.goToPosition = npc.currentTrafficLane.endPoint;
                }
                else
                {
                    localTransform.Position = newPosition;
                    localTransform.Rotation = Quaternion.LookRotation(npc.goToPosition - localTransform.Position);
                    entityManager.SetComponentData<LocalTransform>(entity, localTransform);
                }


                entityManager.SetComponentData<NPCVehicleComponent>(entity, npc);
            }


        }
    }

    private TrafficLaneComponent getNextTrafficLane(EntityManager entityManager, NativeArray<Entity> entities, NPCVehicleComponent npc)
    {
        foreach (Entity entityTL in entities)
        {
            if(entityManager.HasComponent<TrafficLaneComponent>(entityTL))
            {
                var trafficLane = entityManager.GetComponentData<TrafficLaneComponent>(entityTL);
                if(trafficLane.trafficLaneId == npc.currentTrafficLane.trafficLaneId)
                {
                    var nextLanes = entityManager.GetBuffer<NextLanes>(entityTL);
                    if(nextLanes.Length == 0)
                    {
                        return npc.currentTrafficLane;
                    }
                    // TODO make it random
                    var nextLane = nextLanes[0].Value;
                    return nextLane;
                }
            }
        }
        return npc.currentTrafficLane;
    }
}
