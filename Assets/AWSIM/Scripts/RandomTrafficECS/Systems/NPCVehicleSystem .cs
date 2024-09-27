using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;
using System.Collections.Generic;
namespace AWSIM.TrafficSimulationECS
{

    [BurstCompile]
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct NPCVehicleSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NPCVehicleComponent>();
            state.RequireForUpdate<TrafficLaneComponent>();       
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            EntityManager entityManager = state.EntityManager;
            NativeArray<Unity.Entities.Entity> entities = entityManager.GetAllEntities(Allocator.Temp);

            foreach (Unity.Entities.Entity entity in entities)
            {
                if(entityManager.HasComponent<NPCVehicleComponent>(entity))
                {
                    NPCVehicleComponent npc = entityManager.GetComponentData<NPCVehicleComponent>(entity);
                    LocalTransform localTransform = entityManager.GetComponentData<LocalTransform>(entity);

                    NPCVehicleCognitionStep(ref npc, ref state);
                    NPCVehicleDecisionStep(ref npc, ref state);
                    NPCVehicleControlStep(ref localTransform, ref npc, ref state);              
                    NPCVehicleVisualizationStep(ref localTransform, ref npc);

                    entityManager.SetComponentData<LocalTransform>(entity, localTransform);
                    entityManager.SetComponentData<NPCVehicleComponent>(entity, npc);
                }
            }
        }

        private void NPCVehicleCognitionStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            NextWaypointCheckJob(ref npc, ref state);
        }

        private void NextWaypointCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            var distanceToCurrentWaypoint = GeometryUtility.Distance2D(npc.targetPoint, npc.position);
            var isCloseToTarget = distanceToCurrentWaypoint <= 1f;

            if(!isCloseToTarget)
            {
                return;
            }

            var waypoints = getWaypoints(ref state, npc.currentTrafficLane.trafficLaneId);
            if (npc.waypointIndex >= (waypoints.Length-1))
            {
                var nextTrafficLane = getNextTrafficLane(ref state, npc.currentTrafficLane.trafficLaneId);
                if(nextTrafficLane.trafficLaneId == -1)
                {
                    npc.shouldDespawn = true;
                }
                else
                {
                    npc.currentTrafficLane = nextTrafficLane;
                    npc.waypointIndex = 0;
                }
            }       
            else
            {
                npc.waypointIndex += 1;
                npc.targetPoint = waypoints[npc.waypointIndex].Value;
            }
        }

        private void NPCVehicleDecisionStep(ref NPCVehicleComponent npc, ref SystemState state )
        {
            if (npc.shouldDespawn)
            {
                return;
            }
            // UpdateSpeedMode
            npc.speedMode = NPCVehicleSpeedMode.NORMAL;

        }

        private void NPCVehicleControlStep(ref LocalTransform localTransform, ref NPCVehicleComponent npc, ref SystemState state)
        {
            var deltaTime = SystemAPI.Time.DeltaTime;
            UpdateSpeed(ref npc, ref state, deltaTime);
            UpdatePose(ref localTransform, ref npc, ref state, deltaTime);
            UpdateYawSpeed(ref npc, ref state, deltaTime);
        }

        private void UpdateSpeed(ref NPCVehicleComponent npc, ref SystemState state, float deltaTime)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            float targetSpeed;
            float acceleration;
            switch (npc.speedMode)
            {
                case NPCVehicleSpeedMode.NORMAL:
                    targetSpeed = npc.currentTrafficLane.speedLimit;
                    acceleration = npc.config.acceleration;
                    break;
                case NPCVehicleSpeedMode.SLOW:
                    targetSpeed = Mathf.Min(npc.config.slowSpeed, npc.currentTrafficLane.speedLimit);
                    acceleration = npc.config.deceleration;
                    break;
                case NPCVehicleSpeedMode.SUDDEN_STOP:
                    targetSpeed = 0f;
                    acceleration = npc.config.suddenDeceleration;
                    break;
                case NPCVehicleSpeedMode.ABSOLUTE_STOP:
                    targetSpeed = 0f;
                    acceleration = npc.config.absoluteDeceleration;
                    break;
                case NPCVehicleSpeedMode.STOP:
                    targetSpeed = 0f;
                    acceleration = npc.config.deceleration;
                    break;
                default:
                    targetSpeed = 0f;
                    acceleration = npc.config.deceleration;
                    break;
            }

            npc.speed = Mathf.MoveTowards(npc.speed, targetSpeed, acceleration * deltaTime);
        }

        private void UpdatePose(ref LocalTransform localTransform, ref NPCVehicleComponent npc, ref SystemState state, float deltaTime)
        {
            // LocalTransform localTransform = new LocalTransform{Position = npc.position, Rotation = };
            if (npc.shouldDespawn)
            {
                return;
            }

            npc.yaw += npc.yawSpeed * deltaTime;
            var position = npc.position;
            position += localTransform.Forward() * npc.speed * deltaTime;
            position.y = npc.targetPoint.y;
            npc.position = position;
        }

        private float3 FrontCenterPosition(ref NPCVehicleComponent npc)
        {
            var x = Quaternion.AngleAxis(npc.yaw, Vector3.up) * npc.frontCenterLocalPosition;
            var x_float3 = new float3(x);
            return npc.position + x_float3;
        }


        private float3 Forward(ref NPCVehicleComponent npc)
        {
            var x = Quaternion.AngleAxis(npc.yaw, Vector3.up) * Vector3.forward;
            return new float3(x);
        }

        private void UpdateYawSpeed(ref NPCVehicleComponent npc, ref SystemState state, float deltaTime)
        {
            // Steering the vehicle so that it heads toward the target point.
            var steeringDirection = npc.targetPoint - FrontCenterPosition(ref npc);
            steeringDirection.y = 0f;
            var steeringAngle = Vector3.SignedAngle(Forward(ref npc), steeringDirection, Vector3.up);
            var targetYawSpeed = steeringAngle * npc.speed * npc.config.yawSpeedMultiplier;
            // Change YawSpeed gradually to eliminate steering shake.
            npc.yawSpeed = Mathf.Lerp(
                npc.yawSpeed,
                targetYawSpeed,
                npc.config.yawSpeedLerpFactor * deltaTime);
        }

        private void NPCVehicleVisualizationStep(ref LocalTransform localTransform, ref NPCVehicleComponent npc)
        {
            ApplyPose(ref localTransform, ref npc);
            // ApplyTurnSignalState()
        }

        private static void ApplyPose(ref LocalTransform localTransform, ref NPCVehicleComponent npc)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            // var vehicle = state.Vehicle;
            // vehicle.SetPosition(state.Position);
            // vehicle.SetRotation(Quaternion.AngleAxis(state.Yaw, Vector3.up));

            localTransform.Position = npc.position;
            localTransform.Rotation = Quaternion.AngleAxis(npc.yaw, Vector3.up);
        }

        private TrafficLaneComponent getNextTrafficLane(ref SystemState state, int trafficLaneId)
        {
            EntityManager entityManager = state.EntityManager;
            foreach (Unity.Entities.Entity entityTL in entityManager.GetAllEntities(Allocator.Temp))
            {
                if(entityManager.HasComponent<TrafficLaneComponent>(entityTL))
                {
                    var trafficLane = entityManager.GetComponentData<TrafficLaneComponent>(entityTL);
                    if(trafficLane.trafficLaneId == trafficLaneId)
                    {
                        var nextLanes = entityManager.GetBuffer<NextLanes>(entityTL);
                        if(nextLanes.Length == 0)
                        {
                            return new TrafficLaneComponent{trafficLaneId = -1};
                        }
                        // TODO make it random
                        var nextLane = nextLanes[0].Value;
                        return nextLane;
                    }
                }
            }
            return new TrafficLaneComponent{trafficLaneId = -1};
        }


        private DynamicBuffer<Waypoints> getWaypoints(ref SystemState state, int trafficLaneId)
        {
            EntityManager entityManager = state.EntityManager;
            foreach (Unity.Entities.Entity entityTL in entityManager.GetAllEntities(Allocator.Temp))
            {
                if(entityManager.HasComponent<TrafficLaneComponent>(entityTL))
                {
                    var tl = entityManager.GetComponentData<TrafficLaneComponent>(entityTL);
                    if(tl.trafficLaneId == trafficLaneId)
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