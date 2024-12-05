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
        private const float MinFrontVehicleDistance = 4f;
        private const float MinStopDistance = 1.5f;

        // dynamics settings const values.
        const float maxSteerAngle = 40f;                    // deg
        const float maxSteerSpeed = 60f;                    // deg/s
        const float maxVerticalSpeed = 40;                  // m/s
        const float maxSlope = 45;                          // deg

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NPCVehicleComponent>();
            state.RequireForUpdate<TrafficLaneComponent>();       
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            foreach (var (localTransform, npc) in SystemAPI.Query<RefRW<LocalTransform>,RefRW<NPCVehicleComponent>>())
            {
                NPCVehicleCognitionStep(ref npc.ValueRW, ref state);
                NPCVehicleDecisionStep(ref npc.ValueRW, ref state);
                NPCVehicleControlStep(ref localTransform.ValueRW, ref npc.ValueRW, ref state);              
                NPCVehicleVisualizationStep(ref localTransform.ValueRW, ref npc.ValueRW);
            }
        }

        private void NPCVehicleCognitionStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            NextWaypointCheckJob(ref npc, ref state);
            CurveCheckJob(ref npc, ref state);
        }

        private void NextWaypointCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            var distanceToCurrentWaypoint = GeometryUtility.Distance2D(npc.targetPoint, npc.position);
            npc.distanceToCurrentWaypoint = distanceToCurrentWaypoint;
            var isCloseToTarget = distanceToCurrentWaypoint <= 4f;

            if(!isCloseToTarget)
            {
                return;
            }

            var waypoints = getWaypoints(ref state, npc.currentTrafficLane.trafficLaneId);
            if (npc.waypointIndex >= (waypoints.Length-1))
            {
                // equivalent to extend following lanes
                var nextTrafficLane = getNextTrafficLane(ref state, npc.currentTrafficLane.trafficLaneId, npc.config.debugMode);
                npc.currentTrafficLane = nextTrafficLane;
                npc.waypointIndex = 1;
            }       
            else
            {
                npc.waypointIndex += 1;
            }

            if(npc.currentTrafficLane.trafficLaneId == -1)
            {
                npc.shouldDespawn = true;
            }
        }

        private void CurveCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            var currentForward = Quaternion.AngleAxis(npc.yaw, Vector3.up) * Vector3.forward;
            var waypoints = getWaypoints(ref state, npc.currentTrafficLane.trafficLaneId);
            var currentWaypointIndex = npc.waypointIndex;
            var elapsedDistance = Vector3.Distance(FrontCenterPosition(ref npc), waypoints[currentWaypointIndex].Value);
            var turnAngle = 0f;
            while (elapsedDistance < 40f)
            {
                var currentWaypoint = waypoints[currentWaypointIndex].Value;
                currentWaypointIndex++;

                if (currentWaypointIndex >= (waypoints.Length-1))
                    break;

                var nextWaypoint = waypoints[currentWaypointIndex].Value;
                var nextForward = nextWaypoint - currentWaypoint;
                elapsedDistance += Vector3.Distance(currentWaypoint, nextWaypoint);
                turnAngle += Vector3.Angle(currentForward, nextForward);
                currentForward = nextForward;
            }

            npc.isTurning = turnAngle > 45f;
        }   

        private void NPCVehicleDecisionStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            UpdateTargetPoint(ref npc, ref state);
            UpdateSpeedMode(ref npc, ref state);
        }

        private void UpdateTargetPoint(ref NPCVehicleComponent npc, ref SystemState state)
        {
            if (npc.shouldDespawn || npc.currentTrafficLane.trafficLaneId == -1)
            {
                return;
            }

            var waypoints = getWaypoints(ref state, npc.currentTrafficLane.trafficLaneId);
            npc.targetPoint = waypoints[npc.waypointIndex].Value;
        }

        private void UpdateSpeedMode(ref NPCVehicleComponent npc, ref SystemState state)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            var absoluteStopDistance = CalculateStoppableDistance(npc.speed, npc.config.absoluteDeceleration) + MinStopDistance;
            var suddenStopDistance = CalculateStoppableDistance(npc.speed, npc.config.suddenDeceleration) + 2 * MinStopDistance;
            var stopDistance = CalculateStoppableDistance(npc.speed, npc.config.deceleration) + 3 * MinStopDistance;
            var slowDownDistance = stopDistance + 4 * MinStopDistance;

            var distanceToStopPointByFrontVehicle = onlyGreaterThan(npc.distanceToFrontVehicle - MinFrontVehicleDistance, -MinFrontVehicleDistance);
            var distanceToStopPointByTrafficLight = CalculateTrafficLightDistance(ref npc, ref state, suddenStopDistance);
            var distanceToStopPointByRightOfWay = CalculateYieldingDistance(ref npc, ref state);
            var distanceToStopPoint = Mathf.Min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight);
            distanceToStopPoint = Mathf.Min(distanceToStopPoint, distanceToStopPointByRightOfWay);

            npc.isStoppedByFrontVehicle = false;
            if (distanceToStopPointByFrontVehicle <= stopDistance)
            {
                npc.isStoppedByFrontVehicle = true;
            }

            if (distanceToStopPoint <= absoluteStopDistance)
                npc.speedMode = NPCVehicleSpeedMode.ABSOLUTE_STOP;
            else if (distanceToStopPoint <= suddenStopDistance)
                npc.speedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
            else if (distanceToStopPoint <= stopDistance)
                npc.speedMode = NPCVehicleSpeedMode.STOP;
            else if (distanceToStopPoint <= slowDownDistance || npc.isTurning)
                npc.speedMode = NPCVehicleSpeedMode.SLOW;
            else
                npc.speedMode = NPCVehicleSpeedMode.NORMAL;
        }

        private float CalculateYieldingDistance(ref NPCVehicleComponent npc, ref SystemState state)
        {
            // TODO no yielding information so far
            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (npc.yieldPhase != NPCVehicleYieldPhase.NONE && npc.yieldPhase != NPCVehicleYieldPhase.ENTERING_INTERSECTION && npc.yieldPhase != NPCVehicleYieldPhase.AT_INTERSECTION)
            {
                distanceToStopPointByRightOfWay = SignedDistanceToPointOnLane(ref npc, npc.yieldPoint);
            }
            return onlyGreaterThan(distanceToStopPointByRightOfWay, 0);
        }

        private float CalculateTrafficLightDistance(ref NPCVehicleComponent npc, ref SystemState state, float suddenStopDistance)
        {
            // TODO no traffic light information so far
            var distanceToStopPointByTrafficLight = float.MaxValue;
            return onlyGreaterThan(distanceToStopPointByTrafficLight, 0);
        }

        private float CalculateStoppableDistance(float speed, float deceleration)
        {
            return onlyGreaterThan(speed * speed / 2f / deceleration, 0);
        }

        private float onlyGreaterThan(float value, float min_value = 0)
        {
            return value >= min_value ? value : float.MaxValue;
        }

        public float SignedDistanceToPointOnLane(ref NPCVehicleComponent npc, float3 point)
        {
            var position = FrontCenterPosition(ref npc);
            position.y = 0f;
            point.y = 0f;

            var forward = Forward(ref npc);
            var forwardVec = new Vector3{x = forward.x , y = forward.y, z = forward.z};
            var pointPos = point - position;
            var pointPosVec = new Vector3{x = pointPos.x , y = pointPos.y, z = pointPos.z};

            var hasPassedThePoint = Vector3.Dot(forwardVec, pointPosVec) < 0f;

            var distance = Vector3.Distance(position, point);
            return hasPassedThePoint ? -distance : distance;
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
        }

        private static void ApplyPose(ref LocalTransform localTransform, ref NPCVehicleComponent npc)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            localTransform.Position = npc.position;
            localTransform.Rotation = Quaternion.AngleAxis(npc.yaw, Vector3.up);
        }

        private float ClampDegree360(float value, float maxAbsValue)
        {
            if (value < 360f - maxAbsValue && value > 180f)
            {
                return 360f - maxAbsValue;
            }

            if (value > maxAbsValue && value <= 180f)
            {
                return maxAbsValue;
            }

            return value;
        }

        private TrafficLaneComponent getNextTrafficLane(ref SystemState state, int trafficLaneId, bool debugMode)
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
                        var nextLane = nextLanes[UnityEngine.Random.Range(0, nextLanes.Length)].Value; 
                        if (debugMode)
                        {
                            nextLane = nextLanes[0].Value; 
                        }
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