using Unity.Entities;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using Unity.Physics;
using GeometryUtility = AWSIM.Lanelet.GeometryUtility;
using System.Collections.Generic;

namespace AWSIM.TrafficSimulationECS
{

    [BurstCompile]
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial struct NPCVehicleSystem : ISystem
    {
        private const int MaxBoxcastCount = 1;

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
                NPCVehicleControlStep(ref npc.ValueRW, ref state);              
                NPCVehicleVisualizationStep(ref localTransform.ValueRW, ref npc.ValueRW);
            }
        }

        [BurstCompile]
        private void NPCVehicleCognitionStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            NextWaypointCheckJob(ref npc, ref state);
            CurveCheckJob(ref npc, ref state);
            ObstacleCheckJob(ref npc, ref state);
            CalculateObstacleDistanceJob(ref npc, ref state);
        }

        [BurstCompile]
        private void ObstacleCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            var waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);

            npc.startPoint = npc.waypointIndex == 1
                ? FrontCenterPosition(ref npc)
                : waypoints[npc.waypointIndex - 1].Value;

            // Reduce the detection range so that large sized vehicles can pass each other.
            float3 boxCastExtents = npc.extents * 0.5f;
            boxCastExtents.y *= 1;
            boxCastExtents.z = 0.1f;
            float3 endPoint = waypoints[npc.waypointIndex].Value;

            float distance = math.distance(npc.startPoint, endPoint);
            float3 direction = (endPoint - npc.startPoint);
            quaternion orientation = quaternion.LookRotationSafe(direction, math.up());

            npc.raycastHitDistance = float.MaxValue;
            npc.raycastHitPoint = float3.zero;

            npc.boxcastCommandStartPoint = npc.startPoint;
            npc.boxcastCommandDirection = direction;
            npc.boxcastCommandDistance = distance;
            npc.boxcastCommandExtents = boxCastExtents;

            // Boxcast command -> NPC <-> EGO
            // if(npc.config.debugMode)
            // {
            //     var obstacleHitInfoArray = new NativeArray<UnityEngine.RaycastHit>(MaxBoxcastCount, Allocator.TempJob);
            //     var boxcastCommands = new NativeArray<UnityEngine.BoxcastCommand>(MaxBoxcastCount, Allocator.TempJob);
            //     boxcastCommands[0] = new UnityEngine.BoxcastCommand(
            //         npc.startPoint,
            //         boxCastExtents,
            //         UnityEngine.Quaternion.LookRotation(direction),
            //         direction,
            //         distance,
            //         npc.config.vehicleLayerMask
            //     );

            //     JobHandle boxcastJobHandle = UnityEngine.BoxcastCommand.ScheduleBatch(boxcastCommands, obstacleHitInfoArray, 1);
            //     boxcastJobHandle.Complete();

            //     if (obstacleHitInfoArray[0].collider != null)
            //     {
            //         npc.raycastHitPoint = obstacleHitInfoArray[0].point;
            //         npc.raycastHitDistance = obstacleHitInfoArray[0].distance;
            //     }


            //     obstacleHitInfoArray.Dispose();
            //     boxcastCommands.Dispose();
            // }

            // ECS Boxcast -> NPC <-> NPC
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
            var hits = new NativeList<ColliderCastHit>(Allocator.Temp);

            Unity.Physics.CollisionFilter filter = new CollisionFilter{
                BelongsTo = (uint)npc.config.vehicleLayerMask.value,
                CollidesWith = (uint)npc.config.vehicleLayerMask.value
            };
            bool isCollision =  physicsWorld.BoxCastAll(
                npc.startPoint,
                orientation,
                boxCastExtents,
                direction,
                distance,
                ref hits,
                filter
            );

            if(isCollision)
            {
                var shortestDistance = float.MaxValue;
                float3 closestHitPoint = npc.raycastHitPoint;
                foreach (var hit in hits)
                {
                    var hitDistance = math.distance(npc.startPoint, hit.Position);
                    if(npc.meshColliderComponent != hit.Entity && shortestDistance > hitDistance)
                    {
                        shortestDistance = hitDistance;
                        closestHitPoint = hit.Position;
                    }
                }

                npc.raycastHitPoint = closestHitPoint;
                npc.raycastHitDistance = shortestDistance;
            }
            hits.Dispose();
        }

        [BurstCompile]
        private void CalculateObstacleDistanceJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            var waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);

            // CalculateObstacleDistanceJob starts here
            var hasHit = false;
            var totalDistance = 0f;
            var boxcastCount = UnityEngine.Mathf.Min(MaxBoxcastCount, waypoints.Length);

            for (var commandIndex = 0; commandIndex < boxcastCount; commandIndex++)
            {
                // var hit = npc.raycastHit;
                // UnityEngine.Debug.Log($"raycastHit : {hit.distance}, {hit.point}");
                hasHit = npc.raycastHitDistance != float.MaxValue || math.all(npc.raycastHitPoint != float3.zero);
                if (hasHit)
                {
                    totalDistance = math.distance(npc.startPoint, npc.raycastHitPoint);
                    break;
                }
                totalDistance = npc.boxcastCommandDistance;
            }

            npc.distanceToFrontVehicle = hasHit ? totalDistance : float.MaxValue;
            // UnityEngine.Debug.Log($"raynpc.distanceToFrontVehiclecastHit : {npc.distanceToFrontVehicle}, {totalDistance}, {hasHit}");
        }

        [BurstCompile]
        private void NextWaypointCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            var waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);
            var distanceToCurrentWaypoint = GeometryUtility.Distance2D(waypoints[npc.waypointIndex].Value, FrontCenterPosition(ref npc));
            npc.distanceToCurrentWaypoint = distanceToCurrentWaypoint;
            var isCloseToTarget = distanceToCurrentWaypoint <= npc.frontCenterLocalPosition.z;

            if(!isCloseToTarget)
            {
                return;
            }

            if (npc.waypointIndex >= (waypoints.Length-1))
            {
                // equivalent to extend following lanes
                var nextLanes = state.EntityManager.GetBuffer<NextLanes>(npc.currentTrafficLane);
                if(nextLanes.Length != 0)
                {
                    var randomIndex = UnityEngine.Random.Range(0, nextLanes.Length);
                    if(npc.config.debugMode)
                    {
                        randomIndex = 0;
                    }
                    npc.currentTrafficLane = nextLanes[randomIndex].Entity;
                    npc.waypointIndex = 1;
                }
            }       
            else
            {
                npc.waypointIndex += 1;
            }

            waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);
            distanceToCurrentWaypoint = GeometryUtility.Distance2D(waypoints[waypoints.Length-1].Value, FrontCenterPosition(ref npc));
            isCloseToTarget = distanceToCurrentWaypoint <= 2.0f;
            if(state.EntityManager.GetBuffer<NextLanes>(npc.currentTrafficLane).Length == 0 && isCloseToTarget)
            {
                npc.shouldDespawn = true;
            }
        }

        [BurstCompile]
        private void CurveCheckJob(ref NPCVehicleComponent npc, ref SystemState state)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            var currentForward = UnityEngine.Quaternion.AngleAxis(npc.yaw, UnityEngine.Vector3.up) * UnityEngine.Vector3.forward;
            var waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);
            var currentWaypointIndex = npc.waypointIndex;
            var elapsedDistance = math.distance(FrontCenterPosition(ref npc), waypoints[currentWaypointIndex].Value);
            var turnAngle = 0f;
            while (elapsedDistance < 40f)
            {
                var currentWaypoint = waypoints[currentWaypointIndex].Value;
                currentWaypointIndex++;

                if (currentWaypointIndex >= (waypoints.Length-1))
                    break;

                var nextWaypoint = waypoints[currentWaypointIndex].Value;
                var nextForward = nextWaypoint - currentWaypoint;
                elapsedDistance += math.distance(currentWaypoint, nextWaypoint);
                turnAngle += UnityEngine.Vector3.Angle(currentForward, nextForward);
                currentForward = nextForward;
            }

            npc.isTurning = turnAngle > 45f;
        }   

        [BurstCompile]
        private void NPCVehicleDecisionStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            // note: NPCVehicleDecitionStep is the same as in TrafficSimulator
            UpdateTargetPoint(ref npc, ref state);
            UpdateSpeedMode(ref npc, ref state);
        }

        [BurstCompile]
        private void UpdateTargetPoint(ref NPCVehicleComponent npc, ref SystemState state)
        {
            if (npc.shouldDespawn || npc.currentTrafficLane == null)
            {
                return;
            }

            var waypoints = state.EntityManager.GetBuffer<Waypoints>(npc.currentTrafficLane);
            npc.targetPoint = waypoints[npc.waypointIndex].Value;
        }

        [BurstCompile]
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
            var distanceToStopPoint = UnityEngine.Mathf.Min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight);
            distanceToStopPoint = UnityEngine.Mathf.Min(distanceToStopPoint, distanceToStopPointByRightOfWay);

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

        [BurstCompile]
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

        [BurstCompile]
        private float CalculateTrafficLightDistance(ref NPCVehicleComponent npc, ref SystemState state, float suddenStopDistance)
        {
            // TODO no traffic light information so far
            var distanceToStopPointByTrafficLight = float.MaxValue;
            // if (npc.TrafficLightLane != null)
            // {
            //     var distanceToStopLine =
            //         state.SignedDistanceToPointOnLane(state.TrafficLightLane.StopLine.CenterPoint);
            //     switch (state.TrafficLightPassability)
            //     {
            //         case TrafficLightPassability.GREEN:
            //             break;
            //         case TrafficLightPassability.YELLOW:
            //             if (distanceToStopLine < suddenStopDistance) break;
            //             distanceToStopPointByTrafficLight = distanceToStopLine;
            //             break;
            //         case TrafficLightPassability.RED:
            //             distanceToStopPointByTrafficLight = distanceToStopLine;
            //             break;
            //     }
            // }
            return onlyGreaterThan(distanceToStopPointByTrafficLight, 0);
        }

        [BurstCompile]
        private float CalculateStoppableDistance(float speed, float deceleration)
        {
            return onlyGreaterThan(speed * speed / 2f / deceleration, 0);
        }

        [BurstCompile]
        private float onlyGreaterThan(float value, float min_value = 0)
        {
            return value >= min_value ? value : float.MaxValue;
        }

        [BurstCompile]
        public float SignedDistanceToPointOnLane(ref NPCVehicleComponent npc, float3 point)
        {
            var position = FrontCenterPosition(ref npc);
            position.y = 0f;
            point.y = 0f;

            var forward = Forward(ref npc);
            var forwardVec = new UnityEngine.Vector3{x = forward.x , y = forward.y, z = forward.z};
            var pointPos = point - position;
            var pointPosVec = new UnityEngine.Vector3{x = pointPos.x , y = pointPos.y, z = pointPos.z};

            var hasPassedThePoint = UnityEngine.Vector3.Dot(forwardVec, pointPosVec) < 0f;

            var distance = math.distance(position, point);
            return hasPassedThePoint ? -distance : distance;
        }


        [BurstCompile]
        private void NPCVehicleControlStep(ref NPCVehicleComponent npc, ref SystemState state)
        {
            // note: NPCVehicleControlStep is the same as in TrafficSimulator
            var deltaTime = SystemAPI.Time.DeltaTime;
            UpdateSpeed(ref npc, ref state, deltaTime);
            UpdatePose(ref npc, ref state, deltaTime);
            UpdateYawSpeed(ref npc, ref state, deltaTime);
        }

        [BurstCompile]
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
                    targetSpeed = state.EntityManager.GetComponentData<TrafficLaneComponent>(npc.currentTrafficLane).speedLimit;
                    acceleration = npc.config.acceleration;
                    break;
                case NPCVehicleSpeedMode.SLOW:
                    targetSpeed = UnityEngine.Mathf.Min(npc.config.slowSpeed, state.EntityManager.GetComponentData<TrafficLaneComponent>(npc.currentTrafficLane).speedLimit);
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

            npc.speed = UnityEngine.Mathf.MoveTowards(npc.speed, targetSpeed, acceleration * deltaTime);
        }

        [BurstCompile]
        private void UpdatePose(ref NPCVehicleComponent npc, ref SystemState state, float deltaTime)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            npc.yaw += npc.yawSpeed * deltaTime;
            var position = npc.position;
            position += Forward(ref npc) * npc.speed * deltaTime;
            position.y = npc.targetPoint.y;
            npc.position = position;
        }

        [BurstCompile]
        private float3 FrontCenterPosition(ref NPCVehicleComponent npc)
        {
            var x = UnityEngine.Quaternion.AngleAxis(npc.yaw, UnityEngine.Vector3.up) * npc.frontCenterLocalPosition;
            var x_float3 = new float3(x);
            return npc.position + x_float3;
        }


        [BurstCompile]
        private float3 Forward(ref NPCVehicleComponent npc)
        {
            var x = UnityEngine.Quaternion.AngleAxis(npc.yaw, UnityEngine.Vector3.up) * UnityEngine.Vector3.forward;
            return new float3(x);
        }

        [BurstCompile]
        private void UpdateYawSpeed(ref NPCVehicleComponent npc, ref SystemState state, float deltaTime)
        {
            // Steering the vehicle so that it heads toward the target point.
            var steeringDirection = npc.targetPoint - FrontCenterPosition(ref npc);
            steeringDirection.y = 0f;
            var steeringAngle = UnityEngine.Vector3.SignedAngle(Forward(ref npc), steeringDirection, UnityEngine.Vector3.up);
            var targetYawSpeed = steeringAngle * npc.speed * npc.config.yawSpeedMultiplier;
            // Change YawSpeed gradually to eliminate steering shake.
            npc.yawSpeed = UnityEngine.Mathf.Lerp(
                npc.yawSpeed,
                targetYawSpeed,
                npc.config.yawSpeedLerpFactor * deltaTime);
        }

        [BurstCompile]
        private void NPCVehicleVisualizationStep(ref LocalTransform localTransform, ref NPCVehicleComponent npc)
        {
            ApplyPose(ref localTransform, ref npc);
        }

        [BurstCompile]
        private void ApplyPose(ref LocalTransform localTransform, ref NPCVehicleComponent npc)
        {
            if (npc.shouldDespawn)
            {
                return;
            }

            localTransform.Position = npc.position;
            localTransform.Rotation = UnityEngine.Quaternion.AngleAxis(npc.yaw, UnityEngine.Vector3.up);
        }

        // public void SetPosition(UnityEngine.Vector3 position)
        // {
        //     rigidbody.MovePosition(new UnityEngine.Vector3(position.x, rigidbody.position.y, position.z));
        //     var velocityY = UnityEngine.Mathf.Min(rigidbody.velocity.y, maxVerticalSpeed);
        //     rigidbody.velocity = new UnityEngine.Vector3(0, velocityY, 0);
        // }


        // public void SetRotation(UnityEngine.Quaternion rotation)
        // {
        //     var inputAngles = rotation.eulerAngles;
        //     var rigidbodyAngles = rigidbody.rotation.eulerAngles;
        //     var pitch = ClampDegree360(rigidbodyAngles.x, maxSlope);
        //     var roll = ClampDegree360(rigidbodyAngles.z, maxSlope);
        //     rigidbody.MoveRotation(UnityEngine.Quaternion.Euler(pitch, inputAngles.y, roll));
        //     var angularVelocity = rigidbody.angularVelocity;
        //     rigidbody.angularVelocity = new UnityEngine.Vector3(angularVelocity.x, 0f, angularVelocity.z);
        // }

        [BurstCompile]
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
    }
}