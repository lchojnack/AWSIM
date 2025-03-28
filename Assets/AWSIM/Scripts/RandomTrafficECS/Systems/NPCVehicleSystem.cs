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

        private Unity.Mathematics.Random _random;

        private NativeList<ColliderCastHit> _hitList;
        private BufferLookup<Waypoints> _waypointsLookup;
        private BufferLookup<NextLanes> _nextLanesLookup;
        private ComponentLookup<TrafficLaneComponent> _trafficLaneLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NPCVehicleComponent>();
            state.RequireForUpdate<TrafficLaneComponent>();
            _hitList = new NativeList<ColliderCastHit>(Allocator.Persistent);
            _random = new Unity.Mathematics.Random((uint)(SystemAPI.Time.ElapsedTime * 100000) + 1);
            _waypointsLookup = state.GetBufferLookup<Waypoints>(true);
            _nextLanesLookup = state.GetBufferLookup<NextLanes>(true);
            _trafficLaneLookup = state.GetComponentLookup<TrafficLaneComponent>(true);
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
            _hitList.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            _waypointsLookup.Update(ref state);
            _nextLanesLookup.Update(ref state);
            _trafficLaneLookup.Update(ref state);

            foreach (var (localTransform, npc) in SystemAPI.Query<RefRW<LocalTransform>,RefRW<NPCVehicleComponent>>())
            {
                var deltaTime = SystemAPI.Time.DeltaTime;
                NPCVehicleCognitionStep(npc);
                NPCVehicleDecisionStep(npc);
                NPCVehicleControlStep(npc, deltaTime);
                NPCVehicleVisualizationStep(localTransform, npc);
            }
        }

        [BurstCompile]
        private void NPCVehicleCognitionStep(RefRW<NPCVehicleComponent> npc)
        {
            NextWaypointCheckJob(npc);
            CurveCheckJob(npc);
            ObstacleCheckJob(npc);
            CalculateObstacleDistanceJob(npc);
        }

        [BurstCompile]
        private void ObstacleCheckJob(RefRW<NPCVehicleComponent> npc)
        {
            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];

            npc.ValueRW.startPoint = npc.ValueRO.waypointIndex == 1
                ? FrontCenterPosition(npc)
                : waypoints[npc.ValueRO.waypointIndex - 1].Value;

            // Reduce the detection range so that large sized vehicles can pass each other.
            float3 boxCastExtents = npc.ValueRO.extents * 0.5f;
            boxCastExtents.y *= 1;
            boxCastExtents.z = 0.1f;
            float3 endPoint = waypoints[npc.ValueRO.waypointIndex].Value;

            float distance = math.distance(npc.ValueRO.startPoint, endPoint);
            float3 direction = (endPoint - npc.ValueRO.startPoint);
            quaternion orientation = quaternion.LookRotationSafe(direction, math.up());

            npc.ValueRW.raycastHitDistance = float.MaxValue;
            npc.ValueRW.raycastHitPoint = float3.zero;

            npc.ValueRW.boxcastCommandStartPoint = npc.ValueRO.startPoint;
            npc.ValueRW.boxcastCommandDirection = direction;
            npc.ValueRW.boxcastCommandDistance = distance;
            npc.ValueRW.boxcastCommandExtents = boxCastExtents;

            // Boxcast command -> NPC <-> EGO
            // if(npc.ValueRW.config.debugMode)
            // {
            //     var obstacleHitInfoArray = new NativeArray<UnityEngine.RaycastHit>(MaxBoxcastCount, Allocator.TempJob);
            //     var boxcastCommands = new NativeArray<UnityEngine.BoxcastCommand>(MaxBoxcastCount, Allocator.TempJob);
            //     boxcastCommands[0] = new UnityEngine.BoxcastCommand(
            //         npc.ValueRW.startPoint,
            //         boxCastExtents,
            //         UnityEngine.Quaternion.LookRotation(direction),
            //         direction,
            //         distance,
            //         npc.ValueRW.config.vehicleLayerMask
            //     );

            //     JobHandle boxcastJobHandle = UnityEngine.BoxcastCommand.ScheduleBatch(boxcastCommands, obstacleHitInfoArray, 1);
            //     boxcastJobHandle.Complete();

            //     if (obstacleHitInfoArray[0].collider != null)
            //     {
            //         npc.ValueRW.raycastHitPoint = obstacleHitInfoArray[0].point;
            //         npc.ValueRW.raycastHitDistance = obstacleHitInfoArray[0].distance;
            //     }


            //     obstacleHitInfoArray.Dispose();
            //     boxcastCommands.Dispose();
            // }

            // ECS Boxcast -> NPC <-> NPC
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
            _hitList.Clear();

            Unity.Physics.CollisionFilter filter = new CollisionFilter{
                BelongsTo = (uint)npc.ValueRO.config.vehicleLayerMask.value,
                CollidesWith = (uint)npc.ValueRO.config.vehicleLayerMask.value
            };
            bool isCollision =  physicsWorld.BoxCastAll(
                npc.ValueRO.startPoint,
                orientation,
                boxCastExtents,
                direction,
                distance,
                ref _hitList,
                filter
            );

            if(isCollision)
            {
                var shortestDistance = float.MaxValue;
                float3 closestHitPoint = npc.ValueRO.raycastHitPoint;
                foreach (var hit in _hitList)
                {
                    var hitDistance = math.distance(npc.ValueRO.startPoint, hit.Position);
                    if(npc.ValueRO.meshColliderComponent != hit.Entity && shortestDistance > hitDistance)
                    {
                        shortestDistance = hitDistance;
                        closestHitPoint = hit.Position;
                    }
                }

                npc.ValueRW.raycastHitPoint = closestHitPoint;
                npc.ValueRW.raycastHitDistance = shortestDistance;
            }
        }

        [BurstCompile]
        private void CalculateObstacleDistanceJob(RefRW<NPCVehicleComponent> npc)
        {
            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];

            // CalculateObstacleDistanceJob starts here
            var hasHit = false;
            var totalDistance = 0f;
            var boxcastCount = math.min(MaxBoxcastCount, waypoints.Length);

            for (var commandIndex = 0; commandIndex < boxcastCount; commandIndex++)
            {
                hasHit = npc.ValueRO.raycastHitDistance != float.MaxValue || math.all(npc.ValueRO.raycastHitPoint != float3.zero);
                if (hasHit)
                {
                    totalDistance = math.distance(npc.ValueRO.startPoint, npc.ValueRO.raycastHitPoint);
                    break;
                }
                totalDistance = npc.ValueRO.boxcastCommandDistance;
            }

            npc.ValueRW.distanceToFrontVehicle = hasHit ? totalDistance : float.MaxValue;
        }

        [BurstCompile]
        private void NextWaypointCheckJob(RefRW<NPCVehicleComponent> npc)
        {
            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];
            var distanceToCurrentWaypoint = GeometryUtility.Distance2D(waypoints[npc.ValueRO.waypointIndex].Value, FrontCenterPosition(npc));
            npc.ValueRW.distanceToCurrentWaypoint = distanceToCurrentWaypoint;
            var isCloseToTarget = distanceToCurrentWaypoint <= npc.ValueRO.frontCenterLocalPosition.z;

            if(!isCloseToTarget)
            {
                return;
            }

            if (npc.ValueRO.waypointIndex >= (waypoints.Length-1))
            {
                // equivalent to extend following lanes
                var nextLanes = _nextLanesLookup[npc.ValueRO.currentTrafficLane];
                if(nextLanes.Length != 0)
                {
                    var randomIndex = _random.NextInt(0, nextLanes.Length);
                    if(npc.ValueRO.config.debugMode)
                    {
                        randomIndex = 0;
                    }
                    npc.ValueRW.currentTrafficLane = nextLanes[randomIndex].Entity;
                    npc.ValueRW.waypointIndex = 1;
                }
            }       
            else
            {
                npc.ValueRW.waypointIndex += 1;
            }

            waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];
            distanceToCurrentWaypoint = GeometryUtility.Distance2D(waypoints[waypoints.Length-1].Value, FrontCenterPosition(npc));
            isCloseToTarget = distanceToCurrentWaypoint <= 2.0f;
            if(_nextLanesLookup[npc.ValueRO.currentTrafficLane].Length == 0 && isCloseToTarget)
            {
                npc.ValueRW.shouldDespawn = true;
            }
        }

        [BurstCompile]
        private void CurveCheckJob(RefRW<NPCVehicleComponent> npc)
        {
            if (npc.ValueRO.shouldDespawn)
            {
                return;
            }

            float3 currentForward = math.mul(quaternion.AxisAngle(math.up(), math.radians(npc.ValueRO.yaw)), math.forward());
            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];
            var currentWaypointIndex = npc.ValueRO.waypointIndex;
            var elapsedDistance = math.distance(FrontCenterPosition(npc), waypoints[currentWaypointIndex].Value);
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
                turnAngle += AngleBetweenVectors(currentForward, nextForward);
                currentForward = nextForward;
            }

            npc.ValueRW.isTurning = turnAngle > 45f;
        }

        [BurstCompile]
        private static float AngleBetweenVectors(in float3 a, in float3 b)
        {
            float dotProduct = math.dot(math.normalizesafe(a), math.normalizesafe(b));
            return math.degrees(math.acos(math.clamp(dotProduct, -1f, 1f))); // ✅ Computes Angle in Burst!
        }

        [BurstCompile]
        private void NPCVehicleDecisionStep(RefRW<NPCVehicleComponent> npc)
        {
            // note: NPCVehicleDecitionStep is the same as in TrafficSimulator
            UpdateTargetPoint(npc);
            UpdateSpeedMode(npc);
        }

        [BurstCompile]
        private void UpdateTargetPoint(RefRW<NPCVehicleComponent> npc)
        {
            if (npc.ValueRO.shouldDespawn || npc.ValueRO.currentTrafficLane == null)
            {
                return;
            }

            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];
            npc.ValueRW.targetPoint = waypoints[npc.ValueRO.waypointIndex].Value;
        }

        [BurstCompile]
        private void UpdateSpeedMode(RefRW<NPCVehicleComponent> npc)
        {
            if (npc.ValueRO.shouldDespawn)
            {
                return;
            }

            var absoluteStopDistance = CalculateStoppableDistance(npc.ValueRO.speed, npc.ValueRO.config.absoluteDeceleration) + MinStopDistance;
            var suddenStopDistance = CalculateStoppableDistance(npc.ValueRO.speed, npc.ValueRO.config.suddenDeceleration) + 2 * MinStopDistance;
            var stopDistance = CalculateStoppableDistance(npc.ValueRO.speed, npc.ValueRO.config.deceleration) + 3 * MinStopDistance;
            var slowDownDistance = stopDistance + 4 * MinStopDistance;

            var distanceToStopPointByFrontVehicle = onlyGreaterThan(npc.ValueRO.distanceToFrontVehicle - MinFrontVehicleDistance, -MinFrontVehicleDistance);
            var distanceToStopPointByTrafficLight = CalculateTrafficLightDistance(npc, suddenStopDistance);
            var distanceToStopPointByRightOfWay = CalculateYieldingDistance(npc);
            var distanceToStopPoint = math.min(distanceToStopPointByFrontVehicle, distanceToStopPointByTrafficLight);
            distanceToStopPoint = math.min(distanceToStopPoint, distanceToStopPointByRightOfWay);

            npc.ValueRW.isStoppedByFrontVehicle = false;
            if (distanceToStopPointByFrontVehicle <= stopDistance)
            {
                npc.ValueRW.isStoppedByFrontVehicle = true;
            }

            if (distanceToStopPoint <= absoluteStopDistance)
                npc.ValueRW.speedMode = NPCVehicleSpeedMode.ABSOLUTE_STOP;
            else if (distanceToStopPoint <= suddenStopDistance)
                npc.ValueRW.speedMode = NPCVehicleSpeedMode.SUDDEN_STOP;
            else if (distanceToStopPoint <= stopDistance)
                npc.ValueRW.speedMode = NPCVehicleSpeedMode.STOP;
            else if (distanceToStopPoint <= slowDownDistance || npc.ValueRO.isTurning)
                npc.ValueRW.speedMode = NPCVehicleSpeedMode.SLOW;
            else
                npc.ValueRW.speedMode = NPCVehicleSpeedMode.NORMAL;
        }

        [BurstCompile]
        private float CalculateYieldingDistance(RefRW<NPCVehicleComponent> npc)
        {
            // TODO no yielding information so far
            var distanceToStopPointByRightOfWay = float.MaxValue;
            if (npc.ValueRO.yieldPhase != NPCVehicleYieldPhase.NONE && npc.ValueRO.yieldPhase != NPCVehicleYieldPhase.ENTERING_INTERSECTION && npc.ValueRO.yieldPhase != NPCVehicleYieldPhase.AT_INTERSECTION)
            {
                distanceToStopPointByRightOfWay = SignedDistanceToPointOnLane(npc, npc.ValueRO.yieldPoint);
            }
            return onlyGreaterThan(distanceToStopPointByRightOfWay, 0);
        }

        [BurstCompile]
        private float CalculateTrafficLightDistance(RefRW<NPCVehicleComponent> npc, float suddenStopDistance)
        {
            // TODO no traffic light information so far
            var distanceToStopPointByTrafficLight = float.MaxValue;
            // if (npc.ValueRO.TrafficLightLane != null)
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
        public float SignedDistanceToPointOnLane(RefRW<NPCVehicleComponent> npc, float3 point)
        {
            var position = FrontCenterPosition(npc);
            position.y = 0f;
            point.y = 0f;

            var forward = Forward(npc);
            var pointPos = point - position;

            var hasPassedThePoint = math.dot(forward, pointPos) < 0f;

            var distance = math.distance(position, point);
            return hasPassedThePoint ? -distance : distance;
        }


        [BurstCompile]
        private void NPCVehicleControlStep(RefRW<NPCVehicleComponent> npc, float deltaTime)
        {
            // note: NPCVehicleControlStep is the same as in TrafficSimulator
            UpdateSpeed(npc, deltaTime);
            UpdatePose(npc, deltaTime);
            UpdateYawSpeed(npc, deltaTime);
        }

        [BurstCompile]
        private void UpdateSpeed(RefRW<NPCVehicleComponent> npc, float deltaTime)
        {
            if (npc.ValueRO.shouldDespawn)
            {
                return;
            }

            float targetSpeed;
            float acceleration;
            var laneComponent = _trafficLaneLookup[npc.ValueRO.currentTrafficLane];
            switch (npc.ValueRO.speedMode)
            {
                case NPCVehicleSpeedMode.NORMAL:
                    targetSpeed = laneComponent.speedLimit;
                    acceleration = npc.ValueRO.config.acceleration;
                    break;
                case NPCVehicleSpeedMode.SLOW:
                    targetSpeed = math.min(npc.ValueRO.config.slowSpeed, laneComponent.speedLimit);
                    acceleration = npc.ValueRO.config.deceleration;
                    break;
                case NPCVehicleSpeedMode.SUDDEN_STOP:
                    targetSpeed = 0f;
                    acceleration = npc.ValueRO.config.suddenDeceleration;
                    break;
                case NPCVehicleSpeedMode.ABSOLUTE_STOP:
                    targetSpeed = 0f;
                    acceleration = npc.ValueRO.config.absoluteDeceleration;
                    break;
                case NPCVehicleSpeedMode.STOP:
                    targetSpeed = 0f;
                    acceleration = npc.ValueRO.config.deceleration;
                    break;
                default:
                    targetSpeed = 0f;
                    acceleration = npc.ValueRO.config.deceleration;
                    break;
            }

            npc.ValueRW.speed = MoveTowards(npc.ValueRO.speed, targetSpeed, acceleration * deltaTime);
        }

        [BurstCompile]
        private static float MoveTowards(float current, float target, float maxDelta)
        {
            return current + math.clamp(target - current, -maxDelta, maxDelta);
        }

        [BurstCompile]
        private void UpdatePose(RefRW<NPCVehicleComponent> npc, float deltaTime)
        {
            if (npc.ValueRO.shouldDespawn)
            {
                return;
            }

            npc.ValueRW.yaw += npc.ValueRO.yawSpeed * deltaTime;
            var position = npc.ValueRO.position;
            position += Forward(npc) * npc.ValueRO.speed * deltaTime;
            position.y = npc.ValueRO.targetPoint.y;
            npc.ValueRW.position = position;
        }

        [BurstCompile]
        private float3 FrontCenterPosition(RefRW<NPCVehicleComponent> npc)
        {
            quaternion yawRotation = quaternion.AxisAngle(math.up(), math.radians(npc.ValueRO.yaw));
            var x_float3 =  math.mul(yawRotation, npc.ValueRO.frontCenterLocalPosition);
            return npc.ValueRO.position + x_float3;
        }


        [BurstCompile]
        private float3 Forward(RefRW<NPCVehicleComponent> npc)
        {
            quaternion yawRotation = quaternion.AxisAngle(math.up(), math.radians(npc.ValueRO.yaw));
            return math.mul(yawRotation, math.forward());
        }

        [BurstCompile]
        private void UpdateYawSpeed(RefRW<NPCVehicleComponent> npc, float deltaTime)
        {
            // Steering the vehicle so that it heads toward the target point.
            var steeringDirection = npc.ValueRO.targetPoint - FrontCenterPosition(npc);
            steeringDirection.y = 0f;
            var steeringAngle = SignedAngleBetweenVectors(Forward(npc), steeringDirection, math.up());
            var targetYawSpeed = steeringAngle * npc.ValueRO.speed * npc.ValueRO.config.yawSpeedMultiplier;
            // Change YawSpeed gradually to eliminate steering shake.
            npc.ValueRW.yawSpeed = math.lerp(
                npc.ValueRO.yawSpeed,
                targetYawSpeed,
                npc.ValueRO.config.yawSpeedLerpFactor * deltaTime);
        }

        [BurstCompile]
        private static float SignedAngleBetweenVectors(in float3 from, in float3 to, in float3 axis)
        {
            float3 fromDir = math.normalizesafe(from);
            float3 toDir = math.normalizesafe(to);

            float dot = math.dot(fromDir, toDir);
            float angle = math.degrees(math.acos(math.clamp(dot, -1f, 1f)));

            float3 cross = math.cross(fromDir, toDir);
            float sign = math.sign(math.dot(cross, math.normalizesafe(axis)));

            return angle * sign; // ✅ Burst-compatible signed angle
        }

        [BurstCompile]
        private void NPCVehicleVisualizationStep(RefRW<LocalTransform> localTransform, RefRW<NPCVehicleComponent> npc)
        {
            ApplyPose(localTransform, npc);
        }

        [BurstCompile]
        private void ApplyPose(RefRW<LocalTransform> localTransform, RefRW<NPCVehicleComponent> npc)
        {
            if (npc.ValueRO.shouldDespawn)
            {
                return;
            }

            localTransform.ValueRW.Position = npc.ValueRO.position;
            localTransform.ValueRW.Rotation = quaternion.AxisAngle(math.up(), math.radians(npc.ValueRO.yaw));
        }

        // public void SetPosition(UnityEngine.Vector3 position)
        // {
        //     rigidbody.MovePosition(new UnityEngine.Vector3(position.x, rigidbody.position.y, position.z));
        //     var velocityY = math.min(rigidbody.velocity.y, maxVerticalSpeed);
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