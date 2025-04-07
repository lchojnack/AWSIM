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
        private const int MaxBoxcastCount = 5;

        private const float MinFrontVehicleDistance = 4f;
        private const float MinStopDistance = 1.5f;

        // dynamics settings const values.
        const float maxSteerAngle = 40f;                    // deg
        const float maxSteerSpeed = 60f;                    // deg/s
        const float maxVerticalSpeed = 40;                  // m/s
        const float maxSlope = 45;                          // deg


        public const float minimumDistanceToIntersection = 18f;
        public const float maximumOverrunStopPointForLaneRules = 1f;
        public const float differenceOrientationDegreesImplyingPerpendicularRoad = 35f;

        private Unity.Mathematics.Random _random;

        private NativeList<ColliderCastHit> _hitList;
        private BufferLookup<Waypoints> _waypointsLookup;
        private BufferLookup<NextLanes> _nextLanesLookup;
        private BufferLookup<NPCVehicleBoxCasts> _boxCastsLookup;
        private BufferLookup<RightOfWayLanes> _rightOfWayLanesLookup;
        private ComponentLookup<NPCVehicleComponent> _npcVehicleLookup;
        private ComponentLookup<TrafficLaneComponent> _trafficLaneLookup;
        private ComponentLookup<StopLineComponent> _stopLineLookup;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<NPCVehicleComponent>();
            state.RequireForUpdate<TrafficLaneComponent>();
            _hitList = new NativeList<ColliderCastHit>(Allocator.Persistent);
            _random = new Unity.Mathematics.Random((uint)(SystemAPI.Time.ElapsedTime * 100000) + 1);
            _waypointsLookup = state.GetBufferLookup<Waypoints>(true);
            _nextLanesLookup = state.GetBufferLookup<NextLanes>(true);
            _boxCastsLookup = state.GetBufferLookup<NPCVehicleBoxCasts>(false);
            _rightOfWayLanesLookup = state.GetBufferLookup<RightOfWayLanes>(true);
            _npcVehicleLookup = state.GetComponentLookup<NPCVehicleComponent>(true);
            _trafficLaneLookup = state.GetComponentLookup<TrafficLaneComponent>(true);
            _stopLineLookup = state.GetComponentLookup<StopLineComponent>(true);
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
            _boxCastsLookup.Update(ref state);
            _rightOfWayLanesLookup.Update(ref state);
            _npcVehicleLookup.Update(ref state);
            _trafficLaneLookup.Update(ref state);
            _stopLineLookup.Update(ref state);

            foreach (var (localTransform, npc, entity) in SystemAPI.Query<RefRW<LocalTransform>,RefRW<NPCVehicleComponent>>().WithEntityAccess())
            {
                var deltaTime = SystemAPI.Time.DeltaTime;
                NPCVehicleCognitionStep(npc, entity, ref state);
                NPCVehicleDecisionStep(npc);
                NPCVehicleControlStep(npc, deltaTime);
                NPCVehicleVisualizationStep(localTransform, npc);
            }
        }

        [BurstCompile]
        private void NPCVehicleCognitionStep(RefRW<NPCVehicleComponent> npc, Unity.Entities.Entity entity, ref SystemState state)
        {
            NextWaypointCheckJob(npc);
            ObstacleCheckJob(npc, entity);
            CalculateObstacleDistanceJob(npc, entity);
            CurveCheckJob(npc);
            RightOfWayCheckJob(npc, entity, ref state);
        }

        [BurstCompile]
        private void RightOfWayCheckJob(RefRW<NPCVehicleComponent> npc, Unity.Entities.Entity  entity, ref SystemState state)
        {
            if (npc.ValueRO.shouldDespawn)
                return;

            switch (npc.ValueRO.yieldPhase)
            {
                case NPCVehicleYieldPhase.NONE:
                    npc.ValueRW.yieldLane = Unity.Entities.Entity.Null;
                    if (isEnteringIntersection(npc))
                    {
                        npc.ValueRW.yieldPoint = GetStopPoint(FirstLaneWithIntersection(npc)); // Using centerPoint directly
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.ENTERING_INTERSECTION;
                    }
                    else if (IsOnIntersection(npc))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
                    }

                    if (npc.ValueRW.yieldPhase != NPCVehicleYieldPhase.NONE)
                        if (IsIntersectionWithYieldingLane(npc))
                            npc.ValueRW.yieldLane = FirstLaneWithIntersection(npc);
                        else
                            npc.ValueRW.yieldLane = Unity.Entities.Entity.Null;
                    break;

                case NPCVehicleYieldPhase.ENTERING_INTERSECTION:
                    if (IsOnIntersection(npc))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.AT_INTERSECTION;
                    }
                    else if (ShouldYieldDueToLanes(npc, ref state, false))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
                    }
                    else if (IsIntersectionBusy(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.INTERSECTION_BLOCKED;
                    }
                    else if (IsLeftHandRuleEnteringIntersection(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION;
                    }
                    break;

                case NPCVehicleYieldPhase.AT_INTERSECTION:
                    if (!IsOnIntersection(npc))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    else if (ShouldYieldDueToLanes(npc, ref state, true))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
                    }
                    else if (IsSomeVehicleForcingPriority(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.FORCING_PRIORITY;
                    }
                    else if (IsLeftHandRuleOnIntersection(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION;
                    }
                    break;

                case NPCVehicleYieldPhase.INTERSECTION_BLOCKED:
                    if (!IsIntersectionBusy(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                case NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION:
                    if (IsOnIntersection(npc) || !ShouldYieldDueToLanes(npc, ref state, false))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                case NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION:
                    if (!ShouldYieldDueToLanes(npc, ref state, true))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                case NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION:
                    if (IsOnIntersection(npc) || !IsLeftHandRuleEnteringIntersection(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                case NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION:
                    if (!IsLeftHandRuleOnIntersection(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                case NPCVehicleYieldPhase.FORCING_PRIORITY:
                    if (!IsSomeVehicleForcingPriority(npc, ref state))
                    {
                        npc.ValueRW.yieldPhase = NPCVehicleYieldPhase.NONE;
                    }
                    break;
                default:
                    UnityEngine.Debug.Log($"Unattended NPCVehicleYieldPhase case: {npc.ValueRO.yieldPhase}");
                    break;
            }
        }


        [BurstCompile]
        private bool ShouldYieldDueToLanes(RefRW<NPCVehicleComponent> refState, ref SystemState state, bool refOnIntersection)
        {
            if (refState.ValueRO.yieldLane == Unity.Entities.Entity.Null)
                return false;

            var isNowYielding = IsYieldingDueToRules(refState);
            var stopPoint = refOnIntersection 
                ? GetStopPoint(refState.ValueRO.yieldLane, 1)
                : GetStopPoint(refState.ValueRO.yieldLane);

            // Provide hysteresis equal to maximumOverrunStopPointForLaneRules / 2.0
            if (isNowYielding && SignedDistanceToPointOnLane(refState, refState.ValueRO.yieldPoint) < -maximumOverrunStopPointForLaneRules)
                return false;
            else if (!isNowYielding && SignedDistanceToPointOnLane(refState, stopPoint) < -maximumOverrunStopPointForLaneRules / 2.0)
                return false;

            Unity.Entities.Entity dominatingVehicle = Unity.Entities.Entity.Null;

            var rightOfWayLanes = _rightOfWayLanesLookup[refState.ValueRO.yieldLane];
            foreach (var lane in rightOfWayLanes)
            {
                if (IsLaneDominatedByAny(lane.Entity, refState, refOnIntersection, ref state, out dominatingVehicle))
                {
                    refState.ValueRW.yieldPoint = stopPoint;
                    refState.ValueRW.dominatingVehiclePosition = _npcVehicleLookup[dominatingVehicle].position;
                    return true;
                }
                else if (IsLaneDominatedByVehicle(lane.Entity, refState.ValueRO.position, Forward(refState)))
                {
                    refState.ValueRW.yieldPoint = stopPoint;
                    refState.ValueRW.dominatingVehiclePosition = refState.ValueRO.position;
                    return true;
                }
            }
            return false;
        }

        [BurstCompile]
        private bool IsLaneDominatedByVehicle(Unity.Entities.Entity lane, float3 vehiclePosition, float3 vehicleForward)
        {
            vehiclePosition.y = 0f;
            vehicleForward.y = 0f;

            var waypoints = _waypointsLookup[lane];
            for (var i = 0; i < waypoints.Length - 1; i++)
            {
                var wp0 = waypoints[i].Value;
                var wp1 = waypoints[i + 1].Value;
                wp0.y = 0f;
                wp1.y = 0f;

                if (!IsInLaneSection(wp0, wp1, vehiclePosition))
                    continue;

                if (math.degrees(math.acos(math.dot(math.normalize(wp1 - wp0), math.normalize(vehicleForward)))) < 30f)
                    return true;
            }
            return false;
        }

        [BurstCompile]
        private bool IsInLaneSection(float3 v, float3 w, float3 p, float extent = 2f)
        {
            var l2 = math.lengthsq(w - v);
            if (l2 == 0f)
                return false;

            var t = math.dot(p - v, w - v) / l2;
            if (t > 1f || t < 0f)
                return false;

            var projection = v + t * (w - v);
            return math.distance(p, projection) <= extent;
        }

        [BurstCompile]
        private bool IsLaneDominatedByAny(Unity.Entities.Entity lane, RefRW<NPCVehicleComponent> refState, bool refOnIntersection, ref SystemState state, out Unity.Entities.Entity dominatingVehicle)
        {
            dominatingVehicle = Unity.Entities.Entity.Null;

            foreach (var (otherState, otherEntity) in SystemAPI.Query<RefRW<NPCVehicleComponent>>().WithEntityAccess())
            {
                if (!ShouldBeConsideredForYielding(refState, otherState))
                    continue;

                // TODO: no traffic light information so far
                // if (IsYieldingDueToTrafficLight(otherState))
                //     continue;

                if (refOnIntersection && (IsYieldingDueToRules(otherState) || otherState.ValueRO.yieldPhase == NPCVehicleYieldPhase.INTERSECTION_BLOCKED))
                    continue;

                if (!IsLaneDominatedBy(lane, otherState))
                    continue;

                if (!IntersectOverall(refState, otherState))
                    continue;

                dominatingVehicle = otherEntity;
                return true;
            }
            return false;
        }

        [BurstCompile]
        private bool IsLaneDominatedBy(Unity.Entities.Entity lane, RefRW<NPCVehicleComponent> state)
        {
            var currentFollowingLaneEntity = CurrentFollowingLane(state);
            if (currentFollowingLaneEntity == Unity.Entities.Entity.Null)
                return false;
            var currentFollowingLane = _trafficLaneLookup[currentFollowingLaneEntity];
            return (currentFollowingLane.intersectionLane || IsNextLaneIntersection(state)) &&
                FirstLaneWithIntersection(state) == lane;
        }

        [BurstCompile]
        public Unity.Entities.Entity CurrentFollowingLane(RefRW<NPCVehicleComponent> npc)
        {
            if (!_nextLanesLookup.HasBuffer(npc.ValueRO.currentTrafficLane))
                return Unity.Entities.Entity.Null;

            var followingLanes = _nextLanesLookup[npc.ValueRO.currentTrafficLane];
            return followingLanes.Length > 0 ? followingLanes[0].Entity : Unity.Entities.Entity.Null;
        }

        [BurstCompile]
        public bool IsNextLaneIntersection(RefRW<NPCVehicleComponent> npc)
        {
            if (!_nextLanesLookup.HasBuffer(npc.ValueRO.currentTrafficLane))
                return false;

            var followingLanes = _nextLanesLookup[npc.ValueRO.currentTrafficLane];
            return followingLanes.Length > 1 && _trafficLaneLookup[followingLanes[1].Entity].intersectionLane;
        }

        [BurstCompile]
        private bool IsLeftHandRuleOnIntersection(RefRW<NPCVehicleComponent> refState, ref SystemState state)
        {
            foreach (var otherState in SystemAPI.Query<RefRW<NPCVehicleComponent>>())
            {
                if (!ShouldBeConsideredForYielding(refState, otherState))
                    continue;

                if (!IsOnIntersection(otherState))
                    continue;

                if (IsYieldingDueToRules(otherState))
                    continue;

                if (IsEnteringFromTheSameSide(refState, otherState))
                    continue;

                if (!IsVehicleOnTheLeft(refState, otherState))
                    continue;

                // If the refState should yield priority due to lane rules,
                // ignore this condition and allow the refState to proceed to LANES_RULES_**
                if (ShouldHavePriority(otherState, refState))
                    continue;

                if (IntersectNowFront(refState, otherState))
                {
                    refState.ValueRW.dominatingVehiclePosition = otherState.ValueRO.position;
                    refState.ValueRW.yieldPoint = FrontCenterPosition(refState);
                    return true;
                }
            }
            return false;
        }

        [BurstCompile]
        private bool IsYieldingDueToLanesBeforeIntersection(RefRW<NPCVehicleComponent> refState)
        {
            return refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
            refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION;
        }

        [BurstCompile]
        private bool IsVehicleOnTheLeft(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            // Get the positions of the reference vehicle and the other vehicle
            float3 refPosition = FrontCenterPosition(refState);
            refPosition.y = 0f;

            float3 positionBack = BackCenterPosition(otherState);
            positionBack.y = 0f;

            float3 positionFront = FrontCenterPosition(otherState);
            positionFront.y = 0f;

            float3 refForward = Forward(refState);

            // Calculate the cross products to determine relative positions
            float crossFront = math.cross(refForward, positionFront - refPosition).y;
            float crossBack = math.cross(refForward, positionBack - refPosition).y;

            // Return true if either the front or back of the other vehicle is on the left
            return crossFront < 0f || crossBack < 0f;
        }

        [BurstCompile]
        private bool IsLeftHandRuleEnteringIntersection(RefRW<NPCVehicleComponent> refState, ref SystemState state)
        {
            foreach (var otherState in SystemAPI.Query<RefRW<NPCVehicleComponent>>())
            {
                if (!ShouldBeConsideredForYielding(refState, otherState))
                    continue;

                // TODO no traffic light information so far
                if (IsYieldingDueToLanesBeforeIntersection(otherState) /*|| otherState.ValueRO.yieldingPriorityAtTrafficLight*/)
                    continue;

                if (IsEnteringFromTheSameSide(refState, otherState))
                    continue;

                if (!IsVehicleOnTheLeft(refState, otherState))
                    continue;

                if (IntersectOverall(refState, otherState))
                {
                    refState.ValueRW.dominatingVehiclePosition = otherState.ValueRO.position;
                    var followingLanes = _nextLanesLookup[refState.ValueRO.currentTrafficLane];
                    refState.ValueRW.yieldPoint = GetStopPoint(followingLanes[0].Entity);
                    return true;
                }
            }
            return false;
        }

        [BurstCompile]
        private bool IsSomeVehicleForcingPriority(RefRW<NPCVehicleComponent> refState, ref SystemState state)
        {
            foreach (var otherState in SystemAPI.Query<RefRW<NPCVehicleComponent>>())
            {
                if (!ShouldBeConsideredForYielding(refState, otherState))
                    continue;

                if (IsYieldingDueToRules(otherState))
                    continue;

                if (!ShouldHavePriority(refState, otherState))
                    continue;

                if (IntersectNowFront(refState, otherState))
                {
                    refState.ValueRW.yieldPoint = FrontCenterPosition(refState);
                    refState.ValueRW.dominatingVehiclePosition = otherState.ValueRO.position;
                    return true;
                }
            }
            return false;
        }

        [BurstCompile]
        private bool IntersectNowFront(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            if (!_trafficLaneLookup[refState.ValueRO.currentTrafficLane].intersectionLane || 
                !_trafficLaneLookup[otherState.ValueRO.currentTrafficLane].intersectionLane)
                return false;

            var refWaypoints = _waypointsLookup[refState.ValueRO.currentTrafficLane];
            var otherWaypoints = _waypointsLookup[otherState.ValueRO.currentTrafficLane];

            var refFirstWaypointIndex = refState.ValueRO.waypointIndex;
            var otherFirstWaypointIndex = otherState.ValueRO.waypointIndex;

            // Check intersection between all waypoints of current lanes
            for (int i = refFirstWaypointIndex; i < refWaypoints.Length - 1; i++)
            {
                for (int j = otherFirstWaypointIndex; j < otherWaypoints.Length - 1; j++)
                {
                    if (math.distance(refWaypoints[i + 1].Value, otherWaypoints[j + 1].Value) < 1f)
                        return true;

                    if (CheckIfLinesIntersect(refWaypoints[i].Value, refWaypoints[i + 1].Value, 
                                            otherWaypoints[j].Value, otherWaypoints[j + 1].Value))
                        return true;
                }
            }

            float3 refPose = FrontCenterPosition(refState);
            float3 refNearestWaypoint = refWaypoints[refFirstWaypointIndex].Value;

            // Check short section [FrontCenterPosition->CurrentWaypoint] with all sections from otherLane
            for (int j = otherFirstWaypointIndex; j < otherWaypoints.Length - 1; j++)
            {
                if (math.distance(refNearestWaypoint, otherWaypoints[j + 1].Value) < 1f)
                    return true;

                if (CheckIfLinesIntersect(refPose, refNearestWaypoint, 
                                        otherWaypoints[j].Value, otherWaypoints[j + 1].Value))
                    return true;
            }

            float3 otherPose = BackCenterPosition(otherState);
            float3 otherNearestWaypoint = otherWaypoints[otherFirstWaypointIndex].Value;

            // Check short section [otherState.ExpandedBackCenterPosition->otherState.CurrentWaypoint] with all sections from refLane
            for (int i = refFirstWaypointIndex; i < refWaypoints.Length - 1; i++)
            {
                if (math.distance(refWaypoints[i + 1].Value, otherNearestWaypoint) < 1f)
                    return true;

                if (CheckIfLinesIntersect(refWaypoints[i].Value, refWaypoints[i + 1].Value, 
                                        otherPose, otherNearestWaypoint))
                    return true;
            }

            // Check short sections [BackCenterPosition->CurrentWaypoint]x[otherState.BackCenterPosition->otherState.CurrentWaypoint]
            return CheckIfLinesIntersect(refPose, refNearestWaypoint, otherPose, otherNearestWaypoint);
        }

        [BurstCompile]
        private bool ShouldHavePriority(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            var refFirstLaneEntity = FirstLaneWithIntersection(refState);
            var otherFirstLaneEntity = FirstLaneWithIntersection(otherState);

            if (refFirstLaneEntity == Unity.Entities.Entity.Null || otherFirstLaneEntity == Unity.Entities.Entity.Null)
                return false;

            if (!_rightOfWayLanesLookup.HasBuffer(otherFirstLaneEntity))
                return false;

            var otherRightOfWayLanes = _rightOfWayLanesLookup[otherFirstLaneEntity];

            foreach (var rightOfWayLane in otherRightOfWayLanes)
            {
                if (rightOfWayLane.Entity == refFirstLaneEntity)
                {
                    return true;
                }
            }

            return false;
        }

        [BurstCompile]
        public float3 GetStopPoint(Unity.Entities.Entity trafficLaneEntity, int waypointIndex = 0)
        {
            // Ensure the entity has a TrafficLaneComponent
            if (_trafficLaneLookup.HasComponent(trafficLaneEntity))
            {
                var laneComponent = _trafficLaneLookup[trafficLaneEntity];

                // Check if stop line exists
                if (laneComponent.stopLine != Unity.Entities.Entity.Null && _stopLineLookup.HasComponent(laneComponent.stopLine))
                {
                    // Return the stop line's center point
                    return _stopLineLookup[laneComponent.stopLine].centerPoint;
                }
            }

            // If no valid stop line, access waypoints
            if (_waypointsLookup.HasBuffer(trafficLaneEntity))
            {
                var waypoints = _waypointsLookup[trafficLaneEntity];
                // Return the waypoint or a zero vector if none exist
                return waypoints.Length > 0 ? waypoints[waypointIndex].Value : float3.zero;
            }

            // Return zero if no applicable waypoints or stop lines found
            return float3.zero; 
        }

        [BurstCompile]
        private bool isEnteringIntersection(RefRW<NPCVehicleComponent> npc)
        {
            npc.ValueRW.distanceToIntersection = GetDistanceToIntersection(npc);
            npc.ValueRW.obstructedByVehicleBehindIntersection = IsObstructedByVehicleBehindIntersection(npc);
            return npc.ValueRO.distanceToIntersection <= minimumDistanceToIntersection && !npc.ValueRO.obstructedByVehicleBehindIntersection;
        }

        [BurstCompile]
        private bool IsObstructedByVehicleBehindIntersection(RefRW<NPCVehicleComponent> npc)
        {
            float distanceToIntersection = GetDistanceToIntersection(npc); 
            return distanceToIntersection > npc.ValueRO.distanceToFrontVehicle; // Direct float comparison
        }
        
        [BurstCompile]
        private Unity.Entities.Entity FirstLaneWithIntersection(RefRW<NPCVehicleComponent> npc)
        {
            // Use the _nextLanesLookup to retrieve the NextLanes buffer for the current traffic lane
            if (!_nextLanesLookup.HasBuffer(npc.ValueRO.currentTrafficLane))
                return Unity.Entities.Entity.Null; // Safety check

            // Get the following lanes buffer associated with the current lane
            var followingLanes = _nextLanesLookup[npc.ValueRO.currentTrafficLane];

            // Loop through the following lanes to find the first intersection lane
            for (int i = 0; i < followingLanes.Length; i++)
            {
                var laneEntity = followingLanes[i].Entity; // Access the lane entity

                // Check if the Layer component exists for this lane
                if (!_trafficLaneLookup.HasComponent(laneEntity))
                    continue; // If no component, skip to the next one

                var laneComponent = _trafficLaneLookup[laneEntity]; // Get the Lane component

                // Return the first lane that has intersectionLane set to true
                if (laneComponent.intersectionLane)
                {
                    return laneEntity; // Found an intersection lane
                }
            }

            return Unity.Entities.Entity.Null; // No intersection lane found
        }

        [BurstCompile]
        private bool IsOnIntersection(RefRW<NPCVehicleComponent> npc)
        {
            if (_nextLanesLookup.HasBuffer(npc.ValueRO.currentTrafficLane))
            {
                var followingLanes = _nextLanesLookup[npc.ValueRO.currentTrafficLane];

                // Ensure we have lanes and check the intersection condition
                if (followingLanes.Length > 0)
                {
                    var currentLane = npc.ValueRO.currentTrafficLane; // Get the first following lane entity.
                    // Check if it has a corresponding TrafficLaneComponent and check the intersectionLane
                    if (_trafficLaneLookup.HasComponent(currentLane))
                    {
                        var laneComponent = _trafficLaneLookup[currentLane];
                        return laneComponent.intersectionLane; // Return true if it's an intersection lane
                    }
                }
            }
            return false; // Default return if not on intersection
        }

        [BurstCompile]
        private bool IsIntersectionWithYieldingLane(RefRW<NPCVehicleComponent> npc)
        {
            Unity.Entities.Entity intersectionLaneEntity = FirstLaneWithIntersection(npc); // Call a method to get the lane

            // Check if the lane has a right-of-way buffer
            if (intersectionLaneEntity != Unity.Entities.Entity.Null && _rightOfWayLanesLookup.HasBuffer(intersectionLaneEntity))
            {
                var rightOfWayLanes = _rightOfWayLanesLookup[intersectionLaneEntity];

                return rightOfWayLanes.Length > 0; // ✅ Check if there's a right-of-way lane available
            }

            return false; // No right-of-way lane
        }

        [BurstCompile]
        private float GetDistanceToIntersection(RefRW<NPCVehicleComponent> npc)
        {
            var firstLaneWithIntersection = FirstLaneWithIntersection(npc);
            if (firstLaneWithIntersection == Unity.Entities.Entity.Null)
                return float.MaxValue;

            var trafficLane = _trafficLaneLookup[firstLaneWithIntersection];
            var waypoints = _waypointsLookup[firstLaneWithIntersection];
            var stopLineCenterPoint = _stopLineLookup.HasComponent(firstLaneWithIntersection) 
                ? _stopLineLookup[firstLaneWithIntersection].centerPoint 
                : waypoints[0].Value;

            return SignedDistanceToPointOnLane(npc, stopLineCenterPoint);
        }

        [BurstCompile]
        private bool IsIntersectionBusy(RefRW<NPCVehicleComponent> npc, ref SystemState state)
        {
            foreach (var otherNpc in SystemAPI.Query<RefRW<NPCVehicleComponent>>())
            {
                // Check conditions
                if (!ShouldBeConsideredForYielding(npc, otherNpc))
                    continue;

                // Check if intersection is busy
                if (!IsOnIntersection(otherNpc))
                    continue;

                // Check yielding conditions
                if (IsYieldingDueToRules(otherNpc) && IsOnPerpendicularRoad(npc, otherNpc))
                    continue;

                if (IsEnteringFromTheSameSide(npc, otherNpc))
                    continue;

                if (IntersectOverall(npc, otherNpc))
                {
                    npc.ValueRW.dominatingVehiclePosition = otherNpc.ValueRO.position;
                    npc.ValueRW.yieldPoint = GetStopPoint(FirstLaneWithIntersection(npc));
                    return true; // Intersection is busy
                }
            }
            return false; // No intersections busy
        }

        [BurstCompile]
        private bool IntersectOverall(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            float3? refStartNullable = FirstIntersectionWaypoint(refState);
            float3? otherStartNullable = FirstIntersectionWaypoint(otherState);
            if (!refStartNullable.HasValue || !otherStartNullable.HasValue)
                return false;

            float3? refGoalNullable = LastIntersectionWaypoint(refState);
            float3? otherGoalNullable = LastIntersectionWaypoint(otherState);
            if (!refGoalNullable.HasValue || !otherGoalNullable.HasValue)
                return false;

            if (!CheckIfLinesIntersect(refStartNullable.Value, refGoalNullable.Value, otherStartNullable.Value, otherGoalNullable.Value))
                return false;

            var refFirstWaypointIndex = refState.ValueRO.waypointIndex;
            var otherFirstWaypointIndex = otherState.ValueRO.waypointIndex;

            if (!_waypointsLookup.HasBuffer(refState.ValueRO.currentTrafficLane) || !_waypointsLookup.HasBuffer(otherState.ValueRO.currentTrafficLane))
                return false;

            var refWaypoints = _waypointsLookup[refState.ValueRO.currentTrafficLane];
            var otherWaypoints = _waypointsLookup[otherState.ValueRO.currentTrafficLane];

            // Check intersection between all waypoints of these lanes
            for (int i = refFirstWaypointIndex; i < refWaypoints.Length - 1; i++)
            {
                for (int j = otherFirstWaypointIndex; j < otherWaypoints.Length - 1; j++)
                {
                    if (math.distance(refWaypoints[i + 1].Value, otherWaypoints[j + 1].Value) < 1f)
                        return true;

                    if (CheckIfLinesIntersect(refWaypoints[i].Value, refWaypoints[i + 1].Value, otherWaypoints[j].Value, otherWaypoints[j + 1].Value))
                        return true;
                }
            }

            // Check short section [BackCenterPosition->CurrentWaypoint] with all sections from otherWaypoints
            for (int j = otherFirstWaypointIndex; j < otherWaypoints.Length - 1; j++)
            {
                if (CheckIfLinesIntersect(BackCenterPosition(refState), refWaypoints[refFirstWaypointIndex].Value, 
                    otherWaypoints[j].Value, otherWaypoints[j + 1].Value))
                    return true;
            }

            // Check short section [otherState.BackCenterPosition->otherState.CurrentWaypoint] with all sections from refWaypoints
            for (int i = refFirstWaypointIndex; i < refWaypoints.Length - 1; i++)
            {
                if (CheckIfLinesIntersect(refWaypoints[i].Value, refWaypoints[i + 1].Value, 
                    BackCenterPosition(otherState), otherWaypoints[otherFirstWaypointIndex].Value))
                    return true;
            }

            // Check short sections [BackCenterPosition->CurrentWaypoint]x[otherState.BackCenterPosition->otherState.CurrentWaypoint]
            return CheckIfLinesIntersect(BackCenterPosition(refState), refWaypoints[refFirstWaypointIndex].Value, 
                BackCenterPosition(otherState), otherWaypoints[otherFirstWaypointIndex].Value);
        }

        [BurstCompile]
        private bool CheckIfLinesIntersect(float3 A1, float3 B1, float3 A2, float3 B2)
        {
            float2 line1point1 = new float2(A1.x, A1.z);
            float2 line1point2 = new float2(B1.x, B1.z);
            float2 line2point1 = new float2(A2.x, A2.z);
            float2 line2point2 = new float2(B2.x, B2.z);

            float2 a = line1point2 - line1point1;
            float2 b = line2point1 - line2point2;
            float2 c = line1point1 - line2point1;

            float alphaNumerator = b.y * c.x - b.x * c.y;
            float betaNumerator = a.x * c.y - a.y * c.x;
            float denominator = a.y * b.x - a.x * b.y;

            if (denominator == 0)
                return false;

            if (denominator > 0)
            {
                if (alphaNumerator < 0 || alphaNumerator > denominator || betaNumerator < 0 || betaNumerator > denominator)
                    return false;
            }
            else if (alphaNumerator > 0 || alphaNumerator < denominator || betaNumerator > 0 || betaNumerator < denominator)
            {
                return false;
            }
            return true;
        }

        [BurstCompile]
        private bool IsEnteringFromTheSameSide(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            // Check if either state has a null FirstLaneWithIntersection
            if (FirstLaneWithIntersection(otherState) == Unity.Entities.Entity.Null || FirstLaneWithIntersection(refState) == Unity.Entities.Entity.Null)
                return false;

            // Calculate the distance using math operations
            float3 refWaypoint = FirstIntersectionWaypoint(refState).Value;
            float3 otherWaypoint = FirstIntersectionWaypoint(otherState).Value; 

            return math.distance(refWaypoint, otherWaypoint) < 3f;
        }

        [BurstCompile]
        private float3? FirstIntersectionWaypoint(RefRW<NPCVehicleComponent> npc)
        {
            // Retrieve the first lane with an intersection
            var firstLaneEntity = FirstLaneWithIntersection(npc);

            // Check if the lane entity is valid and has waypoints
            if (firstLaneEntity == Unity.Entities.Entity.Null || !_waypointsLookup.HasBuffer(firstLaneEntity))
            {
                return null; // No valid intersection waypoint
            }

            // Retrieve the waypoints buffer for the lane
            var waypoints = _waypointsLookup[firstLaneEntity];

            // Return the first waypoint if it exists, otherwise return null
            return waypoints.Length > 0 ? (float3?)waypoints[0].Value : null;
        }

        [BurstCompile]
        private float3? LastIntersectionWaypoint(RefRW<NPCVehicleComponent> npc)
        {
            // Retrieve the first lane with an intersection
            var firstLaneEntity = FirstLaneWithIntersection(npc);

            // Check if the lane entity is valid and has waypoints
            if (firstLaneEntity == Unity.Entities.Entity.Null || !_waypointsLookup.HasBuffer(firstLaneEntity))
            {
                return null; // No valid intersection waypoint
            }

            // Retrieve the waypoints buffer for the lane
            var waypoints = _waypointsLookup[firstLaneEntity];

            // Return the last waypoint if it exists, otherwise return null
            return waypoints.Length > 0 ? (float3?)waypoints[waypoints.Length - 1].Value : null;
        }

        [BurstCompile]
        private bool IsYieldingDueToRules(RefRW<NPCVehicleComponent> refState)
        {
            return refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_ENTERING_INTERSECTION ||
            refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LEFT_HAND_RULE_AT_INTERSECTION ||
            refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LANES_RULES_ENTERING_INTERSECTION ||
            refState.ValueRO.yieldPhase == NPCVehicleYieldPhase.LANES_RULES_AT_INTERSECTION;
        }

        [BurstCompile]
        private bool ShouldBeConsideredForYielding(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            return /*refState.ValueRO.Vehicle.VehicleID != otherState.ValueRO.Vehicle.VehicleID */
                /*&&*/ IsEnteringIntersection(otherState);
        }

        [BurstCompile]
        private bool IsEnteringIntersection(RefRW<NPCVehicleComponent> npc)
        {
            // Use the class member minimumDistanceToIntersection directly
            npc.ValueRW.distanceToIntersection = GetDistanceToIntersection(npc);
            return npc.ValueRO.distanceToIntersection <= minimumDistanceToIntersection && 
                !IsObstructedByVehicleBehindIntersection(npc);  // Assuming this function checks for obstructions
        }

        // Additional helper function for perpendicular road checking
        [BurstCompile]
        private bool IsOnPerpendicularRoad(RefRW<NPCVehicleComponent> refState, RefRW<NPCVehicleComponent> otherState)
        {
            float diffAngleDegrees = math.abs(refState.ValueRO.yaw - otherState.ValueRO.yaw); // Using math
            return (diffAngleDegrees < differenceOrientationDegreesImplyingPerpendicularRoad ||
                    diffAngleDegrees > 180f - differenceOrientationDegreesImplyingPerpendicularRoad);
        }

        [BurstCompile]
        private void ObstacleCheckJob(RefRW<NPCVehicleComponent> npc, Unity.Entities.Entity entity)
        {
            var waypoints = _waypointsLookup[npc.ValueRO.currentTrafficLane];

            float3 previousPoint = npc.ValueRO.waypointIndex == 1
                ? FrontCenterPosition(npc)
                : waypoints[npc.ValueRO.waypointIndex - 1].Value;
            npc.ValueRW.startPoint = previousPoint;

            // Reduce the detection range so that large sized vehicles can pass each other.
            float3 boxCastExtents = npc.ValueRO.extents * 0.5f;
            // boxCastExtents.y *= 1;
            boxCastExtents.z = 0.1f;
            // float3 endPoint = waypoints[npc.ValueRO.waypointIndex].Value;

            // float distance = math.distance(npc.ValueRO.startPoint, endPoint);
            // float3 direction = (endPoint - npc.ValueRO.startPoint);
            // quaternion orientation = quaternion.LookRotationSafe(direction, math.up());

            // ECS Boxcast -> NPC <-> NPC
            var physicsWorld = SystemAPI.GetSingleton<PhysicsWorldSingleton>();
            _hitList.Clear();

            Unity.Physics.CollisionFilter filter = new CollisionFilter{
                BelongsTo = (uint)npc.ValueRO.config.vehicleLayerMask.value,
                CollidesWith = (uint)npc.ValueRO.config.vehicleLayerMask.value
            };

            int maxSegments = math.min(MaxBoxcastCount, waypoints.Length - npc.ValueRO.waypointIndex);
            var boxcastBuffer = _boxCastsLookup[entity];
            boxcastBuffer.Clear();

            for (int i = 0; i < maxSegments; i++)
            {
                float3 nextWaypoint = waypoints[npc.ValueRO.waypointIndex + i].Value;
                float distance = math.distance(previousPoint, nextWaypoint);
                float3 direction = (nextWaypoint - previousPoint);
                quaternion orientation = quaternion.LookRotationSafe(direction, math.up());

                float raycastHitDistance = float.MaxValue;
                float3 raycastHitPoint = float3.zero;
                bool raycastHit = false;
                bool isCollision = false;

                // Boxcast command -> NPC <-> EGO
                // if(npc.ValueRW.config.debugMode)
                // {
                //     var obstacleHitInfoArray = new NativeArray<UnityEngine.RaycastHit>(1, Allocator.TempJob);
                //     var boxcastCommands = new NativeArray<UnityEngine.BoxcastCommand>(1, Allocator.TempJob);
                //     boxcastCommands[0] = new UnityEngine.BoxcastCommand(
                //         previousPoint,
                //         boxCastExtents,
                //         UnityEngine.Quaternion.LookRotation(direction),
                //         direction,
                //         distance,
                //         npc.ValueRW.config.vehicleLayerMask
                //     );

                //     JobHandle boxcastJobHandle = UnityEngine.BoxcastCommand.ScheduleBatch(boxcastCommands, obstacleHitInfoArray, 1);
                //     boxcastJobHandle.Complete();

                //     isCollision = obstacleHitInfoArray[0].collider != null;
                //     if (isCollision)
                //     {
                //         raycastHitPoint = obstacleHitInfoArray[0].point;
                //         raycastHitDistance = math.distance(previousPoint, obstacleHitInfoArray[0].point);
                //         raycastHit = true;
                //     }

                //     obstacleHitInfoArray.Dispose();
                //     boxcastCommands.Dispose();

                //     boxcastBuffer.Add(new NPCVehicleBoxCasts
                //     {
                //         startPoint = previousPoint,
                //         direction = direction,
                //         distance = distance,
                //         extents = boxCastExtents,
                //         raycastHitPoint = raycastHitPoint,
                //         raycastHitDistance = raycastHitDistance,
                //         raycastHit = raycastHit
                //     });
                // }
                // else
                // {
                    isCollision =  physicsWorld.BoxCastAll(
                        previousPoint,
                        orientation,
                        boxCastExtents,
                        direction,
                        distance,
                        ref _hitList,
                        filter
                    );
                    
                    if(isCollision)
                    {
                        var shortestDistance = raycastHitDistance;
                        float3 closestHitPoint = raycastHitPoint;
                        bool isHit = false;
                        for (int j = 0; j < _hitList.Length; j++)
                        {
                            var hitDistance = math.distance(previousPoint, _hitList[j].Position);
                            isHit = npc.ValueRO.meshColliderComponent != _hitList[j].Entity && shortestDistance > hitDistance && hitDistance < distance;
                            if (isHit)
                            {
                                shortestDistance = hitDistance;
                                closestHitPoint = _hitList[j].Position;
                                raycastHit = isHit;
                            }
                        }
                        raycastHitPoint = closestHitPoint;
                        raycastHitDistance = shortestDistance;
                    }

                    boxcastBuffer.Add(new NPCVehicleBoxCasts
                    {
                        startPoint = previousPoint,
                        direction = direction,
                        distance = distance,
                        extents = boxCastExtents,
                        raycastHitPoint = raycastHitPoint,
                        raycastHitDistance = raycastHitDistance,
                        raycastHit = raycastHit
                    });

                    if(raycastHit)
                        break;
                // }
                previousPoint = nextWaypoint;
            }
        }

        [BurstCompile]
        private void CalculateObstacleDistanceJob(RefRW<NPCVehicleComponent> npc, Unity.Entities.Entity  entity)
        {
            var boxCastBuffer = _boxCastsLookup[entity];
            var hasHit = false;
            var totalDistance = 0f;

            for (var i = 0; i < boxCastBuffer.Length; i++)
            {
                hasHit = boxCastBuffer[i].raycastHit;
                if (hasHit)
                {
                    totalDistance += boxCastBuffer[i].raycastHitDistance;
                    // break;
                }
                else
                {
                    totalDistance += boxCastBuffer[i].distance;
                }
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
        private float3 BackCenterPosition(RefRW<NPCVehicleComponent> npc)
        {
            // Calculate the raw back center position
            float3 backCenterPositionRaw = npc.ValueRO.backCenterLocalPosition;

            // Calculate the position using the vehicle's yaw
            quaternion yawRotation = quaternion.AxisAngle(math.up(), math.radians(npc.ValueRO.yaw));
            return npc.ValueRO.position + math.mul(yawRotation, backCenterPositionRaw);
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