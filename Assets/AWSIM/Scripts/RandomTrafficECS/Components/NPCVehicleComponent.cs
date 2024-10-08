using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace AWSIM.TrafficSimulationECS
{
    public struct NPCVehicleConfigComponent : IComponentData
    {
        public float acceleration;
        public float deceleration;
        public float suddenDeceleration;
        public float absoluteDeceleration;
        public float yawSpeedMultiplier;
        public float yawSpeedLerpFactor;
        public float slowSpeed;
    }

    public enum NPCVehicleSpeedMode
    {
        NORMAL = 1,
        SLOW = 2 ,
        STOP = 3,
        SUDDEN_STOP = 4,
        ABSOLUTE_STOP =5
    }

    public enum NPCVehicleYieldPhase
    {
        NONE = 1,
        ENTERING_INTERSECTION = 2,
        AT_INTERSECTION = 3,
        INTERSECTION_BLOCKED = 4,
        LEFT_HAND_RULE_ENTERING_INTERSECTION = 5,
        LEFT_HAND_RULE_AT_INTERSECTION = 6,
        LANES_RULES_ENTERING_INTERSECTION = 7,
        LANES_RULES_AT_INTERSECTION = 8,
        FORCING_PRIORITY = 9
    }

    public struct NPCVehicleComponent : IComponentData
    {
        public float3 frontCenterLocalPosition;
        public float3 backCenterLocalPosition;
        public NPCVehicleConfigComponent config;
        public bool isStoppedByFrontVehicle;

        // output from cognition (way following)
        public TrafficLaneComponent currentTrafficLane;  
        public int waypointIndex;

        // Output from Cognition (Curve)
        public bool isTurning;

        // output from cognition (front vehicle)
        public float distanceToFrontVehicle;

        // Output from Cognition (Right of Way)
        // public TrafficLane? YieldLane { get; set; }
        public NPCVehicleYieldPhase yieldPhase;
        public float3 yieldPoint;

        // output from decision
        public float3 targetPoint;
        public NPCVehicleSpeedMode speedMode;

        // output from control
        public float3 position;
        public float yaw;
        public float speed;
        public float yawSpeed;
        public float width;

        // output from any step
        public bool shouldDespawn;
    }

}
