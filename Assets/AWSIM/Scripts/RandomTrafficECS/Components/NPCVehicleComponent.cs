using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace AWSIM.TrafficSimulationECS
{
    public struct NPCVehicleConfigComponent : IComponentData
    {
        // config
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

    public struct NPCVehicleComponent : IComponentData
    {
        public float3 frontCenterLocalPosition;
        public float3 backCenterLocalPosition;
        public NPCVehicleConfigComponent config;

        // output from cognition
        public TrafficLaneComponent currentTrafficLane;  
        public int waypointIndex;

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
