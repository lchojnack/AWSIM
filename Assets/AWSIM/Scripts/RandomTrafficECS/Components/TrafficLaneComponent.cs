using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

namespace AWSIM.TrafficSimulationECS
{
    [InternalBufferCapacity(16)]
    public struct Waypoints : IBufferElementData
    {
        public float3 Value;
    }

    [InternalBufferCapacity(16)]
    public struct NextLanes : IBufferElementData
    {
        public TrafficLaneComponent Value;
    }

    public enum TurnDirectionType
    {
        STRAIGHT = 0,
        LEFT = 1,
        RIGHT = 2,
        NULL = 3
    }

    public struct TrafficLaneComponent : IComponentData
    {
        public int trafficLaneId;
        public TurnDirectionType turnDirection;
        public int stopLine;
        public float speedLimit;
        public bool intersectionLane;
    }
}