using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

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

public struct TrafficLaneComponent : IComponentData
{
    public int trafficLaneId;
    public int turnDirection;
    public float3 startPoint;
    public float3 endPoint;
    public int stopLine;
    public float speedLimit;
    public bool intersectionLane;
}
