using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

public struct NPCVehicleComponent : IComponentData
{
    public TrafficLaneComponent currentTrafficLane;

    public float moveSpeed;
    public float3 goToPosition;
}
