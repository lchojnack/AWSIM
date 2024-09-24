using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;


[InternalBufferCapacity(50)]
public struct SpawnLanes : IBufferElementData
{
    public TrafficLaneComponent Value;
}

[InternalBufferCapacity(15)]
public struct NpcPrefabs : IBufferElementData
{
    public Entity Value;
}

public struct NPCVehicleSpawnerComponent : IComponentData
{
    public int seed;
    public int maxVehicleCount;
    public Entity npcPrefab;
    public TrafficLaneComponent spawnLane;
}
