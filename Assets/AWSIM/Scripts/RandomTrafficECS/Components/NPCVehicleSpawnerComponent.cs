using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

namespace AWSIM.TrafficSimulationECS
{
    [InternalBufferCapacity(50)]
    public struct SpawnLanes : IBufferElementData
    {
        public TrafficLaneComponent Value;
    }

    [InternalBufferCapacity(15)]
    public struct NpcPrefabs : IBufferElementData
    {
        public Unity.Entities.Entity Entity;
        public float3 BoundsCenter;
        public float3 BoundsExtents;
        public float3 BoundsMax;
        public float3 BoundsMin;
        public float3 BoundsSize;
    }

    public struct NPCVehicleSpawnerComponent : IComponentData
    {
        public int spawnedVehicle;
        public int seed;
        public int maxVehicleCount;
    }
}