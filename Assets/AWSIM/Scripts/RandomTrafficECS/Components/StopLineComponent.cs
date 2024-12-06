using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

namespace AWSIM.TrafficSimulationECS
{
    [InternalBufferCapacity(16)]
    public struct Points : IBufferElementData
    {
        public float3 Value;
    }

    public struct StopLineComponent : IComponentData
    {
        public bool hasStopSign;
        public float3 centerPoint;
        public Unity.Entities.Entity trafficLight;
    }
}