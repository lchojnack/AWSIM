using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

namespace AWSIM.TrafficSimulationECS
{
    /// <summary>
    /// Type of each bulb.
    /// </summary>
    public enum BulbType
    {
        ANY_CIRCLE_BULB = 0,
        RED_BULB = 1,
        YELLOW_BULB = 2,
        GREEN_BULB = 3,
        LEFT_ARROW_BULB = 4,
        RIGHT_ARROW_BULB = 5,
        UP_ARROW_BULB = 6,
        DOWN_ARROW_BULB = 7,
        DOWN_LEFT_ARROW_BULB = 8,
        DOWN_RIGHT_ARROW_BULB = 9,
        CROSS_BULB = 10,
    }

    /// <summary>
    /// Bulb lighting status.
    /// </summary>
    public enum BulbStatus
    {
        SOLID_OFF = 0,        // Lights off.
        SOLID_ON = 1,        // Lights on.
        FLASHING = 2,        // Lights on every flashSec.
    }

    /// <summary>
    /// Bulb lighting color.
    /// </summary>
    public enum BulbColor
    {
        RED = 0,
        YELLOW = 1,
        GREEN = 2,
        WHITE = 3,
    }

    [InternalBufferCapacity(16)]
    public struct BulbData : IBufferElementData
    {
        public BulbType type;
        public BulbColor color;
        public BulbStatus status;
    }

    public struct TrafficLightComponent : IComponentData
    {
        
    }
}