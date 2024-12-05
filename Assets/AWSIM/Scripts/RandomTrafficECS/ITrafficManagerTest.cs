using Unity.Entities;
using UnityEngine;
using System.Collections.Generic;

using Unity.Collections;
using Unity.Mathematics;

namespace AWSIM
{

    public interface ITrafficManagerTest
    {
        public void RestartTraffic();
        public void setMaxVehicleCount(int max);
        public void setTargetVehicleCount(int target);
        public int getCurrentVehicleCount();
    }

   
}