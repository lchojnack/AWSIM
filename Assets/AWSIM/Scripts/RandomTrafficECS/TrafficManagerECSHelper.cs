using UnityEngine;
using Unity.Entities;
using Unity.Collections;

namespace AWSIM.TrafficSimulationECS
{
    public class TrafficManagerECSHelper : MonoBehaviour
    {

        [SerializeField] private TrafficManagerECS trafficManager;

     private EntityManager manager;

        private void Awake()
        {
            Debug.Log("ECS Start");
            if (trafficManager == null)
            {
                var go = GameObject.FindGameObjectsWithTag("TrafficManager")[0];
                trafficManager = go.GetComponent<TrafficManagerECS>();
            }
            if(trafficManager == null)
            {
                Debug.LogWarning("Set 'TafficManager' tag for test game object");
                return;
            }
            manager = World.DefaultGameObjectInjectionWorld.EntityManager;
        }

        private void Update()
        {
            var query = manager.CreateEntityQuery(typeof(NPCVehicleSpawnerComponent));
            if(query.CalculateEntityCount() == 1)
            {
                var entities = query.ToEntityArray(Allocator.TempJob);
                var data = manager.GetComponentData<NPCVehicleSpawnerComponent>(entities[0]);
                // Debug.Log($"current value {data.currentVehicleCount}");
                if (trafficManager != null)
                {
                    trafficManager.currentVehicleCount = data.currentVehicleCount;
                }
            }
            // Debug.Log("ECS update");
        }
    }



}