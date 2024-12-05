using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using AWSIM;
using AWSIM.TrafficSimulation;
using AWSIM.TrafficSimulationECS;
using Unity.Entities;
using Unity.Collections;

namespace FPSCounter
{

    public class FPSCounter : MonoBehaviour
    {
        [Header("Components")]
        [SerializeField] private Text _textField = default;
        [SerializeField] private Button _recordButton = default;
        [SerializeField] private Button _startButton = default;

        [Header("Vars")]
        [SerializeField] private float _samplingRate = 0.052f;
   
             
        [Header("Recording")]
        [SerializeField] private int _timeout = 60; 
        [SerializeField] private string _outputName = "";

        [Header("TrafficManager")]
        [SerializeField] private ITrafficManagerTest trafficManager;
        [SerializeField] private int[] targetVehicleCount;

        private int maxVehicleCount;
        private int iteratorVehicleCount = 0;
        private bool startTrafficTest = false;
        private float[] fpsAvgs = default;

        private bool startTest = false;

        private int _sampleCount = 0;
        private float[] _samples = default;

        private float _samplingTimer = 0f;
        private bool _record = false;

        private float _fps = 0f;

        private int _validSampleCount = 0;

        private int _saveFileId = 0;

        public bool ECS = false;
        public int currentVehicleCount = 0;
        
     private EntityManager manager;


        private void Start()
        {
            _sampleCount = Mathf.FloorToInt((float)_timeout / _samplingRate);
            _samples = new float[_sampleCount];

            _record = false;
            _samplingTimer = _samplingRate;

            _validSampleCount = 0;
            _saveFileId = 0;
            maxVehicleCount = targetVehicleCount.Length-1;
            fpsAvgs = new float[targetVehicleCount.Length];
            if (trafficManager == null)
            {
                var go = GameObject.FindGameObjectsWithTag("TrafficManager")[0];
                Debug.Log($"TM name {go.name}");
                trafficManager = go.GetComponent<ITrafficManagerTest>();
            }
            if(trafficManager == null)
            {
                Debug.LogWarning("Set 'TafficManager' tag for test game object");
                return;
            }
            if(ECS)
            {
                manager = World.DefaultGameObjectInjectionWorld.EntityManager;
            }
            trafficManager.setMaxVehicleCount(targetVehicleCount[maxVehicleCount]);
        }


        private void OnEnable() 
        {
            _recordButton.onClick.AddListener(OnRecordButtonClick);
            _startButton.onClick.AddListener(OnStartButtonClick);
        }

        private void OnDisable() 
        {
            _recordButton.onClick.RemoveListener(OnRecordButtonClick);
            _startButton.onClick.RemoveListener(OnStartButtonClick);
        }


        #region UI Callbacks

        private void OnRecordButtonClick()
        {
            if(_record)
            {
                return;
            }

            _recordButton.interactable = false;

            _validSampleCount = 0;
            for (int i = 0; i < _sampleCount; i++)
            {
                _samples[i] = 0f;
            }

            _record = true;        
        }

        private void OnStartButtonClick()
        {
            if(trafficManager == null)
            {
                Debug.LogWarning("Set 'TafficManager' tag for test game object");
                return;
            }

            iteratorVehicleCount = 0;
            startTest = true;
            _startButton.interactable = false;
        }


        #endregion

        private void Update()
        {
            if(ECS)
            {
                var query = manager.CreateEntityQuery(typeof(NPCVehicleSpawnerComponent));
                if(query.CalculateEntityCount() == 1)
                {
                    var entities = query.ToEntityArray(Allocator.TempJob);
                    var data = manager.GetComponentData<NPCVehicleSpawnerComponent>(entities[0]);
                    currentVehicleCount = data.currentVehicleCount;
                }
            }
            else
            {
                currentVehicleCount = trafficManager.getCurrentVehicleCount();
            }
            if(startTest)
            {
                
                var vehicleCount = targetVehicleCount[iteratorVehicleCount];
                if(!startTrafficTest)
                {
                    if(ECS)
                    {
                        var query = manager.CreateEntityQuery(typeof(NPCVehicleSpawnerComponent));
                        if(query.CalculateEntityCount() == 1)
                        {
                            var entities = query.ToEntityArray(Allocator.TempJob);
                            var data = manager.GetComponentData<NPCVehicleSpawnerComponent>(entities[0]);
                            data.targetVehicleCount = vehicleCount;
                            manager.SetComponentData<NPCVehicleSpawnerComponent>(entities[0], data);
                        }
                    // trafficManager.setTargetVehicleCount(vehicleCount);
                    }
                    else{
                    trafficManager.setTargetVehicleCount(vehicleCount);
                    trafficManager.RestartTraffic();

                    }
                    Debug.Log($"Wait for target number {vehicleCount}");
                    startTrafficTest = true;
                }

                if((currentVehicleCount == vehicleCount) && !_record)
                {
                    
                    Debug.Log("Start FPS record");
                    _saveFileId = vehicleCount;
                    OnRecordButtonClick();
                }
            }

            if (Time.unscaledTime >= _samplingTimer)
            {
                float fps = 1f / Time.unscaledDeltaTime;
                _textField.text = "FPS: " + (int)(fps);
    
                if(_record)
                {
                    AddSample(fps);
                }

                _samplingTimer = Time.unscaledTime + _samplingRate;
            }
        }

        private void AddSample(float value)
        {
            if(_validSampleCount < _sampleCount)
            {
                _samples[_validSampleCount] = value;
                _validSampleCount++;
            }
            else
            {
                _record = false;
                SaveData();
            }
        }


        private void SaveData()
        {
            string path = Application.persistentDataPath + "/" + _outputName + "_" + _saveFileId.ToString() + ".csv";

            Debug.Log(path);

            float avg = 0.0f;
            System.IO.StreamWriter writer = new System.IO.StreamWriter(path, false);
            for (int i = 0; i < _samples.Length; i++)
            {
                avg += _samples[i];
                writer.WriteLine(_samples[i].ToString());
            }
            writer.Close();
            avg /= _samples.Length;
            fpsAvgs[iteratorVehicleCount] = avg;
            Debug.Log($"avg fps: {avg} for {targetVehicleCount[iteratorVehicleCount]} vehicles");

            _saveFileId++;
            _recordButton.interactable = true;
            startTrafficTest = false;
            iteratorVehicleCount++;
            if(iteratorVehicleCount > maxVehicleCount)
            {
                startTest = false;
                _startButton.interactable = true;
                string path1 = Application.persistentDataPath + "/trafficManagerSummary" + ".csv";
                if(ECS)
                {
                    path1 = Application.persistentDataPath + "/trafficManagerSummaryECS" + ".csv";
                }
                Debug.Log(path1);
                System.IO.StreamWriter writer1 = new System.IO.StreamWriter(path1, false);
                writer1.WriteLine($"NoOfVehicles,FpsAvg");
                for (int i = 0; i < fpsAvgs.Length; i++)
                {
                    writer1.WriteLine($"{targetVehicleCount[i]},{fpsAvgs[i].ToString()}");
                }
                writer1.Close();
            }
        }

    }
}
