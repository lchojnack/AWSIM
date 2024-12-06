using UnityEditor;
using UnityEngine;
using AWSIM.TrafficSimulation;

namespace AWSIM.Scripts.Editor.RenameStopLines
{
    public class RenameStopLines : EditorWindow
    {
        [MenuItem("AWSIM/RenameStopLines")]
        public static void ShowWindow()
        {
            GetWindow<RenameStopLines>("RenameStopLines");
        }

        private void OnGUI()
        {
            GUILayout.Label("Base Settings", EditorStyles.boldLabel);

            // This can load from any path doesn't have to be from /Externals.
            // ./Assets/AWSIM/Externals/
            // _sensorKitPath = EditorGUILayout.TextField("Sensor Kit Path", _sensorKitPath);

            // GUI button to load URDF file
            if (GUILayout.Button("Start"))
            {
                var allStopLines = GameObject.FindObjectsOfType<StopLine>();
                int i = 0;
                foreach (var stopLine in allStopLines)
                {
                    stopLine.name = $"StopLine.{i}";
                    i++;
                }
                Debug.Log("StopLines Renamed");
            }

            // EditorGUI.BeginChangeCheck();
        }


    }
}
