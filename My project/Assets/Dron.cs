using System;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.Std;

// Alias para resolver conflictos entre Unity y ROS
using PoseMsg = RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose;
using QuaternionMsg = RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion;

public class DronePositionPublisher : MonoBehaviour
{
    public string poseArrayTopic = "/drone/gps";
    private List<PoseMsg> gpsTrajectory = new List<PoseMsg>();
    private RosSocket rosSocket;
    private string poseArrayPublicationId;

    private const double baseLat = -35.3622;
    private const double baseLon = 149.1652;

    void Start()
    {
        rosSocket = GetComponent<RosConnector>().RosSocket;

        if (rosSocket == null)
        {
            Debug.LogError("❌ rosSocket es null.");
            return;
        }

        Debug.Log("✅ Conectado al RosBridge");
        poseArrayPublicationId = rosSocket.Advertise<PoseArray>(poseArrayTopic);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SaveCurrentPosition();
        }

        if (Input.GetKeyDown(KeyCode.F))
        {
            PublishFullTrajectory();
        }
    }

    void SaveCurrentPosition()
    {
        var pose = new PoseMsg
        {
            position = new Point
            {
                x = transform.position.z * 0.0001 + baseLat,
                y = transform.position.x * 0.0001 + baseLon,
                z = transform.position.y
            },
            orientation = new QuaternionMsg
            {
                x = 0,
                y = 0,
                z = 0,
                w = 1
            }
        };

        gpsTrajectory.Add(pose);
        Debug.Log($"📍 Guardada posición #{gpsTrajectory.Count}: {pose.position.x}, {pose.position.y}, {pose.position.z}");
    }

    void PublishFullTrajectory()
    {
        if (gpsTrajectory.Count == 0)
        {
            Debug.LogWarning("⚠️ No hay puntos almacenados.");
            return;
        }

        var message = new PoseArray
        {
            header = new Header
            {
                frame_id = "map",
            },
            poses = gpsTrajectory.ToArray()
        };

        rosSocket.Publish(poseArrayPublicationId, message);
        Debug.Log($"📡 Publicada trayectoria completa con {gpsTrajectory.Count} puntos.");
    }
}
