using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

using HKDT.Connection;

/// <summary>
/// マイコンとUDP通信する
/// </summary>
public class MicroControllerConnector : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "MicroControllerConnector";

    [Header("UDP通信設定")]
    public string myIP = "192.168.11.1";
    private ROS2Node node;
    private FastUdpServer udp_;
    // Start is called before the first frame update
    void Start()
    {
        node = ROSComponent.CreateNode(node_name);
        
    }

    // Update is called once per frame
    void Update()
    {
    }
}
