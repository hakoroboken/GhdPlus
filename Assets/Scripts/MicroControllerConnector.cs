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
    public string send_topic_name = "/to_mc";
    public string recv_topic_name = "/from_mc";
    public string status_topic_name = "/status";

    [Header("UDP通信設定")]
    public string myIP = "192.168.11.1";
    public int myPort = 64201;
    public string destIP = "192.168.11.1";
    public int destPort = 64202;
    private ROS2Node node;
    private FastUdpServer udp_;
    // Start is called before the first frame update
    void Start()
    {
        udp_ = new FastUdpServer(myIP, myPort, destIP, destPort);
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok())
        {
            if(node == null)
            {
                node = ROSComponent.CreateNode(node_name);

                bool udp_status = udp_.Start(node, send_topic_name, recv_topic_name, status_topic_name);

                if(udp_status)
                {
                    Debug.Log($"[{node_name}] 初期化成功");
                }
                else
                {
                    Debug.LogError($"");
                }
            }
        }
    }
}
