using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using HKDT.Connection;

public class PlayerConnector : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "PlayerConnector";
    public string send_topic_name = "/to_player";
    public string recv_topic_name = "/from_player";
    public string status_topic_name = "/status";

    [Header("UDP通信設定")]
    public string myIP = "192.168.11.1";
    public int myPort = 64201;
    private ROS2Node node;
    private FastUdpAgent udp_;
    // Start is called before the first frame update
    void Start()
    {
        udp_ = new FastUdpAgent(myIP, myPort);
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
