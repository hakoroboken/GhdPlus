using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using TMPro;

public class StatusVisualizer : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "status_visualizer";
    public string left_mc_status_topic = "/left_mc_status";
    public string right_mc_status_topic = "/right_mc_status";
    public string player_status_topic = "/player_status";

    [Header("表示するオブジェクト")]
    public GameObject left_mc_status_text;
    public GameObject right_mc_status_text;
    public GameObject player_status_text;


    private ROS2Node node;
    private ISubscription<std_msgs.msg.Bool> left_mc_status_subscriber;
    private ISubscription<std_msgs.msg.Bool> right_mc_status_subscriber;
    private ISubscription<std_msgs.msg.Bool> player_status_subscriber;

    private string left_mc_status = "";
    private string right_mc_status = "";
    private string player_status = "";
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok())
        {
            if(node == null)InitROS2();
        }

        var left_mc_text = left_mc_status_text.GetComponent<TextMeshProUGUI>();
        left_mc_text.text = left_mc_status;
        var right_mc_text = right_mc_status_text.GetComponent<TextMeshProUGUI>();
        right_mc_text.text = right_mc_status;
        var player_text = player_status_text.GetComponent<TextMeshProUGUI>();
        player_text.text = player_status;
    }

    private void InitROS2()
    {
        node = ROSComponent.CreateNode(node_name);

        left_mc_status_subscriber = node.CreateSubscription<std_msgs.msg.Bool>(left_mc_status_topic, LeftMCStatusCallback);
        right_mc_status_subscriber = node.CreateSubscription<std_msgs.msg.Bool>(right_mc_status_topic, RightMCStatusCallback);
        player_status_subscriber = node.CreateSubscription<std_msgs.msg.Bool>(player_status_topic, PlayerStatusCallback);
    }

    private void LeftMCStatusCallback(std_msgs.msg.Bool msg)
    {
        left_mc_status = "Left MC: " + (msg.Data ? "OK" : "ERROR");
    }

    private void RightMCStatusCallback(std_msgs.msg.Bool msg)
    {
        right_mc_status = "Right MC: " + (msg.Data ? "OK" : "ERROR");
    }

    private void PlayerStatusCallback(std_msgs.msg.Bool msg)
    {
        player_status = "Player: " + (msg.Data ? "OK" : "ERROR");
    }
}
