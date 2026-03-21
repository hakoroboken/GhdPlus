using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

public class SensorParser : MonoBehaviour
{
    [Header("ROS2設定")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "SensorParser";
    public string left_mc_topic_name = "/from_left";
    public string right_mc_topic_name = "/from_right";

    public string wheel_sensor_topic = "/sensor/wheel";
    public string machine_sensor_topic = "/sensor/machine";

    private ROS2Node node;
    private ISubscription<std_msgs.msg.UInt8MultiArray> left_mc_subscriber;
    private ISubscription<std_msgs.msg.UInt8MultiArray> right_mc_subscriber;
    private IPublisher<std_msgs.msg.Float64MultiArray> wheel_sensor_publisher;
    private IPublisher<std_msgs.msg.Float64MultiArray> machine_sensor_publisher;

    private System.Threading.Thread publish_thread;

    // 一次保存
    private std_msgs.msg.UInt8MultiArray left_mc_data;
    private std_msgs.msg.UInt8MultiArray right_mc_data;


    // Start is called before the first frame update
    void Start()
    {
        left_mc_data = null;
        right_mc_data = null;
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok())
        {
            if(node == null)InitROS2();
        }        
    }

    private void InitROS2()
    {
        node = ROSComponent.CreateNode(node_name);

        left_mc_subscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(left_mc_topic_name, LeftMCCallback);
        right_mc_subscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(right_mc_topic_name, RightMCCallback);

        wheel_sensor_publisher = node.CreatePublisher<std_msgs.msg.Float64MultiArray>(wheel_sensor_topic);
        machine_sensor_publisher = node.CreatePublisher<std_msgs.msg.Float64MultiArray>(machine_sensor_topic);

        publish_thread = new System.Threading.Thread(PublishThread);
        publish_thread.Start();
    }

    private void LeftMCCallback(std_msgs.msg.UInt8MultiArray msg)
    {
        left_mc_data = msg;
    }

    private void RightMCCallback(std_msgs.msg.UInt8MultiArray msg)
    {
        right_mc_data = msg;
    }

    private void PublishThread()
    {
        if(left_mc_data != null && right_mc_data != null)
        {
            GhdCommon.Sensor sensor_data = new GhdCommon.Sensor(left_mc_data, right_mc_data);

            var wheel_msg = sensor_data.GetWheelSensorMsg();
            var machine_msg = sensor_data.GetMachineSensorMsg();

            wheel_sensor_publisher.Publish(wheel_msg);
            machine_sensor_publisher.Publish(machine_msg);
        }
    }
}
