using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

public class ModeManager : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "mode_manager";
    [Header("サブスクライバーのトピック")]
    public string mode_topic = "/mode";
    public string manual_wheel_topic = "/manual/wheel";
    public string manual_machine_topic = "/manual/machine";
    public string auto_wheel_topic = "/auto/wheel";
    [Header("パブリッシャーのトピック")]
    public string pub_wheel_topic = "/target/wheel";
    public string pub_machine_topic = "/target/machine";

    private ROS2Node node;
    private ISubscription<std_msgs.msg.UInt8> mode_subscriber;
    private ISubscription<geometry_msgs.msg.Twist> manual_wheel_cmd_subscriber;
    private ISubscription<geometry_msgs.msg.Twist> auto_wheel_cmd_subscriber;
    private ISubscription<std_msgs.msg.Float64MultiArray> manual_machine_cmd_subscriber;
    private IPublisher<geometry_msgs.msg.Twist> target_wheel_publisher;
    private IPublisher<std_msgs.msg.Float64MultiArray> target_machine_publisher;
    private geometry_msgs.msg.Twist manual_wheel;
    private geometry_msgs.msg.Twist auto_wheel;
    private std_msgs.msg.Float64MultiArray manual_machine;
    private AutoCommon.Mode current_mode;
    
    // Start is called before the first frame update
    void Start()
    {
        current_mode = AutoCommon.Mode.STOP;
        manual_wheel = new geometry_msgs.msg.Twist();
        auto_wheel = new geometry_msgs.msg.Twist();
        manual_machine = new std_msgs.msg.Float64MultiArray();
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok())
        {
            if(node == null)InitROS2();

            if(current_mode == AutoCommon.Mode.AUTO)
            {
                target_wheel_publisher.Publish(auto_wheel);
                target_machine_publisher.Publish(manual_machine); //機構は自由
            }
            else if(current_mode == AutoCommon.Mode.RESET)
            {
                var stop_wheel = manual_wheel;
                // Linear.Zに展開量が入っているので０にする
                stop_wheel.Linear.Z = 0.0;
                var reset_machine = manual_machine;
                // すべての関節を０にする
                for(int i = 0; i < reset_machine.Data.Length; i++)
                {
                    reset_machine.Data[i] = 0.0;
                }
                target_wheel_publisher.Publish(stop_wheel);
                target_machine_publisher.Publish(reset_machine);
            }
            else
            {
                if(manual_wheel != null)
                {
                    target_wheel_publisher.Publish(manual_wheel);
                    target_machine_publisher.Publish(manual_machine);
                }
            }
        }
    }

    private void InitROS2()
    {
        node = ROSComponent.CreateNode(node_name);

        manual_wheel_cmd_subscriber = node.CreateSubscription<geometry_msgs.msg.Twist>(manual_wheel_topic, ManualWheelCallback);
        auto_wheel_cmd_subscriber = node.CreateSubscription<geometry_msgs.msg.Twist>(auto_wheel_topic, AutoWheelCallback);
        manual_machine_cmd_subscriber = node.CreateSubscription<std_msgs.msg.Float64MultiArray>(manual_machine_topic, ManualMachineCallback);

        mode_subscriber = node.CreateSubscription<std_msgs.msg.UInt8>(mode_topic, ModeCallback);

        target_wheel_publisher = node.CreatePublisher<geometry_msgs.msg.Twist>(pub_wheel_topic);
        target_machine_publisher = node.CreatePublisher<std_msgs.msg.Float64MultiArray>(pub_machine_topic);
    }

    private void ManualWheelCallback(geometry_msgs.msg.Twist msg)
    {
        manual_wheel = msg;
    }

    private void AutoWheelCallback(geometry_msgs.msg.Twist msg)
    {
        auto_wheel = msg;
    }

    private void ManualMachineCallback(std_msgs.msg.Float64MultiArray msg)
    {
        manual_machine = msg;
    }

    private void ModeCallback(std_msgs.msg.UInt8 msg)
    {
        switch (msg.Data)
        {
            case (byte)AutoCommon.Mode.AUTO:
                current_mode = AutoCommon.Mode.AUTO;
                break;
            case (byte)AutoCommon.Mode.STOP:
                current_mode = AutoCommon.Mode.STOP;
                break;
            case (byte)AutoCommon.Mode.RESET:
                current_mode = AutoCommon.Mode.RESET;
                break;
            default:
                Debug.LogWarning("Unknown mode received: " + msg.Data);
                break;
        }
    }
}
