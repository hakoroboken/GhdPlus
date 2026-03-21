using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

public class ManualCmdManager : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "manual_cmd_manager";
    public string from_player_topic_name = "/from_player";
    public string wheel_cmd_topic = "/manual/wheel";
    public string machine_cmd_topic = "/manual/machine";

    [Header("足回り設定")]
    public double max_linear_velocity = 2.6; // [m/s]
    public double max_angular_velocity = 3.0; // [rad/s]

    [Header("機構設定")]
    public int updown_limit = 341;
    public int hand_limit = 83;

    private ROS2Node node;
    private ISubscription<std_msgs.msg.UInt8MultiArray> data_subscriber;
    private IPublisher<std_msgs.msg.Float64MultiArray> machine_target_cmd_publisher;
    private IPublisher<geometry_msgs.msg.Twist> wheel_target_cmd_publisher;

    // ロック機構
    private int prev_extension_input;
    private bool enable_extension;

    // 上昇機構
    private int updown_value;
    // 掴み機構
    private int hand_value;
    // ポンプ
    private int prev_left_vacuum_input;
    private bool left_vacuum_state;
    private int prev_right_vacuum_input;
    private bool right_vacuum_state;
    

    // Start is called before the first frame update
    void Start()
    {
        prev_extension_input = 0;
        prev_left_vacuum_input = 0;
        prev_right_vacuum_input = 0;
        left_vacuum_state = false;
        right_vacuum_state = false;
        enable_extension = false;
        updown_value = 0;
        hand_value = 0;
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok())
        {
            if(node == null)
            {
                node = ROSComponent.CreateNode(node_name);

                data_subscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(from_player_topic_name, FromPlayerDataCallback);

                machine_target_cmd_publisher = node.CreatePublisher<std_msgs.msg.Float64MultiArray>(machine_cmd_topic);
                wheel_target_cmd_publisher = node.CreatePublisher<geometry_msgs.msg.Twist>(wheel_cmd_topic);
            }
        }
    }

    private void FromPlayerDataCallback(std_msgs.msg.UInt8MultiArray msg)
    {
        var twist_msg = CreateManualWheelMsg(msg.Data);
        var machine_msg = CreateManualMachineMsg(msg.Data);

        wheel_target_cmd_publisher.Publish(twist_msg);
        machine_target_cmd_publisher.Publish(machine_msg);
    }

    private geometry_msgs.msg.Twist CreateManualWheelMsg(byte[] data)
    {
        int square = (data[4] >> 5) & 1;

        // コントローラー入力に変化があるとき
        if(square != prev_extension_input)
        {
            // さらに押されていたら
            if(square == 1)
            {
                enable_extension = !enable_extension;
            }
        }
        prev_extension_input = square;

        var twist_msg = new geometry_msgs.msg.Twist();

        twist_msg.Linear.X = max_linear_velocity * byte2double(data[0]);
        twist_msg.Linear.Y = max_linear_velocity * byte2double(data[1]);
        twist_msg.Angular.Z = -1.0* max_angular_velocity * byte2double(data[2]);

        twist_msg.Linear.Z = enable_extension ? 0.8 : 0.0; // trueのとき展開する。falseのときは0にする。

        return twist_msg;
    }

    private std_msgs.msg.Float64MultiArray CreateManualMachineMsg(byte[] data)
    {
        var machine_msg = new std_msgs.msg.Float64MultiArray();
        double[] machine_data = new double[8];

        // 十字キー上下
        int key_y = ((data[4] >> 0) & 1) - ((data[4] >> 2) & 1);

        int r1 = (data[5] >> 5) & 1;
        int l1 = (data[5] >> 4) & 1;

        //////// 上昇機構 ////////
        if(key_y > 0)
        {
            updown_value++;
        }
        else if(key_y < 0)
        {
            updown_value--;
        }

        if(updown_value > updown_limit)
        {
            updown_value = updown_limit;
        }
        else if(updown_value < 0)
        {
            updown_value = 0;
        }
        // 左の目標値
        machine_data[0] = updown_value;
        //////// 上昇機構 ////////
        
        //////// 掴み機構 ////////
        hand_value += r1 - l1;

        if(hand_value > hand_limit)
        {
            hand_value = hand_limit;
        }
        else if(hand_value < 0)
        {
            hand_value = 0;
        }
        // 右の目標値
        machine_data[1] = hand_value;
        //////// 掴み機構 ////////
        

        //////// ポンプ ////////
        int r2 = (data[5] >> 7) & 1;
        if(r2 != prev_right_vacuum_input)
        {
            // さらに押されていたら
            if(r2 == 1)
            {
                right_vacuum_state = !right_vacuum_state;
            }
        }

        int l2 = (data[5] >> 6) & 1;
        if(l2 != prev_left_vacuum_input)
        {
            if(l2 == 1)
            {
                left_vacuum_state = !left_vacuum_state;
            }
        }

        if(right_vacuum_state)
        {
            machine_data[2] = 1;
        }
        else
        {
            machine_data[2] = 0;
        }

        if(left_vacuum_state)
        {
            machine_data[3] = 1;
        }
        else
        {
            machine_data[3] = 0;
        }

        machine_msg.Data = machine_data;

        return machine_msg;
    }

    private double byte2double(byte b)
    {
        int integer = (int)b - 128;
        double converted = (double)integer / 127.0;

        if(converted > 1.0)return 1.0;
        if(converted < -1.0)return -1.0;

        return converted;
    }
}
