using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using AutoCommon;

public class AutoCmdManager : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "auto_cmd_manager";
    public string mode_topic = "/mode";
    public string target_pose_topic = "/goal_pose";
    public string current_pose_topic = "/current_pose";
    public string to_web_ui_topic = "/web_ui/odometry";
    public string from_web_ui_topic = "/web_ui/cmd";

    [Header("自律走行設定")]
    public List<Vector3> waypoints = new List<Vector3>(); // 自律走行の目標地点のリスト
    public double distanceThreshold = 0.1; // 目標地点に到達したとみなす距離
    public double rotationThreshold = 0.1; // 目標角度に到達したとみなす角度（ラジアン）

    private ROS2Node node;
    private ISubscription<geometry_msgs.msg.PoseStamped> current_pose_subscriber;
    private ISubscription<geometry_msgs.msg.Point> webui_cmd_subscriber;
    private IPublisher<geometry_msgs.msg.Point> webui_odometry_publisher;
    private IPublisher<geometry_msgs.msg.PoseStamped> target_pose_publisher;
    private IPublisher<std_msgs.msg.UInt8> mode_publisher;

    private Mode current_mode;
    private int current_waypoint_index = 0;

    // Start is called before the first frame update
    void Start()
    {
        current_waypoint_index = 0;
        current_mode = Mode.STOP; // 初期モードはSTOP
    }

    // Update is called once per frame
    void Update()
    {
        if(ROSComponent.Ok() && node == null)initROS2();
    }

    private void initROS2()
    {
        node = ROSComponent.CreateNode(node_name);

        current_pose_subscriber = node.CreateSubscription<geometry_msgs.msg.PoseStamped>(current_pose_topic, CurrentPoseCallback);
        webui_cmd_subscriber = node.CreateSubscription<geometry_msgs.msg.Point>(from_web_ui_topic, FromWebUICallback);
        webui_odometry_publisher = node.CreatePublisher<geometry_msgs.msg.Point>(to_web_ui_topic);
        target_pose_publisher = node.CreatePublisher<geometry_msgs.msg.PoseStamped>(target_pose_topic);
        mode_publisher = node.CreatePublisher<std_msgs.msg.UInt8>(mode_topic);
    }

    private void CurrentPoseCallback(geometry_msgs.msg.PoseStamped msg)
    {
        double currentTheta = HKDT.Auto.Common.TransformUtils.QuatToYaw(msg.Pose.Orientation);
        geometry_msgs.msg.Point to_webui_msg = new geometry_msgs.msg.Point
        {
            X = msg.Pose.Position.X,
            Y = msg.Pose.Position.Y,
            Z = currentTheta
        };
        webui_odometry_publisher.Publish(to_webui_msg);

        if(current_mode == Mode.AUTO)
        {
            if(waypoints.Count == 0)
            {
                Debug.LogError("No waypoints available for autonomous driving.");
                return;
            }

            Vector3 targetPosition = waypoints[current_waypoint_index];
            Vector3 currentPosition = ToVector3(msg.Pose);

            if(Vector3.Distance(currentPosition, targetPosition) < distanceThreshold)
            {
                current_waypoint_index++;
                if(current_waypoint_index >= waypoints.Count)
                {
                    current_mode = Mode.STOP; // 全てのウェイポイントに到達したら停止
                    current_waypoint_index = 0; // ウェイポイントのリストの最初に戻す
                    Debug.Log("Reached final waypoint. Stopping.");
                    return;
                }
                else
                {
                    Debug.Log("Reached waypoint " + (current_waypoint_index - 1) + ". Moving to next waypoint.");
                }
            }


            var target_pose_msg = CreatePoseStamped(targetPosition);
            target_pose_publisher.Publish(target_pose_msg);
        }
    }

    private void FromWebUICallback(geometry_msgs.msg.Point msg)
    {
        switch(msg.X)
        {
            case 0.0:
                current_waypoint_index = 0; // ウェイポイントのリストの最初からスタート
                current_mode = Mode.AUTO;
                break;
            case 1.0:
                current_mode = Mode.STOP;
                break;
            case 2.0:
                current_mode = Mode.RESET;
                break;
            default:
                Debug.LogWarning("Received unknown command from web UI: " + msg.X);
                break;
        }

        var mode_msg = new std_msgs.msg.UInt8
        {
            Data = (byte)current_mode
        };
        mode_publisher.Publish(mode_msg);
    }

    private geometry_msgs.msg.PoseStamped CreatePoseStamped(Vector3 position)
    {
        return new geometry_msgs.msg.PoseStamped
        {
            Header = new std_msgs.msg.Header
            {
                Frame_id = "map",
            },
            Pose = new geometry_msgs.msg.Pose
            {
                Position = new geometry_msgs.msg.Point
                {
                    X = position.x,
                    Y = position.y,
                    Z = 0
                },
                Orientation = HKDT.Auto.Common.TransformUtils.YawToQuat(position.z)
            }
        };
    }

    private Vector3 ToVector3(geometry_msgs.msg.Pose pose)
    {
        return new Vector3
        {
            x = (float)pose.Position.X,
            y = (float)pose.Position.Y,
            z = (float)HKDT.Auto.Common.TransformUtils.QuatToYaw(pose.Orientation)
        };
    }
}
