using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

public class SwerveNavigation : MonoBehaviour
{
    [Header("ROS2設定")]
    [Tooltip("ROS2UnityComponentをつけたオブジェクトを入れる")]
    public ROS2UnityComponent ROSComponent;
    public string node_name = "swerve_navigation";
    public string cmd_topic_name = "/wheel_cmd";
    public string target_topic_name = "/target/wheel";
    public string lock_cmd_topic_name = "/lock_cmd";
    public string lock_feedback_topic_name = "/lock_feedback";
    public string wheel_feedback_topic_name = "/wheel_feedback";

    [Header("足回り設定")]
    public float steer_yoko_length = 0.25f; // [m]
    public float steer_tate_length = 0.587365f; // [m]
    [Header("左右展開の設定")]
    public float extension_threshold = 0.05f; // [m] 展開モードと走行モードの切り替え閾値
    [Tooltip("左右展開に入る許可を得る角度差")]
    public float angle_threshold = 30f; // [回転]

    public float p_gain = 1.00f;
    public float i_gain = 0.00f;
    public float d_gain = 0.00f;
    public float integralMax = 1.00f;
    public float maxVelocity = 0.5f;

    private ROS2Node node;
    private ISubscription<geometry_msgs.msg.Twist> cmd_subscriber;
    private ISubscription<std_msgs.msg.Bool> lock_feedback_subscriber;
    private ISubscription<std_msgs.msg.Float64MultiArray> wheel_feedback_subscriber;
    private IPublisher<std_msgs.msg.Float64MultiArray> wheel_target_publisher;
    private IPublisher<std_msgs.msg.Bool> lock_target_publisher;

    private System.Threading.Thread control_thread;

    // 一次保存バッファ
    private geometry_msgs.msg.Twist current_cmd;
    private std_msgs.msg.Bool lock_feedback;
    private GhdCommon.MotorDataSet wheel_feedback;

    // 左右展開用のPID
    private HKDT.Auto.Control.PID pid;
    // Start is called before the first frame update
    void Start()
    {
        pid = new HKDT.Auto.Control.PID(p_gain, i_gain, d_gain, integralMax, -maxVelocity, maxVelocity);
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

        cmd_subscriber = node.CreateSubscription<geometry_msgs.msg.Twist>(cmd_topic_name, CmdCallback);
        lock_feedback_subscriber = node.CreateSubscription<std_msgs.msg.Bool>(lock_feedback_topic_name, LockFeedbackCallback);
        wheel_feedback_subscriber = node.CreateSubscription<std_msgs.msg.Float64MultiArray>(wheel_feedback_topic_name, WheelFeedbackCallback);

        wheel_target_publisher = node.CreatePublisher<std_msgs.msg.Float64MultiArray>(target_topic_name);
        lock_target_publisher = node.CreatePublisher<std_msgs.msg.Bool>(lock_cmd_topic_name);

        control_thread = new System.Threading.Thread(ControlThread);
        control_thread.Start();
    }

    /// <summary>
    /// 主に制御はここで行う
    /// </summary>
    private void ControlThread()
    {
        while(true)
        {
            if(current_cmd == null || lock_feedback == null || wheel_feedback == null)continue;

            // Linear.Zには目標展開量が入っているので、現在の展開量との差分を取る
            double delta_extension = Math.Abs(current_cmd.Linear.Z - wheel_feedback.extend_length);

            // 展開量の差分が閾値以下かつロック機構が閉じてる(true)なら走行モード、そうでなければ展開モード
            if(delta_extension < extension_threshold && lock_feedback.Data == true)
            {
                // 走行モード
                DriveMode();
            }
            else
            {
                // 展開モード
                ExtensionMode((float)delta_extension);
            }
        }
    }

    private void ExtensionMode(float delta_extension)
    {
        // 展開モードは３つの状態に分ける
        // 1. 展開前の準備
        // 2. 展開
        // 3. 展開後の終了

        // 展開前の準備では以下を満たすまでは次に移行しない
        // A.各ユニットの角度が０に近い B.ロック機構が空いている

        // falseのときに準備モード
        if(!IsReadyForExtension())
        {
            var targetValues = new GhdCommon.MotorDataSet();
            targetValues.fl_unit.m2006_position = 0.0;
            targetValues.fr_unit.m2006_position = 0.0;
            targetValues.rl_unit.m2006_position = 0.0;
            targetValues.rr_unit.m2006_position = 0.0;

            targetValues.fl_unit.m3508_rpm = 0.0;
            targetValues.fr_unit.m3508_rpm = 0.0;
            targetValues.rl_unit.m3508_rpm = 0.0;
            targetValues.rr_unit.m3508_rpm = 0.0;

            wheel_target_publisher.Publish(targetValues.GetWheelSensorMsg());

            var lock_target_msg = new std_msgs.msg.Bool
            {
                Data = false
            };

            lock_target_publisher.Publish(lock_target_msg);
        }
        else if(IsReadyForExtension() && delta_extension > extension_threshold)
        {
            // ここは展開準備が終わっている　かつ　目標展開量との間に誤差があるとき￥
            
            double targetVelocity = pid.Compute(current_cmd.Linear.Z, wheel_feedback.extend_length, 0.02);

            var targetValues = new GhdCommon.MotorDataSet();
            targetValues.fl_unit.m2006_position = 0.0;
            targetValues.fr_unit.m2006_position = 0.0;
            targetValues.rl_unit.m2006_position = 0.0;
            targetValues.rr_unit.m2006_position = 0.0;

            targetValues.fl_unit.m3508_rpm = -targetVelocity;
            targetValues.fr_unit.m3508_rpm = -targetVelocity;
            targetValues.rl_unit.m3508_rpm = targetVelocity;
            targetValues.rr_unit.m3508_rpm = targetVelocity;

            wheel_target_publisher.Publish(targetValues.GetWheelSensorMsg());
        }
        else if(IsReadyForExtension() && delta_extension < extension_threshold)
        {
            var targetValues = new GhdCommon.MotorDataSet();
            targetValues.fl_unit.m2006_position = 0.0;
            targetValues.fr_unit.m2006_position = 0.0;
            targetValues.rl_unit.m2006_position = 0.0;
            targetValues.rr_unit.m2006_position = 0.0;

            targetValues.fl_unit.m3508_rpm = 0.0;
            targetValues.fr_unit.m3508_rpm = 0.0;
            targetValues.rl_unit.m3508_rpm = 0.0;
            targetValues.rr_unit.m3508_rpm = 0.0;

            wheel_target_publisher.Publish(targetValues.GetWheelSensorMsg());

            var lock_target_msg = new std_msgs.msg.Bool
            {
                Data = true
            };

            lock_target_publisher.Publish(lock_target_msg);
        }
    }

    //　準備完了ならtrueを返す
    private bool IsReadyForExtension()
    {
        // 何事もなければtrueを返す
        bool enable = true;

        if(Math.Abs(wheel_feedback.fl_unit.m2006_position) > angle_threshold)enable = false;
        if(Math.Abs(wheel_feedback.fr_unit.m2006_position) > angle_threshold)enable = false;
        if(Math.Abs(wheel_feedback.rl_unit.m2006_position) > angle_threshold)enable = false;
        if(Math.Abs(wheel_feedback.rr_unit.m2006_position) > angle_threshold)enable = false;

        enable = !lock_feedback.Data;

        return enable;
    }

    /// <summary>
    /// 通常走行する際のモード
    /// </summary>
    private void DriveMode()
    {
        var front_left_feedback = new HKDT.Kinematics.SwerveUnit();
        var front_right_feedback = new HKDT.Kinematics.SwerveUnit();
        var rear_left_feedback = new HKDT.Kinematics.SwerveUnit();
        var rear_right_feedback = new HKDT.Kinematics.SwerveUnit();
        // 各エンコーダ値をラジアンにして保存
        // m2006が36分の１かつギア比が４分の１なことに注意する
        front_left_feedback.angle =  2.0f * Mathf.PI * (float)wheel_feedback.fl_unit.m2006_position / 144.0f;
        front_right_feedback.angle = 2.0f * Mathf.PI * (float)wheel_feedback.fr_unit.m2006_position / 144.0f;
        rear_left_feedback.angle =   2.0f * Mathf.PI * (float)wheel_feedback.rl_unit.m2006_position / 144.0f;
        rear_right_feedback.angle =  2.0f * Mathf.PI * (float)wheel_feedback.rr_unit.m2006_position / 144.0f;

        // 独立ステアリングライブラリを利用する
        var swerveCalculator = new HKDT.Kinematics.Swerve(steer_yoko_length, steer_tate_length, 2.6f);

        swerveCalculator.SetPreviousState(front_left_feedback, front_right_feedback, rear_left_feedback, rear_right_feedback);

        swerveCalculator.Calculate((float)current_cmd.Linear.X, (float)current_cmd.Linear.Y, (float)current_cmd.Angular.Z);

    

        var targetValues = new GhdCommon.MotorDataSet();
        targetValues.fl_unit.m2006_position = swerveCalculator.GetFLUnit().angle;
        targetValues.fr_unit.m2006_position = swerveCalculator.GetFRUnit().angle;
        targetValues.rl_unit.m2006_position = swerveCalculator.GetRLUnit().angle;
        targetValues.rr_unit.m2006_position = swerveCalculator.GetRRUnit().angle;

        targetValues.fl_unit.m3508_rpm = swerveCalculator.GetFLUnit().speed;
        targetValues.fr_unit.m3508_rpm = swerveCalculator.GetFRUnit().speed;
        targetValues.rl_unit.m3508_rpm = swerveCalculator.GetRLUnit().speed;
        targetValues.rr_unit.m3508_rpm = swerveCalculator.GetRRUnit().speed;

        var target_msg = targetValues.GetWheelSensorMsg();

        wheel_target_publisher.Publish(target_msg);
    }


    // 走行命令を受け取ったときのコールバック関数
    private void CmdCallback(geometry_msgs.msg.Twist msg)
    {
        current_cmd = msg;
    }

    // ロックフィードバックを受け取ったときのコールバック関数
    private void LockFeedbackCallback(std_msgs.msg.Bool msg)
    {
        lock_feedback = msg;
    }

    // ホイールフィードバックを受け取ったときのコールバック関数
    private void WheelFeedbackCallback(std_msgs.msg.Float64MultiArray msg)
    {
        wheel_feedback = new GhdCommon.MotorDataSet();
        wheel_feedback.SetWheelMsg(msg);
    }
}
