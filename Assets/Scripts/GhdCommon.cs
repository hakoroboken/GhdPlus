using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using std_msgs;

namespace GhdCommon
{
    struct SwerveUnit
    {
        public double m3508_rpm;
        public double m3508_position;
        public double m2006_rpm;
        public double m2006_position;
    };

    struct Motor
    {
        public double rpm;
        public double position;
    }

    enum SwerveIndex : int
    {
        FL_2006_RPM=0,
        FL_2006_POS=1,
        FL_3508_RPM=2,
        FL_3508_POS=3,
        RL_2006_RPM=4,
        RL_2006_POS=5,
        RL_3508_RPM=6,
        RL_3508_POS=7,
        FR_2006_RPM=8,
        FR_2006_POS=9,
        FR_3508_RPM=10,
        FR_3508_POS=11,
        RR_2006_RPM=12,
        RR_2006_POS=13,
        RR_3508_RPM=14,
        RR_3508_POS=15,
        LOCK_RPM = 16,
        LOCK_POS = 17,
        EXTEND_LENGTH = 18
    }

    enum MachineIndex : int
    {
        LEFT_UPDOWN_POS = 1,
        RIGHT_UPDOWN_POS = 2,
        LEFT_HAND_POS = 3,
        RIGHT_HAND_POS = 4,

        LEFT_UPDOWN_RPM = 5,
        RIGHT_UPDOWN_RPM = 6,
        LEFT_HAND_RPM = 7,
        RIGHT_HAND_RPM = 8
    }
    class MotorDataSet
    {
        public double extend_length;
        
        public SwerveUnit fl_unit;
        public SwerveUnit fr_unit;
        public SwerveUnit rl_unit;
        public SwerveUnit rr_unit;

        public Motor lock_motor;
        public Motor left_updown;
        public Motor left_hand;
        public Motor right_updown;
        public Motor right_hand;

        public MotorDataSet()
        {
            
        }

        public MotorDataSet(std_msgs.msg.UInt8MultiArray left, std_msgs.msg.UInt8MultiArray right)
        {
            short[] left_rpms = new short[8];
            short[] left_positions = new short[8];

            short[] right_rpms = new short[8];
            short[] right_positions = new short[8];

            for(int i = 1; i < 9; i++)
            {
                left_rpms[i-1] = (short)(left.Data[4*i-3] << 8 | left.Data[4*i-2]);
                left_positions[i-1] = (short)(left.Data[4*i-1]<< 8 | left.Data[4*i]);

                right_rpms[i-1] = (short)(right.Data[4*i-3] << 8 | right.Data[4*i-2]);
                right_positions[i-1] = (short)(right.Data[4*i-1]<< 8 | right.Data[4*i]);
            }

            short encoder = (short)(right.Data[33] << 8 | right.Data[34]);
            double r = encoder / 100.0;

            extend_length = -1.0 * r * 0.018 * System.Math.PI * 2.0;

            // 左前ユニット
            fl_unit.m2006_position = (double)left_positions[0];
            fl_unit.m2006_rpm = (double)left_rpms[0];

            fl_unit.m3508_position = (double)left_positions[1];
            fl_unit.m3508_rpm = (double)left_rpms[1];

            // 左後ユニット
            rl_unit.m2006_position = (double)left_positions[2];
            rl_unit.m2006_rpm = (double)left_rpms[2];

            rl_unit.m3508_position = (double)left_positions[3];
            rl_unit.m3508_rpm = (double)left_rpms[3];

            // 右前ユニット
            fr_unit.m2006_position = (double)right_positions[0];
            fr_unit.m2006_rpm = (double)right_rpms[0];

            fr_unit.m3508_position = (double)right_positions[1];
            fr_unit.m3508_rpm = (double)right_rpms[1];

            // 右後ユニット
            rr_unit.m2006_position = (double)right_positions[2];
            rr_unit.m2006_rpm = (double)right_rpms[2];

            rr_unit.m3508_position = (double)right_positions[3];
            rr_unit.m3508_rpm = (double)right_rpms[3];

            // ロック機構
            lock_motor.position = right_positions[4];
            lock_motor.rpm = right_rpms[4];

            // 左上昇機構
            left_updown.position = left_positions[5];
            left_updown.rpm = left_rpms[5];

            // 左掴み
            left_hand.position = left_positions[6];
            left_hand.rpm = left_rpms[6];

            // 右上昇機構
            right_updown.position = right_positions[5];
            right_updown.rpm = right_rpms[5];

            // 右掴み
            right_hand.position = right_positions[6];
            right_hand.rpm = right_rpms[6];
        }

        public void SetWheelMsg(std_msgs.msg.Float64MultiArray wheel_msg)
        {
            fl_unit.m2006_position = wheel_msg.Data[(int)SwerveIndex.FL_2006_POS];
            fl_unit.m2006_rpm = wheel_msg.Data[(int)SwerveIndex.FL_2006_RPM];
            fl_unit.m3508_position = wheel_msg.Data[(int)SwerveIndex.FL_3508_POS];
            fl_unit.m3508_rpm = wheel_msg.Data[(int)SwerveIndex.FL_3508_RPM];

            fr_unit.m2006_position = wheel_msg.Data[(int)SwerveIndex.FR_2006_POS];
            fr_unit.m2006_rpm = wheel_msg.Data[(int)SwerveIndex.FR_2006_RPM];
            fr_unit.m3508_position = wheel_msg.Data[(int)SwerveIndex.FR_3508_POS];
            fr_unit.m3508_rpm = wheel_msg.Data[(int)SwerveIndex.FR_3508_RPM];

            rl_unit.m2006_position = wheel_msg.Data[(int)SwerveIndex.RL_2006_POS];
            rl_unit.m2006_rpm = wheel_msg.Data[(int)SwerveIndex.RL_2006_RPM];
            rl_unit.m3508_position = wheel_msg.Data[(int)SwerveIndex.RL_3508_POS];
            rl_unit.m3508_rpm = wheel_msg.Data[(int)SwerveIndex.RL_3508_RPM];

            rr_unit.m2006_position = wheel_msg.Data[(int)SwerveIndex.RR_2006_POS];
            rr_unit.m2006_rpm = wheel_msg.Data[(int)SwerveIndex.RR_2006_RPM];
            rr_unit.m3508_position = wheel_msg.Data[(int)SwerveIndex.RR_3508_POS];
            rr_unit.m3508_rpm = wheel_msg.Data[(int)SwerveIndex.RR_3508_RPM];

            lock_motor.position = wheel_msg.Data[(int)SwerveIndex.LOCK_POS];
            lock_motor.rpm = wheel_msg.Data[(int)SwerveIndex.LOCK_RPM];

            extend_length = wheel_msg.Data[(int)SwerveIndex.EXTEND_LENGTH];
        }

        public void SetMachineMsg(std_msgs.msg.Float64MultiArray machine_msg)
        {
            left_updown.position = machine_msg.Data[(int)MachineIndex.LEFT_UPDOWN_POS];
            left_updown.rpm = machine_msg.Data[(int)MachineIndex.LEFT_UPDOWN_RPM];

            right_updown.position = machine_msg.Data[(int)MachineIndex.RIGHT_UPDOWN_POS];
            right_updown.rpm = machine_msg.Data[(int)MachineIndex.RIGHT_UPDOWN_RPM];

            left_hand.position = machine_msg.Data[(int)MachineIndex.LEFT_HAND_POS];
            left_hand.rpm = machine_msg.Data[(int)MachineIndex.LEFT_HAND_RPM];

            right_hand.position = machine_msg.Data[(int)MachineIndex.RIGHT_HAND_POS];
            right_hand.rpm = machine_msg.Data[(int)MachineIndex.RIGHT_HAND_RPM];
        }

        public std_msgs.msg.Float64MultiArray GetWheelSensorMsg()
        {
            var msg = new std_msgs.msg.Float64MultiArray();

            double[] data = new double[19];

            data[(int)SwerveIndex.FL_2006_POS] =  fl_unit.m2006_position;
            data[(int)SwerveIndex.FL_2006_RPM] =  fl_unit.m2006_rpm;
            data[(int)SwerveIndex.FL_3508_POS] =  fl_unit.m3508_position;
            data[(int)SwerveIndex.FL_3508_RPM] =  fl_unit.m3508_rpm;

            data[(int)SwerveIndex.RL_2006_POS] =  rl_unit.m2006_position;
            data[(int)SwerveIndex.RL_2006_RPM] =  rl_unit.m2006_rpm;
            data[(int)SwerveIndex.RL_3508_POS] =  rl_unit.m3508_position;
            data[(int)SwerveIndex.RL_3508_RPM] =  rl_unit.m3508_rpm;

            data[(int)SwerveIndex.FR_2006_POS] =  fr_unit.m2006_position;
            data[(int)SwerveIndex.FR_2006_RPM] =  fr_unit.m2006_rpm;
            data[(int)SwerveIndex.FR_3508_POS] =  fr_unit.m3508_position;
            data[(int)SwerveIndex.FR_3508_RPM] =  fr_unit.m3508_rpm;

            data[(int)SwerveIndex.RR_2006_POS] =  rr_unit.m2006_position;
            data[(int)SwerveIndex.RR_2006_RPM] =  rr_unit.m2006_rpm;
            data[(int)SwerveIndex.RR_3508_POS] =  rr_unit.m3508_position;
            data[(int)SwerveIndex.RR_3508_RPM] =  rr_unit.m3508_rpm;

            data[(int)SwerveIndex.LOCK_POS] = lock_motor.position;
            data[(int)SwerveIndex.LOCK_RPM] = lock_motor.rpm;

            data[(int)SwerveIndex.EXTEND_LENGTH] = extend_length;

            msg.Data = data;
            return msg;           
        }

        public std_msgs.msg.Float64MultiArray GetMachineSensorMsg()
        {
            var msg = new std_msgs.msg.Float64MultiArray();

            double[] data = new double[8];

            data[(int)MachineIndex.LEFT_HAND_POS] =  left_hand.position;
            data[(int)MachineIndex.LEFT_HAND_RPM] =  left_hand.rpm;
            data[(int)MachineIndex.LEFT_UPDOWN_POS] =  left_updown.position;
            data[(int)MachineIndex.LEFT_UPDOWN_RPM] =  left_updown.rpm;

            data[(int)MachineIndex.RIGHT_HAND_POS] =  right_hand.position;
            data[(int)MachineIndex.RIGHT_HAND_RPM] =  right_hand.rpm;
            data[(int)MachineIndex.RIGHT_UPDOWN_POS] =  right_updown.position;
            data[(int)MachineIndex.RIGHT_UPDOWN_RPM] =  right_updown.rpm;

            msg.Data = data;
            return msg;           
        }
    }
}