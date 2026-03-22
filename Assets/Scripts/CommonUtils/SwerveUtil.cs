using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace HKDT.Kinematics
{
    /// <summary>
    /// ４輪独立ステアリングのユニットの角度と速度を表す構造体
    /// </summary>
    public struct SwerveUnit
    {
        public float angle;
        public float speed;
    }

    /// <summary>
    /// ４輪独立ステアリングを制御するためのクラス
    /// </summary>
    public class Swerve
    {
        private SwerveUnit fl_unit;
        private SwerveUnit fr_unit;
        private SwerveUnit rl_unit;
        private SwerveUnit rr_unit;

        private SwerveUnit previous_fl_unit;
        private SwerveUnit previous_fr_unit;
        private SwerveUnit previous_rl_unit;
        private SwerveUnit previous_rr_unit;

        private float trackWidth; // 左右の車輪の距離
        private float wheelBase; // 前後の車輪の距離

        private float maxSpeed; // モーターの最大速度

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="yoko">左右の車輪の距離</param>
        /// <param name="tate">前後の車輪の距離</param>
        /// <param name="max_speed">モーターの最大速度</param>
        public Swerve(float yoko, float tate, float max_speed)
        {
            trackWidth = yoko;
            wheelBase = tate;
            maxSpeed = max_speed;
        }

        /// <summary>
        /// 前回のユニットの角度と速度を保存する。
        /// </summary>
        /// <param name="fl"></param>
        /// <param name="fr"></param>
        /// <param name="rl"></param>
        /// <param name="rr"></param>
        public void SetPreviousState(SwerveUnit fl, SwerveUnit fr, SwerveUnit rl, SwerveUnit rr)
        {
            previous_fl_unit = fl;
            previous_fr_unit = fr;
            previous_rl_unit = rl;
            previous_rr_unit = rr;
        }

        /// <summary>
        /// 目標のベクトルと回転量から、各ユニットの角度と速度を計算する。
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="rotation"></param>
        public void Calculate(float x, float y, float rotation)
        {
            // 目標のベクトルと回転量が0の場合は、前回のユニットの角度を維持し、速度を0にする。
            if(x == 0.0f && y == 0.0f && rotation == 0.0f)
            {
                fl_unit.angle = previous_fl_unit.angle;
                fr_unit.angle = previous_fr_unit.angle;
                rl_unit.angle = previous_rl_unit.angle;
                rr_unit.angle = previous_rr_unit.angle;

                fl_unit.speed = 0.0f;
                fr_unit.speed = 0.0f;
                rl_unit.speed = 0.0f;
                rr_unit.speed = 0.0f;

                return;
            }

            // 目標のベクトルを作成
            Vector2 target_vector = new Vector2(x, y);
            
            // 各ユニットのロボット中心からの位置を計算
            Vector2 front_left_pos = new Vector2(trackWidth/-2.0f, wheelBase/-2.0f);
            Vector2 front_right_pos = new Vector2(trackWidth/2.0f, wheelBase/-2.0f);
            Vector2 rear_left_pos = new Vector2(trackWidth/-2.0f, wheelBase/2.0f);
            Vector2 rear_right_pos = new Vector2(trackWidth/2.0f, wheelBase/2.0f);

            // 逆運動学に基づいて、各ユニットの目標のベクトルを計算
            Vector2 front_left_vector = new Vector2
            {
                x = target_vector.x - rotation * front_left_pos.y,
                y = target_vector.y - rotation * front_left_pos.x
            };

            Vector2 front_right_vector = new Vector2
            {
                x = target_vector.x - rotation * front_right_pos.y,
                y = target_vector.y - rotation * front_right_pos.x
            };

            Vector2 rear_left_vector = new Vector2
            {
                x = target_vector.x - rotation * rear_left_pos.y,
                y = target_vector.y - rotation * rear_left_pos.x
            };

            Vector2 rear_right_vector = new Vector2
            {
                x = target_vector.x - rotation * rear_right_pos.y,
                y = target_vector.y - rotation * rear_right_pos.x
            };

            // 各ユニットのベクトルの大きさを比べて最大値を探す
            float front_max = Math.Max(front_left_vector.magnitude, front_right_vector.magnitude);
            float rear_max = Math.Max(rear_left_vector.magnitude, rear_right_vector.magnitude);
            float max = Math.Max(front_max, rear_max);

            // 最大値が最大速度を超える場合は、全てのユニットのベクトルを最大速度に合わせて縮小する
            if(max > maxSpeed)
            {
                front_left_vector *= maxSpeed / max;
                front_right_vector *= maxSpeed / max;
                rear_left_vector *= maxSpeed / max;
                rear_right_vector *= maxSpeed / max;
            }

            // 目標のベクトルと前回のユニットの角度から、目標のユニットの角度と速度を計算する。
            CalcUnit(ref fl_unit, front_left_vector, previous_fl_unit);
            CalcUnit(ref fr_unit, front_right_vector, previous_fr_unit);
            CalcUnit(ref rl_unit, rear_left_vector, previous_rl_unit);
            CalcUnit(ref rr_unit, rear_right_vector, previous_rr_unit);
        }

        /// <summary>
        /// 左前ユニットの角度と速度を取得する。
        /// </summary>
        /// <returns></returns>
        public SwerveUnit GetFLUnit()
        {
            return fl_unit;
        }

        /// <summary>
        /// 右前ユニットの角度と速度を取得する。
        /// </summary>
        /// <returns></returns>
        public SwerveUnit GetFRUnit()
        {
            return fr_unit;
        }
        
        /// <summary>
        /// 左後ユニットの角度と速度を取得する。
        /// </summary>
        /// <returns></returns>
        public SwerveUnit GetRLUnit()
        {
            return rl_unit;
        }

        /// <summary>
        /// 右後ユニットの角度と速度を取得する。
        /// </summary>
        /// <returns></returns>
        public SwerveUnit GetRRUnit()
        {
            return rr_unit;
        }

        /// <summary>
        /// 目標のベクトルと前回のユニットの角度から、目標のユニットの角度と速度を計算する。
        /// </summary>
        /// <param name="target"></param>
        /// <param name="vector"></param>
        /// <param name="previous_data"></param>
        private void CalcUnit(ref SwerveUnit target, Vector2 vector, SwerveUnit previous_data)
        {
            // 目標の角度を計算
            float target_angle = Mathf.Atan2(vector.y, vector.x);
            // 目標の角度の逆向きも計算
            float inversed_target_angle = target_angle + Mathf.PI;

            // 逆向きの角度が2πを超える場合は、0～2πの範囲に収める
            if(inversed_target_angle > 2.0f * Mathf.PI)
            {
                inversed_target_angle -= 2.0f * Mathf.PI;
            }

            // 目標の角度と逆向きの角度のどちらが前回の角度に近いかを判断
            float difference_angle = Mathf.Abs(target_angle - previous_data.angle);
            float inversed_difference_angle = Mathf.Abs(inversed_target_angle - previous_data.angle);

            
            if(difference_angle > inversed_difference_angle)/* 逆向きが近い */
            {
                // 目標の角度を逆向きにする
                target.angle = inversed_target_angle;

                // 目標角度との差が90度を超える場合は、速度を0にする。
                if(inversed_difference_angle > Mathf.PI * 0.5f)
                {
                    target.speed = 0.0f;
                }
                else
                {
                    // 角度が逆向きになった分、速度も逆向きにする
                    target.speed = -vector.magnitude;
                }
            }
            else
            {
                // 目標の角度をそのまま使用
                target.angle = target_angle;

                // 目標角度との差が90度を超える場合は、速度を0にする。
                if(difference_angle > Mathf.PI * 0.5f)
                {
                    target.speed = 0.0f;
                }
                else
                {
                    // 角度がそのままの場合は、速度もそのまま使用
                    target.speed = vector.magnitude;
                }
            }
        }
    }
}