#include "Chassis.h"
#include "ins.h"
#include "bsp_dwt.h"
#include "usart.h"
/*************const***************/
const float k_gravity_comp =5.068 * 9.8f;
const float k_roll_extra_comp_p = 100.0f;
const float k_wheel_radius = 0.091f;

const float k_phi1_bias =0.8956f + 3.141f;
const float k_phi4_bias = -0.8956;

const float k_lf_joint_bias =5.94;
const float k_lb_joint_bias =0.43;
const float k_rf_joint_bias =5.94;
const float k_rb_joint_bias =4.6844;

const float k_jump_force = 220.0f;
const float k_jump_time = 0.2f;
const float k_retract_force = -120.0f;
const float k_retract_time = 0.1f;

static float target_yaw;

float test_data = 0;
float input_angel = 0;
/***********************************/

balance_Chassis chassis;
/**
 * @brief 腿部电机初始化
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::MotorInit()
{
	M1505B_Array[0].feedbackSeq=0x01;
	M1505B_Array[1].feedbackSeq=0x01;
	M1505B_FUN.M1505B_Enable();
	M1505B_FUN.M1505B_setSeq();
	 lf_joint_.Init(&huart2, 0x00, 10, k_lf_joint_bias);
	lb_joint_.Init(&huart2, 0x02, 10, k_lb_joint_bias);
  rf_joint_.Init(&huart3, 0x00, 10, k_rf_joint_bias);
  rb_joint_.Init(&huart3, 0x02, 10, k_rb_joint_bias);
	//中断回调方式接收本末电机数据
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*)&temprecvl,sizeof(temprecvl));
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t*)&temprecvr,sizeof(temprecvr));
}
/**
 * @brief 所有所需PID初始化
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::PidInit() {
  left_leg_len_.Init(600.0f, 0.0f, 50.0f, 100.0f, 0.001f);
  right_leg_len_.Init(600.0f, 0.0f, 50.0f,100.0f, 0.001f);
  anti_crash_.Init(6.0f, 0.0f, 1.0f, 10.0f, 0.001f);
  yaw_pos_.Init(8.0f, 0.0f, 0.0f, 5.0f, 0.001f);
  yaw_speed_.Init(3.05f, 0.0f, 0.50f, 5.0f, 0.0f);
  left_leg_len_.Inprovement(PID_CHANGING_INTEGRATION_RATE |
                                PID_TRAPEZOID_INTEGRAL | PID_DERIVATIVE_FILTER |
                                PID_DERIVATIVE_ON_MEASUREMENT,
                            0.0f, 0.0f, 0.01f, 0.01f, 0.08f);
  right_leg_len_.Inprovement(
      PID_CHANGING_INTEGRATION_RATE | PID_TRAPEZOID_INTEGRAL |
          PID_DERIVATIVE_FILTER | PID_DERIVATIVE_ON_MEASUREMENT,
      0.0f, 0.0f, 0.01f, 0.01f, 0.08f);
  anti_crash_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_pos_.Inprovement(PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f);
  yaw_speed_.Inprovement(PID_DERIVATIVE_FILTER | PID_INTEGRAL_LIMIT, 2.0f, 0.0f,
                         0.0f, 0.0f, 0.05f);
}

/**
 * @brief 速度和加速度卡尔曼滤波观测器
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SpeedEstInit() {
  // 使用kf同时估计速度和加速度
  Kalman_Filter_Init(&kf, 2, 0, 2);
	 Kalman_Filter_Init(&kf_l, 2, 0, 2);
	 Kalman_Filter_Init(&kf_r, 2, 0, 2);
  float F[4] = {1, 0.001, 0, 1};//A矩阵由于d_t为0.001所以1行2列处为0.001
  float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};//状态变量过程噪声矩阵
  float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};//测量噪声矩阵
  float P[4] = {100000, 0, 0, 100000};//状态估计误差协方差矩阵
  float H[4] = {1, 0, 0, 1};
  memcpy(kf.F_data, F, sizeof(F));
  memcpy(kf.Q_data, Q, sizeof(Q));
  memcpy(kf.R_data, R, sizeof(R));
  memcpy(kf.P_data, P, sizeof(P));
  memcpy(kf.H_data, H, sizeof(H));
	memcpy(kf_l.F_data, F, sizeof(F));
  memcpy(kf_l.Q_data, Q, sizeof(Q));
  memcpy(kf_l.R_data, R, sizeof(R));
  memcpy(kf_l.P_data, P, sizeof(P));
  memcpy(kf_l.H_data, H, sizeof(H));
	memcpy(kf_r.F_data, F, sizeof(F));
  memcpy(kf_r.Q_data, Q, sizeof(Q));
  memcpy(kf_r.R_data, R, sizeof(R));
  memcpy(kf_r.P_data, P, sizeof(P));
  memcpy(kf_r.H_data, H, sizeof(H));
}

/**
 * @brief 腿部状态正运动学计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LegCalc() {
  left_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[2]);
  right_leg_.SetBodyData(INS.Pitch * DEGREE_2_RAD, INS.MotionAccel_n[2]);
  left_leg_.SetLegData(lb_joint_.GetAngle() + k_phi1_bias, lb_joint_.GetSpeed(),
                       lf_joint_.GetAngle() + k_phi4_bias, lf_joint_.GetSpeed(),
                       lb_joint_.GetTor(), lf_joint_.GetTor());
  right_leg_.SetLegData(
      -rb_joint_.GetAngle() + k_phi1_bias, -rb_joint_.GetSpeed(),
      -rf_joint_.GetAngle() + k_phi4_bias, -rf_joint_.GetSpeed(),
      -rb_joint_.GetTor(), -rf_joint_.GetTor());

  left_leg_.LegCalc();
  right_leg_.LegCalc();
  left_leg_.Jacobian();
  right_leg_.Jacobian();
  left_leg_.LegForceCalc();
  right_leg_.LegForceCalc();
}

/**
 * @brief LQR计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LQRCalc() {
  lqr_left_.SetSpeed(target_speed_);
  lqr_right_.SetSpeed(target_speed_);
lqr_left_.SetNowDist(target_dist_);
lqr_right_.SetNowDist(target_dist_);
	//避免纹波干扰
  if (fabsf(vel_) < 0.1f) {
    dist_ += vel_ * controller_dt_;
  } else {
    dist_ = 0.0f;
  }
  lqr_left_.SetData(dist_,  vel_l, INS.Pitch * DEGREE_2_RAD, INS.Gyro[0],
                    left_leg_.GetTheta(), left_leg_.GetDotTheta(),
                    left_leg_.GetLegLen(), left_leg_.GetForceNormal());
  lqr_right_.SetData(dist_,  vel_r, INS.Pitch * DEGREE_2_RAD, INS.Gyro[0],
                     right_leg_.GetTheta(), right_leg_.GetDotTheta(),
                     right_leg_.GetLegLen(), right_leg_.GetForceNormal());
  lqr_left_.Calc();
  lqr_right_.Calc();
}

/**
 * @brief 控制力矩计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::TorCalc() {
  left_leg_.SetTor(left_leg_F_, left_leg_T_);
  right_leg_.SetTor(right_leg_F_, right_leg_T_);
  left_leg_.TorCalc();
  right_leg_.TorCalc();
}

/**
 * @brief 腿长计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::LegLenCalc() {
  left_leg_len_.SetMeasure(left_leg_.GetLegLen());
  right_leg_len_.SetMeasure(right_leg_.GetLegLen());

  roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
  left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
  right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;
}


/**
 * @brief 运动控制量整合
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SynthesizeMotion() {
//  yaw_pos_.SetRef((ang_yaw_ / 8192.0f) * 2 * PI);
 // yaw_pos_.SetMeasure((target_yaw_ / 8192.0f) * 2 * PI);
	
//  if (board_comm.GetLeftRotate() && !board_comm.GetRightRotate()) {
//    yaw_speed_.SetRef(5.0f);
//  } else if (board_comm.GetRightRotate() && !board_comm.GetLeftRotate()) {
//    yaw_speed_.SetRef(-5.0f);
//  } else {
//    yaw_speed_.SetRef(yaw_pos_.Calculate());
//  }
	 yaw_speed_.SetRef(0);
  yaw_speed_.SetMeasure(INS.Gyro[2]);
  yaw_speed_.Calculate();

  if (left_leg_.GetForceNormal() < 10.0f) {
    l_wheel_T_ = lqr_left_.GetWheelTor();
  } else {
    l_wheel_T_ = lqr_left_.GetWheelTor() - yaw_speed_.GetOutput();
  }
  if (right_leg_.GetForceNormal() < 10.0f) {
    r_wheel_T_ = lqr_right_.GetWheelTor();
  } else {
    r_wheel_T_ = lqr_right_.GetWheelTor() + yaw_speed_.GetOutput();
  }
  anti_crash_.SetRef(0.0f);
  anti_crash_.SetMeasure(left_leg_.GetPhi0() - right_leg_.GetPhi0());
  anti_crash_.Calculate();
  left_leg_T_ = lqr_left_.GetLegTor() + anti_crash_.GetOutput();
  right_leg_T_ = lqr_right_.GetLegTor() - anti_crash_.GetOutput();
}


/**
 * @brief 主控制指令计算循环
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::Controller() {
  SetState();
  LegCalc();
  SpeedCalc();
  LQRCalc();
  SynthesizeMotion();
  LegLenCalc();
  TorCalc();
	if(INS.Yaw!=0&&lf_joint_.GetAngle()<1.7&&lf_joint_.GetAngle()>-0.15&&lb_joint_.GetAngle()<0.15&&lb_joint_.GetAngle()>-1.7&&rf_joint_.GetAngle()<0.15&&rf_joint_.GetAngle()>-1.7&&rb_joint_.GetAngle()<1.7&&rb_joint_.GetAngle()>-0.15)
	{SetMotorTor();
	}else
	{
		StopMotor();
	}
}

/**
 * @brief 力矩指令设置
 * @param 
 * @param 
 * @param
 */
// 电机力矩输入模式
void balance_Chassis::SetMotorTor() {
  lf_joint_.SetMotorT(left_leg_.GetT2());
  lb_joint_.SetMotorT(left_leg_.GetT1());
  rf_joint_.SetMotorT(-right_leg_.GetT2());
  rb_joint_.SetMotorT(-right_leg_.GetT1());
	M1505B_Array[0].targetTorque=l_wheel_T_;
	M1505B_Array[1].targetTorque=-r_wheel_T_;
	M1505B_FUN.M1505B_SetTor();
}
/**
 * @brief 力矩指令设置为0-急停
 * @param 
 * @param 
 * @param
 */
// 电机急停模式
void balance_Chassis::StopMotor() {
  lf_joint_.SetMotorT(0.0f);
  rf_joint_.SetMotorT(0.0f);
  lb_joint_.SetMotorT(0.0f);
  rb_joint_.SetMotorT(0.0f);
	M1505B_Array[0].targetTorque=0;
	M1505B_Array[1].targetTorque=0;
	M1505B_FUN.M1505B_SetTor();
	M1505B_Array[0].sendCurrent=0;
	M1505B_Array[1].sendCurrent=0;
}

/**
 * @brief 设置腿长
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetLegLen() {
//  if (fabsf(INS.Pitch) < 8.0f) {
//    if (board_comm.GetLongLenFlag() && !board_comm.GetShortLenFlag()) {
//      left_leg_len_.SetRef(0.3f);
//      right_leg_len_.SetRef(0.3f);
//    } else if (!board_comm.GetLongLenFlag() && board_comm.GetShortLenFlag()) {
//      left_leg_len_.SetRef(0.08f);
//      right_leg_len_.SetRef(0.08f);
//    } else {
//      left_leg_len_.SetRef(0.18f);
//      right_leg_len_.SetRef(0.18f);
//    }
//  } else {
//    left_leg_len_.SetRef(0.08f);
//    right_leg_len_.SetRef(0.08f);
//  }
	//设定保持腿长的PID目标值
	 left_leg_len_.SetRef(0.20f);
 right_leg_len_.SetRef(0.20f);
}

/**
 * @brief 设置YAW
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetFollow() {
//  if (fabsf(board_comm.GetYSpeed()) > 0.0f && side_flag_ == 1) {
//    side_flag_ = 0;
//  }

//  if (fabsf(board_comm.GetXSpeed()) > 0.0f && side_flag_ == 0) {
//    side_flag_ = 1;
//  }

//  if (side_flag_ == 0) {
//    if (arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f){
//      SetTargetYaw(6812);
//    } else {
//      SetTargetYaw(2716);
//    }
//  } else {
//    if (arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) >= 0.0f) {
//      SetTargetYaw(668);
//    } else {
//      SetTargetYaw(4764);
//    }
//  }

//  ang_yaw_ = yaw_motor_.GetEncode();
//  if (ang_yaw_ - target_yaw_ > 4096) {
//    ang_yaw_ -= 8192;
//  } else if (ang_yaw_ - target_yaw_ < -4096) {
//    ang_yaw_ += 8192;
//  }
}

/**
 * @brief 设置速度
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetSpd() {
//  if (board_comm.GetCapFlag()) {
//    set_spd_ = 2.5f;
//  } else {
//    set_spd_ = 1.5f;
//  }
}

/**
 * @brief 设置机器人状态
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SetState() {
  controller_dt_ = DWT_GetDeltaT(&dwt_cnt_controller_);

  SetLegLen();
  //SetFollow();
 // SetSpd();
//  float y_spd_ = +arm_cos_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
//                     board_comm.GetYSpeed() +
//                 arm_sin_f32(yaw_motor_.GetAngle() * DEGREE_2_RAD) *
//                     board_comm.GetXSpeed();

//  if (fabsf((y_spd_ / 2000.0f) * set_spd_ - target_speed_) / controller_dt_ <
//      5.0f) {
//    target_speed_ = (y_spd_ / 2000.0f) * set_spd_;
//  } else {
//    target_speed_ += Math::Sign((y_spd_ / 2000.0f) * set_spd_ - target_speed_) *
//                     4.f * controller_dt_;
//  }
//  if (board_comm.GetReadyFlag() == 0) {
//    lqr_left_.SetNowDist(0.0f);
//    lqr_right_.SetNowDist(0.0f);
//  }

//  if (board_comm.GetJumpFlag()) {
//    jump_state_ = true;
//  }
target_speed_=0;
target_dist_=0;

}

/**
 * @brief 跳跃
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::Jump() {
//  if (left_leg_.GetLegLen() > 0.12f && right_leg_.GetLegLen() > 0.12f &&
//      last_jump_state_ == false) {
//    left_leg_len_.SetRef(0.1f);
//    right_leg_len_.SetRef(0.1f);
//    roll_comp = k_roll_extra_comp_p * INS.Roll * DEGREE_2_RAD;
//    left_leg_F_ = left_leg_len_.Calculate() + k_gravity_comp - roll_comp;
//    right_leg_F_ = right_leg_len_.Calculate() + k_gravity_comp + roll_comp;
//    last_jump_state_ = false;
//    return;
//  } else {
//    if (jump_state_ == true && last_jump_state_ == false) {
//      jump_start_time_ = HAL_GetTick() / 1000.0f;
//    }

//    jump_now_time_ = HAL_GetTick() / 1000.0f;

//    if (fabs(jump_now_time_ - jump_start_time_) <= k_jump_time) {
//      left_leg_F_ = k_jump_force;
//      right_leg_F_ = k_jump_force;
//    }

//    jump_now_time_ = HAL_GetTick() / 1000.0f;
//    if ((jump_now_time_ - jump_start_time_ - k_jump_time) <= k_retract_time &&
//        (jump_now_time_ - jump_start_time_) > k_jump_time) {
//      left_leg_F_ = k_retract_force;
//      right_leg_F_ = k_retract_force;
//      l_wheel_T_ = 0.0f;
//      r_wheel_T_ = 0.0f;
//    }
//    if ((jump_now_time_ - jump_start_time_ - k_jump_time) > k_retract_time) {
//      jump_state_ = false;
//    }
//    last_jump_state_ = jump_state_;
//  }
}

/**
 * @brief 速度计算
 * @param 
 * @param 
 * @param
 */
void balance_Chassis::SpeedCalc() {
  left_w_wheel_ = M1505B_Array[0].realSpeed/60*2*3.14159+ left_leg_.GetPhi2Speed() - INS.Gyro[0];
  right_w_wheel_ =-M1505B_Array[1].realSpeed/60*2*3.14159+ right_leg_.GetPhi2Speed() - INS.Gyro[0];

  left_v_body_ = left_w_wheel_ * k_wheel_radius +
                 left_leg_.GetLegLen() * left_leg_.GetDotTheta()*arm_cos_f32(left_leg_.GetTheta()) +
                 left_leg_.GetLegSpeed() * arm_sin_f32(left_leg_.GetTheta());
  right_v_body_ = right_w_wheel_ * k_wheel_radius +
                  right_leg_.GetLegLen() * right_leg_.GetDotTheta()*arm_cos_f32(left_leg_.GetTheta()) +
                  right_leg_.GetLegSpeed() * arm_sin_f32(right_leg_.GetTheta());
  vel_m = (left_v_body_ + right_v_body_) / 2;
  if (left_leg_.GetForceNormal() < 10.0f &&
      right_leg_.GetForceNormal() < 10.0f) {
    vel_m = 0;
  }

  // 使用kf同时估计加速度和速度,滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = INS.MotionAccel_n[1];
  kf.F_data[1] = controller_dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf);
  vel_ = kf.xhat_data[0];
  acc_ = kf.xhat_data[1];
	
	 kf_l.MeasuredVector[0] = left_w_wheel_*k_wheel_radius ;
  kf_l.MeasuredVector[1] = INS.MotionAccel_n[1];
  kf_l.F_data[1] = controller_dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf_l);
vel_l = kf_l.xhat_data[0];

 kf_r.MeasuredVector[0] = right_w_wheel_*k_wheel_radius ;
  kf_r.MeasuredVector[1] = INS.MotionAccel_n[1];
  kf_r.F_data[1] = controller_dt_;  // 更新F矩阵
  Kalman_Filter_Update(&kf_r);
vel_r = kf_r.xhat_data[0];
}

