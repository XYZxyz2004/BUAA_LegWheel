#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "lqr.h"
#include "pid.h"
#include "vmc.h"

#include "kalman_filter.h"
#include "unitree.h"
#include "M1505B_Motor.h"

#define RAD_2_DEGREE 57.2957795f     // 180/pi
#define DEGREE_2_RAD 0.01745329252f  // pi/180
#define VEL_PROCESS_NOISE 25 
#define VEL_MEASURE_NOISE 800 
#define ACC_PROCESS_NOISE 2000  
#define ACC_MEASURE_NOISE 0.01  


class balance_Chassis
{
	public:
		Unitree_Motor joint,lf_joint_, lb_joint_, rf_joint_, rb_joint_;
		Vmc left_leg_, right_leg_;
		void motor_test_init();
	  float MotorAngle();
		void LegCalc();
//		void SetTargetYaw(float _pos) { target_yaw_ = _pos; }
		void MotorInit();
		void PidInit();
		void Controller();
		void TorCalc();
		void LQRCalc();
		void LegLenCalc();
		void SpeedCalc();
		void SynthesizeMotion();
		void Jump();
		void SpeedEstInit();
		void SetMotorTor();
		void StopMotor();
		void SetLegLen();
		void SetFollow();
		void SetState();
		void SetSpd();
	
	private:
		KalmanFilter_t kf,kf_l,kf_r;
		Lqr lqr_left_, lqr_right_;
		Pid left_leg_len_, right_leg_len_, anti_crash_, yaw_pos_,
				yaw_speed_;
		float left_leg_F_, right_leg_F_, roll_comp;
		float l_wheel_T_, r_wheel_T_, left_leg_T_, right_leg_T_;
		float set_spd_, target_speed_, target_yaw_, target_dist_, vel_, dist_, acc_,vel_l,vel_r;
		float vel_m, left_v_body_, right_v_body_, left_w_wheel_, right_w_wheel_;
		float jump_start_time_, jump_now_time_;
		uint32_t dwt_cnt_controller_, dwt_cnt_observer;
		bool jump_state_ = false, last_jump_state_ = false;
		float controller_dt_, observer_dt_;
		int16_t ang_yaw_;
		uint8_t side_flag_;
		
};


extern balance_Chassis chassis;
#endif