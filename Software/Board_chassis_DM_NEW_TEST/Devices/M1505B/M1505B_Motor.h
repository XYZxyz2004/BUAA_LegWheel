/**
 * @file M1505B_Motor.h
 * @author Cyx
 * @brief
 * @version 0.1
 * @date 2024-10-13
 *
 * @copyright
 *
 */
#ifndef __M1505B_MOTOR_H
#define __M1505B_MOTOR_H

#include "fdcan.h"
#include "BSP_fdcan.h"
#include "tpid.h"
#include "string.h"
#include <stdbool.h>
#include <stdint.h>

#define M1505B_L_ID 0x97   //左
#define M1505B_R_ID 0x98   //右
#define M1505B_READID_START 0x97
#define M1505B_READID_END 0x98
#define Send_ID 0x32   //发送ID
#define Set_ID 0x108   //ID设置
#define Mode_ID 0x105   //模式设置
#define Seq_ID 0x106 //反馈间隔设置
	/**
  * @brief  用以区别1505电机数组各个元素的归属
  * @param  
  *			Wheel_L			左轮
  *			Wheel_R			右轮
  */
typedef enum
{
	Wheel_L = 0,
	Wheel_R,
	TotalNum,
}M1505B_MotorName;

typedef enum
{
	Voltage = 0x00,
	Current = 0x01,
	Speed = 0x02,
	Disable = 0x09,
	Enable = 0x0A,
}M1505B_mode;

typedef enum
{
	None = 0x00,
	LowVoltage20 = 0x01,
	LowVoltage18 = 0x02,
	HighVoltage = 0x03,
	HighCurrent = 0x0A,
	HighTemperature80 = 0x20,
	HighTemperature120 = 0x1F,
	Resistance = 0x29,
	SensorPosError = 0x2A,
	SensorPosSignal = 0x2B,
	SensorTem = 0x2D,
	Timeout = 0x3C,
	Block = 0x62,
}M1505B_error;

typedef struct
{   float targetTorqueI;	//目标转矩得到的目标电流
	int16_t receiveTorqueI;
	int16_t receiveSpeed;
	  float realTorqueI;		//读回转矩电流
		float targetTorque;//目标转矩
    float realSpeed;   	//读回速度
    float targetCurrent;		//设置电流发送值
		int16_t sendCurrent;
    uint16_t circleAngle; //读回位置
		uint8_t feedbackSeq;//读回反馈时间间隔
    M1505B_error error;   //故障模式
		M1505B_mode mode;     //电机模式
		struct Struct_PID_Manage_Object TPID;
} M1505B_t;

#define M1505B_FunGroundInit        \
    {                               \
        &M1505B_Enable,             \
        &M1505B_Disable,           	\
				&M1505B_SetIDL,							\
				&M1505B_SetIDR,							\
        &M1505B_setCurrent, \
				&M1505B_setSeq,			\
        &M1505B_getInfo,         \
				&M1505B_SetTor,			\
    }


typedef struct
{
	void (*M1505B_Enable)(void);
	void (*M1505B_Disable)(void);
	void (*M1505B_SetIDL)(void);
	void (*M1505B_SetIDR)(void);
	void (*M1505B_setCurrent)(void);
	void (*M1505B_setSeq)(void);
	void (*M1505B_getInfo)(FDCan_Export_Data_t RxMessage);
	void (*M1505B_SetTor)(void);
} M1505B_FUN_t;

extern M1505B_FUN_t M1505B_FUN;
extern M1505B_t M1505B_Array[TotalNum] ;


#endif