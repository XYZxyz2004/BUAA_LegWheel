
#include "M6020_Motor.h"
#include "BSP_fdcan.h"
//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
M6020s_t M6020s_Yaw;                                    //ID为1
uint8_t M6020_Data_Tx[8];

void M6020_Enable(void);
void M6020_setVoltage(void);
void M6020_getInfo(FDCan_Export_Data_t RxMessage,M6020s_t *m6020);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);

M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit


/**
  * @brief  M6020pid设置
  * @param  None
  * @retval None
  */
void M6020_Enable(void)
{	
	Position_PIDInit(&M6020s_Yaw.Aim_position_PID, 1300.0f, 70.0f, 900.0f, 300.0f, 30000, 10000);
	Position_PIDInit(&M6020s_Yaw.position_PID, 0.035f, 0.0f, 0.01f, 0.0f, 30000, 10000);
}


/**
  * @brief  设置M6020电机电压（id号为1~4）
  * @param  uqx (x:1~4) 对应id号电机的电压值，范围 -30000~0~30000
  * @retval None
  */

 void M6020_setVoltage(void)
 {

 	M6020_Data_Tx[0] = M6020s_Yaw.outCurrent >> 8;
 	M6020_Data_Tx[1] = M6020s_Yaw.outCurrent;
 	M6020_Data_Tx[2] = 0;
 	M6020_Data_Tx[3] = 0;
 	M6020_Data_Tx[4] = 0;
 	M6020_Data_Tx[5] = 0;
 	M6020_Data_Tx[6] = 0;
 	M6020_Data_Tx[7] = 0;
	Can_Fun.fdcanx_send_data(&hfdcan1,M6020_SENDID, M6020_Data_Tx, 8);
 }

/**
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(FDCan_Export_Data_t RxMessage,M6020s_t *m6020)
{


    //解包数据，数据格式详见C620电调说明书P33
   m6020->lastAngle =m6020->realAngle;
    m6020->realAngle = (uint16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    m6020->realSpeed = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    m6020->realCurrent = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    m6020->temperture = RxMessage.FDCANx_Export_RxMessage[6];

    if (m6020->realAngle - m6020->lastAngle < -6500)
    {
       m6020->turnCount++;
    }

    if (m6020->lastAngle -m6020->realAngle < -6500)
    {
        m6020->turnCount--;
    }

  m6020->totalAngle =m6020->realAngle + (8192 * m6020->turnCount);
    //帧率统计，数据更新标志位
    m6020->InfoUpdateFrame++;
   m6020->InfoUpdateFlag = 1;

}

/*
*@brief  设定M6020电机的目标角度
* @param  motorName 	电机名字 @ref M6623Name_e
*			angle		电流值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
* @retval None
* */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/*************************************
* Method:    M6020_OverflowReset
* Returns:   void
* 说明：调运此函数以解决totalAngle 等溢出的问题。
************************************/
void M6020_Reset(M6020s_t *m6020)
{
    //解包数据，数据格式详见C620电调说明书P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}

