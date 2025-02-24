/**
 * @file M1505B_Motor.c
 * @author Cyx
 * @brief
 * @version 0.1
 * @date 2024-10-13
 *
 * @copyright
 *
 */
#include "M1505B_Motor.h"
#include <stdio.h>
static int16_t FloatDeadband(int16_t _value,int16_t _min_value, int16_t _max_value)
{
    if (_value < _max_value && _value > _min_value) {
        _value = 0;
    }
    return _value;
}

M1505B_t M1505B_Array[TotalNum] = {
    [Wheel_L] 		  = {0},
    [Wheel_R] 		  = {0},
};

float test2;
/*接收数据数组*/
uint8_t M1505B_Data_Rx[8];
/*发送数据数组*/
uint8_t M1505B_Data_Tx[8];

/*协议数据*/
uint8_t SetLID_M1505B[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t SetRID_M1505B[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
M1505B_mode Disable_M1505B[8] = {Disable , Disable, Disable, Disable, Disable, Disable, Disable, Disable};
M1505B_mode Enable_M1505B[8] = {Current, Current, Current, Current, Current, Current, Current, Current};

void M1505B_Enable(void);
void M1505B_Disable(void);
void M1505B_SetIDL(void);
void M1505B_SetIDR(void);
void M1505B_setCurrent(void);
void	M1505B_setSeq(void);
void M1505B_getInfo(FDCan_Export_Data_t RxMessage);
void	M1505B_SetTor(void);

M1505B_FUN_t M1505B_FUN = M1505B_FunGroundInit;
#undef M1505B_FunGroundInit

/**
  * @brief  配置电流环
  * @param  None
  * @retval None
  */
void M1505B_Enable(void)
{	TPID_Init(&M1505B_Array[0].TPID, 0.1, 0, 0.05, 0.08, 100, 1000, 0);
	TPID_Init(&M1505B_Array[1].TPID, 0.1, 0, 0.05, 0.08, 100, 1000, 0);
	uint8_t getenable=0;
	getenable=Can_Fun.fdcanx_send_data(&hfdcan1, Mode_ID, Enable_M1505B, 8);
}

/**
  * @brief  失能电机
  * @param  None
  * @retval None
  */
void M1505B_Disable(void)
{
	Can_Fun.fdcanx_send_data(&hfdcan1, Mode_ID, Disable_M1505B, 8);
}

/**
  * @brief  左侧ID修改（先失能）
  * @param  None
  * @retval None
  */
void M1505B_SetIDL(void)
{
	Can_Fun.fdcanx_send_data(&hfdcan1, Set_ID, SetLID_M1505B, 8);
}

/**
  * @brief  右侧ID修改（先失能）
  * @param  None
  * @retval None
  */
void M1505B_SetIDR(void)
{
	Can_Fun.fdcanx_send_data(&hfdcan1, Set_ID, SetRID_M1505B, 8);
}


/**
  * @brief  设置M1505B电机电流值（id号为1~4）
  * @param  CtrlDatax (x:1~4) 对应id号电机的电流值，范围-16383~0~16383
  * @retval None
  */
void M1505B_setCurrent(void)
{
		uint8_t data[8];
    data[0] = M1505B_Array[0].sendCurrent >> 8;
    data[1] = M1505B_Array[0].sendCurrent;
////    data[2] = M1505B_Array[1].targetCurrent >> 8;
////    data[3] = M1505B_Array[1].targetCurrent;
////    data[4] = 0;
////    data[5] = 0;
////    data[6] = 0;
////    data[7] = 0;
////    data[0] = 0x03;
////    data[1] = 0x21;
    data[2] = M1505B_Array[1].sendCurrent >> 8;
    data[3] = M1505B_Array[1].sendCurrent ;
//	    data[0] = 0;
//    data[1] = 0;
//    data[2] = 0;
//    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
	Can_Fun.fdcanx_send_data(&hfdcan1, Send_ID, data, 8);
}
void M1505B_setSeq(void)
{
	uint8_t data[8];
    data[0] = M1505B_Array[0].feedbackSeq;
    data[1] = M1505B_Array[1].feedbackSeq;
//    data[2] = M1505B_Array[1].targetCurrent >> 8;
//    data[3] = M1505B_Array[1].targetCurrent;
//    data[4] = 0;
//    data[5] = 0;
//    data[6] = 0;
//    data[7] = 0;
//    data[0] = 0x03;
//    data[1] = 0x21;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
	Can_Fun.fdcanx_send_data(&hfdcan1, Seq_ID, data, 8);
}
/**
  * @brief  从CAN报文中获取M1505B电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M1505B_getInfo(FDCan_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.fdcan_RxHeader.Identifier- M1505B_READID_START);
    //解包数据，数据格式见手册
    M1505B_Array[StdId].receiveSpeed= (int16_t)((RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]));
    M1505B_Array[StdId].receiveTorqueI = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
    M1505B_Array[StdId].circleAngle = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    M1505B_Array[StdId].error = RxMessage.FDCANx_Export_RxMessage[6];
    M1505B_Array[StdId].mode = RxMessage.FDCANx_Export_RxMessage[7];
		 M1505B_Array[StdId].realSpeed=((float)M1505B_Array[StdId].receiveSpeed)/10*0.95+0.05* M1505B_Array[StdId].realSpeed;
	 M1505B_Array[StdId].realTorqueI=((float) M1505B_Array[StdId].receiveTorqueI)*55/32767*0.2+0.8* M1505B_Array[StdId].realTorqueI;
	
}
void M1505B_SetTor(void)
{
	uint32_t StdId;
	for(StdId=0; StdId<=1; StdId++)
	{
		M1505B_Array[StdId].targetTorqueI=(float)(M1505B_Array[StdId].targetTorque*12.5/5.8);
		 M1505B_Array[StdId].targetCurrent=	M1505B_Array[StdId].targetTorqueI+T_Update(&M1505B_Array[StdId].TPID, M1505B_Array[StdId].realTorqueI, M1505B_Array[StdId].targetTorqueI);
		 M1505B_Array[StdId].sendCurrent= (int16_t)(M1505B_Array[StdId].targetCurrent/55*32767);
		
	}
}
