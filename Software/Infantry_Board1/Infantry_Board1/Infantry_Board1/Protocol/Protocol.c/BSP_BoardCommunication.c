/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)
 * @brief
 * @version 1.0
 * @date 2023-9-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "BSP_BoardCommunication.h"
#include "queue.h"

ControlMessge ControlMes;

void Board1_To_2(void);
void Board1_getGimbalInfo(Fdcan_Export_Data_t RxMessage);

Board1_FUN_t Board1_FUN = Board1_FunGroundInit;

// /**
//  * @brief 十六进制转float
//  */
//static float R2float(uint8_t *p) {float r; memcpy(&r, p, 4); return r;}

//此函数用来按照报文规则生成数据并发送。
void Board1_To_2(void)
{
    uint8_t data[8] = {0};
    uint8_t data2[8] = {0};
    //打包数据
    data[0] = ControlMes.x_velocity >> 8;
    data[1] = ControlMes.x_velocity ;
    data[2] = ControlMes.y_velocity >> 8;
    data[3] = ControlMes.y_velocity ;
    data[4] = ControlMes.z_rotation_velocity >> 8;
    data[5] = ControlMes.z_rotation_velocity;
    data[6] = ControlMes.yaw_velocity >> 8;
    data[7] = ControlMes.yaw_velocity ;
    //数据发送
    Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_CHASSIS, data);

    data2[0] = ControlMes.yaw_position >> 8;
    data2[1] = ControlMes.yaw_position;
    data2[2] = ControlMes.shoot_Speed;
    data2[3] = ControlMes.fric_Flag;
    data2[4] = ControlMes.AutoAimFlag ;
    data2[5] = ControlMes.change_Flag ;
    data2[6] = ControlMes.modelFlag;
		data2[7] = ControlMes.jump_Flag;
    Fdcan_Fun.FDCAN_SendData(FDCAN_SendHandle, &hfdcan2, FDCAN_STANDARD_ID, FDCAN_ID_GIMBAL , data2);
}

void Board1_getGimbalInfo(Fdcan_Export_Data_t RxMessage)
{
    ControlMes.yaw_realAngle = (int16_t)(RxMessage.FDCANx_Export_RxMessage[0] << 8 | RxMessage.FDCANx_Export_RxMessage[1]);
    ControlMes.Gimbal_Chassis_Pitch_Angle = (int16_t)(RxMessage.FDCANx_Export_RxMessage[2] << 8 | RxMessage.FDCANx_Export_RxMessage[3]);
	  //ControlMes.heat_remain = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    ControlMes.Speed_Bullet = (int16_t)(RxMessage.FDCANx_Export_RxMessage[4] << 8 | RxMessage.FDCANx_Export_RxMessage[5]);
    ControlMes.Speed_Bullet /= 1000;
    ControlMes.tnndcolor = RxMessage.FDCANx_Export_RxMessage[6];
}
