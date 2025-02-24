#include "Task_manager.h"
#include "SBUS.h"
#include "board_comm.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
/**DEBUG**/

/**
 * @brief 遥控器指令接收
 * @param 
 * @param 
 * @param
 */

void boardCommunicateTask()
{
	board_comm.send_.speed=(int8_t)((SBUS.Ch2-992.0)/671.0*127.0);
	board_comm.send_.w_speed=(int8_t)((SBUS.Ch1-992.0)/671.0*127.0);
	board_comm.send_.len=(int8_t)((SBUS.Ch3-992.0)/671.0*127.0);
	board_comm.send_.short_len_flag =(SBUS.Ch8==321?1:0);
	board_comm.send_.long_len_flag 	=(SBUS.Ch8==992?1:0);
	board_comm.send_.jump_flag =(SBUS.Ch8==1663?1:0);
	BoardCommSend(&board_comm);
}
void SBUS_Task()
{
	SBUS_Read();
}	
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
	if(huart->Instance == USART2 )
    {
			memcpy(SBUS_RXBuffer,SBUS_Rx_Data,sizeof(SBUS_Rx_Data));
			if(Control_Cheak()==1)
			{
			SBUS_RX_Finish=1;//已接收完一包数据
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,SBUS_Rx_Data,sizeof(SBUS_Rx_Data));//准备接收下一包数据	
			}
    }
}