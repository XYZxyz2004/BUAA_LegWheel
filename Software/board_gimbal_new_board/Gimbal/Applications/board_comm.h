/**
 *******************************************************************************
 * @file      : board_comm.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "Bsp_fdcan.h"
#include "stdlib.h"
#include "string.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct {
  int8_t speed;
  int8_t w_speed;
	int8_t len;
  uint8_t jump_flag ;
  uint8_t long_len_flag ;
  uint8_t short_len_flag ;
	uint8_t change_len_flag ;
  uint8_t reserved ;
	
} send_pack;

typedef struct {
  int16_t pitch;
  int16_t yaw;
  int16_t roll;
  int16_t reserve;
} rece_pack;


typedef struct {
	send_pack send_;
  rece_pack rece_;
 	uint8_t rece_len;
	uint8_t send_len;
  FDCAN_HandleTypeDef *boardfdcan;
	uint32_t boardid;
	uint8_t senddata[8];
}BoardComm;

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
extern void BoardCommSend(BoardComm *_board_comm);
extern void BoardCommReceive(BoardComm *_board_comm,FDCan_Export_Data_t RxMessage);
extern void BoardCommInit(BoardComm *_board_comm,FDCAN_HandleTypeDef* _phcan, uint16_t _id);	

/* Exported function prototypes ----------------------------------------------*/

#endif /* __BOARD_COMM_H_ */
