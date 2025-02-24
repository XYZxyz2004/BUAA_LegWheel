
#include "board_comm.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
BoardComm board_comm;
/* Private function prototypes -----------------------------------------------*/

void BoardCommInit(BoardComm *_board_comm,FDCAN_HandleTypeDef* _phcan, uint16_t _id){
_board_comm->boardfdcan=_phcan;
	_board_comm->boardid=_id;
	_board_comm->rece_len=0x08;
	_board_comm->send_len=0x08;
	
	_board_comm->send_.speed=0;
	_board_comm->send_.w_speed=0;
	_board_comm->send_.len=0;
	_board_comm->send_.jump_flag=0;
	_board_comm->send_.long_len_flag=0;
	_board_comm->send_.short_len_flag=0;
	_board_comm->send_.change_len_flag=0;
	
	_board_comm->rece_.pitch=0;
	_board_comm->rece_.reserve=0;
	_board_comm->rece_.roll=0;
	_board_comm->rece_.yaw=0;
}

void BoardCommSend(BoardComm *_board_comm) {
   _board_comm->senddata[0] = (uint8_t)_board_comm->send_.speed;
   _board_comm->senddata[1] = (uint8_t)_board_comm->send_.w_speed;
   _board_comm->senddata[2] = (uint8_t)_board_comm->send_.len;
   _board_comm->senddata[3] = 0;
   _board_comm->senddata[4] = 0;
   _board_comm->senddata[5] = 0;
   _board_comm->senddata[6] = 0;
   _board_comm->senddata[7] =(uint8_t)(_board_comm->send_.jump_flag<<3|_board_comm->send_.long_len_flag<<2|_board_comm->send_.short_len_flag<<1|_board_comm->send_.change_len_flag);
	Can_Fun.fdcanx_send_data(_board_comm->boardfdcan,_board_comm->boardid,_board_comm->senddata, _board_comm->send_len);
}

void BoardCommReceive(BoardComm *_board_comm,FDCan_Export_Data_t RxMessage) {
		_board_comm->rece_.pitch= 0;
    _board_comm->rece_.yaw  = 0;
    _board_comm->rece_.roll = 0;

}

