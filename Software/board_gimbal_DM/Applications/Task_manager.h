#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__


#ifdef __cplusplus
extern "C" {
#endif
#include "SBUS.h"
#include "board_comm.h"
extern void Task_init();
extern void boardCommunicateTask();
extern void SBUS_Task();
#ifdef __cplusplus
}
#endif

#endif