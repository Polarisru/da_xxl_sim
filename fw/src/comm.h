#ifndef COMM_H
#define COMM_H

#include "defines.h"

void COMM_CAN_Periodic_Task(void *pParameters);
void COMM_CAN_Task(void *pParameters);
void COMM_RS485_Task(void *pParameters);

#endif
