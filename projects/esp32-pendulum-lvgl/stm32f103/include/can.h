#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

void CAN_Init_Loopback(void);
void CAN_Test_Send(void);

void CAN_Init_Normal(void);
void CAN_Send_Data(uint32_t id, uint8_t *data, uint8_t len);
uint8_t CAN_Receive_Data(uint32_t *id, uint8_t *data, uint8_t *len);

#endif
