#ifndef  _IDLENEW_H_

#define  _IDLENEW_H_  

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

extern uint8_t buffer[255];
extern uint16_t lastRemote[13];
extern uint16_t remotedata[13];
extern fp32 vx;
extern fp32 vy;
extern fp32 wz;
extern osThreadId_t myTaskAssignHandle;

void Usart_Receive_Data(UART_HandleTypeDef *huart);
void Data_Processing(uint8_t *uartBuffer, uint16_t *data);
#endif
