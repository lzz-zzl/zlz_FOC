#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "Control.h"

extern char kv_buf[64];

extern osSemaphoreId_t uart_sem;

extern char Serial_RxPacket[100];
extern uint8_t Serial_RxFlag;
extern uint8_t temp_rxdata6[11];


extern uint16_t Serial_value[10];
extern uint8_t Serial_key[10];
extern float Serial_slider[5];
extern uint16_t Serial_joystick[2][2];

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
void Serial_Firewater(float *data, uint8_t n);
void Serial_JustFloat(float *data, uint8_t n);
void LOG(const char *tag, const char *format, ...);
void Serial_GET_order();
void Serial_debug_init(control_object * object, control_PID * PID);

void Serial_SendFloat(const char *tag, float value);
void Serial_SendInt(const char *tag, int value);
void Serial_SendUInt(const char *tag, unsigned int value);
void Serial_SendVar(const char *tag, unsigned int value);

#endif
