#include "stm32f4xx_hal.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"
#include "string.h"
#include <stdlib.h>
#include "Serial.h"
#include "FOC_1.h"
#include "OLED.h"
#include "Control.h"



osSemaphoreId_t uart_sem;

void Serial_Printf(char *format, ...);

control_object * object_debug;
control_PID * PID_debug;

void Serial_debug_init(control_object * object, control_PID * PID){
	object_debug = object;
	PID_debug = PID;
	Serial_Printf("[plot,%d,%d]",(int)(object->Actual),
	  (int)(object->Target)
	  //, (int16_t)(PID->object.Out)
	  );
}

char Serial_RxPacket[100];
char Serial_RxPacket2[100];

uint8_t temp_rxdata1;
uint8_t temp_rxdata2;
uint8_t Serial_RxFlag;
uint8_t Serial_RxFlag2;

void Serial_Init(void)
{
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* 使能 USART1 中断 (NVIC) — CubeMX 未生成此项 */
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_UART_Receive_IT(&huart1, &temp_rxdata1, 1);
	HAL_UART_Receive_IT(&huart2, &temp_rxdata2, 1);
	uart_sem = osSemaphoreNew(1, 0, NULL);  // 最大计数1, 初始可用0
}

void Serial_SendByte(uint8_t Byte)
{
	HAL_UART_Transmit(&huart1, &Byte, 1, HAL_MAX_DELAY);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t RxState_1 = 0;    // 接收状态机
	static uint8_t pRxPacket_1 = 0;  // 缓存指针

	static uint8_t RxState_2 = 0;    // 接收状态机
	static uint8_t pRxPacket_2 = 0;  // 缓存指针


	uint8_t RxData;


	if(huart->Instance == USART1 )
	{
		RxData = temp_rxdata1;
		// 状态机解析：[数据] 格式
		if (RxState_1 == 0)
		{
			// 起始符检测：'[' 且未处于接收中
			if (RxData == '[' && Serial_RxFlag == 0)
			{
				RxState_1 = 1;
				pRxPacket_1 = 0;  // 重置缓存指针
			}
		}
		else if (RxState_1 == 1)
		{
			// 结束符检测：']'
			if (RxData == ']')
			{
				RxState_1 = 0;
				Serial_RxPacket[pRxPacket_1] = '\0';  // 字符串结束符
				Serial_RxFlag = 1;                  // 接收完成置位
				osSemaphoreRelease(uart_sem);
			}
			else
			{
				// 防止缓存溢出
				if(pRxPacket_1 < 100 - 1)
				{
					Serial_RxPacket[pRxPacket_1] = RxData;
					pRxPacket_1++;
				}
			}
		}
		if(huart->Instance == USART1) HAL_UART_Receive_IT(&huart1, &temp_rxdata1, 1);
	}
	else if(huart->Instance == USART2 )
	{
		RxData = temp_rxdata2;
		// 状态机解析：[数据] 格式
		if (RxState_2 == 0)
		{
			// 起始符检测：'[' 且未处于接收中
			if (RxData == '[' && Serial_RxFlag == 0)
			{
				RxState_2 = 1;
				pRxPacket_2 = 0;  // 重置缓存指针
			}
		}
		else if (RxState_2 == 1)
		{
			// 结束符检测：']'
			if (RxData == ']')
			{
				RxState_2 = 0;
				Serial_RxPacket2[pRxPacket_2] = '\0';  // 字符串结束符
				Serial_RxFlag2 = 1;                  // 接收完成置位
			}
			else
			{
				// 防止缓存溢出
				if(pRxPacket_2 < 100 - 1)
				{
					Serial_RxPacket2[pRxPacket_2] = RxData;
					pRxPacket_2++;
				}
			}
		}
		if(huart->Instance == USART2) HAL_UART_Receive_IT(&huart2, &temp_rxdata2, 1);
	}
}





uint16_t Serial_value[10];
uint8_t Serial_key[10];
float Serial_slider[5];
uint16_t Serial_joystick[2][2];

void Serial_GET_order() {
	if (Serial_RxFlag == 1)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		OLED_ShowString(1,1,Serial_RxPacket);
		char * tag = strtok(Serial_RxPacket,",");
		if(strcmp(tag, "value") == 0)
		{
			int8_t name = atoi(strtok(NULL, ","));
			int16_t ation = atoi(strtok(NULL, ","));
			Serial_value[name]= ation;

			//test
		}
		if(strcmp(tag, "key") == 0)
		{
			int8_t name = atoi(strtok(NULL, ","));
			int16_t ation = atoi(strtok(NULL, ","));
			if (ation <=2) {
				Serial_key[name] = ation = 0? 0:1;
			}
			if (ation == 3) {
				Serial_key [name] ++;
			}
			if (ation == 4) {
				Serial_key [name] --;
			}
		}

		else if(strcmp(tag, "slider") == 0)
		{
			char * name = strtok(NULL, ",");
			float num = atof(strtok(NULL, ","));
			if(strcmp(name,"0") == 0)
			{
				Serial_slider[0] = num;
			}
			else if(strcmp(name,"1") == 0)
			{
				Serial_slider[1] = num;
			}
			else if(strcmp(name,"2") == 0)
			{
				Serial_slider[2] = num;
			}
			else if(strcmp(name,"3") == 0)
			{
				Serial_slider[3]  = num;
			}
			else if(strcmp(name,"kp") == 0)
			{
				PID_debug->kp = num;
			}
			else if(strcmp(name,"ki") == 0)
			{
				PID_debug->ki = num;
			}
			else if(strcmp(name,"kd") == 0)
			{
				PID_debug->kd = num;
			}
		}
		else if(strcmp(tag,"joystick") == 0)
		{
			Serial_joystick[0][0] = atoi(strtok(NULL, ","));
			Serial_joystick[0][1] = atoi(strtok(NULL, ","));
			Serial_joystick[1][0] = atoi(strtok(NULL, ","));
			Serial_joystick[1][1] = atoi(strtok(NULL, ","));
			//值在
		}
		Serial_RxFlag = 0;
	}
	if (Serial_RxFlag2 == 1) {
		char * tag = strtok(Serial_RxPacket2,",");
		if(strcmp(tag, "value") == 0)
		{
			int8_t name = atoi(strtok(NULL, ","));
			int16_t ation = atoi(strtok(NULL, ","));
			Serial_value[name]= ation;

			//test
		}
		Serial_RxFlag2 = 0;
	}

}