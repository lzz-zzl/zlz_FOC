#include "stm32f4xx_hal.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"
#include "string.h"
#include <stdlib.h>
#include "Serial.h"

#include "app_task.h"
#include "OLED.h"
#include "Control.h"
#include "JY61P.h"


/* USART6 DMA 接收缓冲 (声明在 main.c) */
extern uint8_t uart6_rx_buf[];

/* USART2 DMA 接收缓冲 (声明在 main.c) */
extern uint8_t uart2_rx_buf[];

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
uint8_t Serial_RxFlag;
uint8_t Serial_RxFlag2;

uint8_t temp_rxdata6[11];

/* USART1 DMA+IDLE 接收缓冲 */
#define UART1_RX_BUF_SIZE  256
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];

/* USART6 DMA+IDLE 接收缓冲 (JY61P 陀螺仪) */
#define UART6_RX_BUF_SIZE  256

void Serial_Init(void)
{
	/* USART1/2 已在 main.c 中由 MX_USARTx_UART_Init() 初始化, 不再重复调用 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, UART1_RX_BUF_SIZE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, 256);
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	uart_sem = osSemaphoreNew(1, 0, NULL);  // 最大计数1, 初始可用0

	/* 启动 USART6 DMA+IDLE 接收 (JY61P 陀螺仪) */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buf, UART6_RX_BUF_SIZE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
}

/* ========== DMA TX 环型缓冲区 ========== */
#define TX_RING_SIZE    512       // 环形缓冲区大小
#define TX_CHUNK_MAX    256       // 单次 DMA 最大长度

static uint8_t  tx_ring[TX_RING_SIZE];    // 环形缓冲区
static volatile uint16_t tx_head = 0;     // 写入位置 (生产者)
static volatile uint16_t tx_tail = 0;     // 读取位置 (消费者)
static volatile uint8_t tx_dma_busy = 0;  // DMA 正在发送
static uint8_t  tx_dma_buf[TX_CHUNK_MAX]; // DMA 连续缓冲区

/* 启动 DMA 发送 (从环形缓冲区搬数据到 DMA 缓冲区再发送) */
static void tx_dma_start(void)
{
    if (tx_dma_busy) return;    /* 快速路径: DMA 正在忙，等待完成回调 */

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 关中断后再确认一次 — 防止 ISR 在刚那条 if 之后抢占了 */
    if (tx_dma_busy) {
        __set_PRIMASK(primask);
        return;
    }

    uint16_t avail = (tx_head >= tx_tail)
                     ? (tx_head - tx_tail)
                     : (TX_RING_SIZE - tx_tail);
    if (avail == 0) {
        __set_PRIMASK(primask);
        return;
    }

    if (avail > TX_CHUNK_MAX) avail = TX_CHUNK_MAX;

    memcpy(tx_dma_buf, &tx_ring[tx_tail], avail);
    /* 先推进尾指针！假如 DMA 瞬间完成，回调中 tx_tail 已经前进，
       不会重复复制同一块数据到 tx_dma_buf (否则会覆盖正在 DMA 的数据) */
    tx_tail = (tx_tail + avail) % TX_RING_SIZE;
    tx_dma_busy = 1;
    __set_PRIMASK(primask);   /* 恢复中断 */

    if (HAL_UART_Transmit_DMA(&huart1, tx_dma_buf, avail) != HAL_OK) {
        /* HAL 失败极罕见 (UART 未就绪等) — 释放锁，数据已经 advance 了，回不去了 */
        tx_dma_busy = 0;
    }
}

/* 向环形缓冲区写一个字节 (多任务安全) */
static void tx_ring_put(uint8_t c)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t next = (tx_head + 1) % TX_RING_SIZE;
    if (next == tx_tail) {
        /* 缓冲区满 → 丢弃最旧的一个字节 */
        tx_tail = (tx_tail + 1) % TX_RING_SIZE;
    }
    tx_ring[tx_head] = c;
    tx_head = next;

    __set_PRIMASK(primask);
}

void Serial_SendByte(uint8_t Byte)
{
    tx_ring_put(Byte);
    tx_dma_start();
}

void Serial_SendString(char *String)
{
    for (uint8_t i = 0; String[i] != '\0'; i++) {
        tx_ring_put(String[i]);
    }
    tx_dma_start();
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++) {
        tx_ring_put(Array[i]);
    }
    tx_dma_start();
}

/**
  * @brief DMA 发送完成回调 — 发送环形缓冲区中的下一块数据.
  *
  * HAL 的 UART_DMATransmitCplt 会在 DMA 完成后使能 UART_IT_TC，
  * 导致本回调被 DMA ISR 和 UART ISR 各调用一次。我们在入口处直接
  * 关掉 TCIE 来阻止冗余的 UART TC 调用。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 关掉 UART TC 中断 — HAL 刚在 UART_DMATransmitCplt 里开了它 */
        huart1.Instance->CR1 &= ~USART_CR1_TCIE;

        tx_dma_busy = 0;
        tx_dma_start();  /* 继续发送下一块 (如果有) */
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

/**
  * @brief  VOFA+ justfloat 协议输出 — 二进制裸浮点，尾标记 +inf
  *         每个 float 4 字节 (STM32 little-endian IEEE754)
  *         结尾 4 字节 0x00 0x00 0x80 0x7F (+inf) 标记帧结束
  *         上位机选 justfloat 协议，自动波形显示
  * @param  data 浮点数组
  * @param  n    数组元素个数
  */
void Serial_JustFloat(float *data, uint8_t n)
{
    /* 尾标记: IEEE754 +inf = 0x7F800000, little-endian → [00 00 80 7F] */
    static const uint8_t tail[4] = { 0x00, 0x00, 0x80, 0x7F };

    for (uint8_t i = 0; i < n; i++) {
        Serial_SendArray((uint8_t *)&data[i], 4);
    }
    Serial_SendArray((uint8_t *)tail, 4);
}

/**
  * @brief  简易日志打印 [tag] format...
  *         用法: LOG("FOC", "speed=%d", speed);
  */
void LOG(const char *tag, const char *format, ...)
{
    char buf[128];
    uint16_t pos = 0;

    /* [tag] */
    buf[pos++] = '[';
    while (*tag && pos < sizeof(buf) - 3) {
        buf[pos++] = *tag++;
    }
    buf[pos++] = ']';
    buf[pos++] = ' ';

    /* format 正文 */
    va_list arg;
    va_start(arg, format);
    pos += vsnprintf(buf + pos, sizeof(buf) - pos, format, arg);
    va_end(arg);

    /* 换行 */
    if (pos < sizeof(buf) - 2) {
        buf[pos++] = '\r';
        buf[pos++] = '\n';
    }
    buf[pos] = '\0';

    Serial_SendString(buf);
}

// 针对不同类型的实际处理函数
void Serial_SendFloat(const char *tag, float value) {
	char buffer[50];
	snprintf(buffer, sizeof(buffer), "%s: %.2f\n", tag, value);
	Serial_Printf("%s", buffer);  // 实际发送代码
}

void Serial_SendInt(const char *tag, int value) {
	char buffer[50];
	snprintf(buffer, sizeof(buffer), "%s: %d\n", tag, value);
	Serial_Printf("%s", buffer);
}

void Serial_SendUInt(const char *tag, unsigned int value) {
	char buffer[50];
	snprintf(buffer, sizeof(buffer), "%s: %u\n", tag, value);
	Serial_Printf("%s", buffer);
}

// 泛型宏：根据类型自动选择函数
#define Serial_SendVa_H(tag, value) \
_Generic((value), \
float: Serial_SendFloat, \
double: Serial_SendFloat, \
int: Serial_SendInt, \
unsigned int: Serial_SendUInt, \
char*: Serial_SendString, \
const char*: Serial_SendString \
)(tag, value)

void Serial_SendVar(const char *tag, unsigned int value) {
	Serial_SendVa_H(tag, value);
}

/**
  * @brief  VOFA+ firewater 协议输出 — 逗号分隔浮点数，换行结尾
  *         上机位选 firewater 模式即可自动绘图
  *         使用整数缩放避免 newlib-nano %f 问题
  * @param  data 浮点数组
  * @param  n    数组元素个数
  */
void Serial_Firewater(float *data, uint8_t n)
{
    char buf[128];
    uint16_t pos = 0;
    for (uint8_t i = 0; i < n; i++) {
        /* 缩放 ×100 保留2位小数，用整数打印 */
        int16_t v = (int16_t)(data[i] * 100.0f);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%d", v);
        if (i < n - 1 && pos < sizeof(buf) - 2)
            buf[pos++] = ',';
    }
    buf[pos++] = '\n';
    buf[pos]   = '\0';
    Serial_SendString(buf);
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

void receive_state1() {
	static uint8_t RxState_1 = 0;    // 接收状态机
	static uint8_t pRxPacket_1 = 0;  // 缓存指针
	uint8_t RxData;
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
}

char kv_buf[64];
static uint8_t kv_len = 0;
volatile uint8_t kv_ready = 0;

void receive_state_keyVal(uint8_t data) {
	if (data == '\r') return;
	if (data == '\n') {
		kv_buf[kv_len] = '\0';
		kv_ready = 1;
		kv_len = 0;
		osSemaphoreRelease(uart_sem);
		return;
	}
	if (kv_len < sizeof(kv_buf) - 1) {
		kv_buf[kv_len++] = (char)data;
	}
}

/**
  * @brief  USART1 DMA+IDLE 接收完成回调
  *         收到数据或总线空闲时触发，逐字节喂给状态机
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        for (uint16_t i = 0; i < Size; i++) {
            receive_state_keyVal(uart1_rx_buf[i]);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, UART1_RX_BUF_SIZE);
    }
    else if (huart->Instance == USART6) {
        for (uint16_t i = 0; i < Size; i++) {
            jy61p_byte_receive(uart6_rx_buf[i]);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buf, UART6_RX_BUF_SIZE);
    }
    else if (huart->Instance == USART2) {
        for (uint16_t i = 0; i < Size; i++) {
        	static uint8_t count = 0;
        	count++;
        	if (count>100) {
        		Serial_SendVar("t1",uart2_rx_buf[0]);
        		Serial_SendVar("t2",uart2_rx_buf[1]);
        		Serial_SendVar("t2",uart2_rx_buf[2]);
        		Serial_SendVar("t2",uart2_rx_buf[3]);
        		Serial_SendVar("t2",uart2_rx_buf[4]);
        		count = 0;
        	}
        }
    	test_count121++;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, 256);
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	/* USART1: RxEventCallback 已处理数据并重启了 DMA, 这里不做任何事 */
	if (huart->Instance == USART1) {
		return;
	}

	if(huart->Instance == USART2 )
	{
		/* DMA 缓冲收满 (256 字节) — 喂给解析函数再重启 DMA */
		for (uint16_t i = 0; i < 256; i++) {
			/* TODO: 替换为你的视觉数据解析函数 */
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, 256);
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	}
	else if(huart->Instance == USART6){
		/* DMA 缓冲收满 (256 字节) — 也喂给状态机再重启 DMA */
		for (uint16_t i = 0; i < 256; i++) {
			jy61p_byte_receive(uart6_rx_buf[i]);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buf, 256);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	}
}

/**
  * @brief UART 错误回调 — 检测到 ORE/PE/FE 时重新开启中断接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		/* 清除 ORE/NE/FE/PE 错误标志并重启 RX DMA */
		volatile uint8_t tmp = huart1.Instance->DR;  (void)tmp;
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, UART1_RX_BUF_SIZE);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	}
	else if (huart->Instance == USART6) {
		/* 清除错误标志 (ORE/NE/FE/PE) — HAL 已清部分，再读一次 DR 彻底清 ORE */
		volatile uint8_t tmp = huart6.Instance->DR;  (void)tmp;
		__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
		/* 重新开启 DMA+IDLE 接收 */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_buf, 256);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	}
	else if (huart->Instance == USART2) {
		/* 清除错误标志并重启 RX DMA */
		volatile uint8_t tmp = huart2.Instance->DR;  (void)tmp;
		__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, 256);
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
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

void Serial_GET_order2() {
	if (Serial_RxFlag == 1)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		OLED_ShowString(1,1,Serial_RxPacket);
		char * tag = strtok(Serial_RxPacket,":");
		if(strcmp(tag, "value") == 0)
		{
			int8_t name = atoi(strtok(NULL, ","));

		}

		Serial_RxFlag = 0;
	}
}