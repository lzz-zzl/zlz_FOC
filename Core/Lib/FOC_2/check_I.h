//
// Created by zhang on 2026/6/9.
//
// 双电机 × 两相电流检测 (ADC1 + DMA).
// 硬件: ADC1 IN6(PA6)=M1_A, IN7(PA7)=M2_B, IN8(PB0)=M2_A, IN9(PB1)=M1_B
// 触发: TIM5_CH1 OC1REF 上升沿 (@ 15kHz, 通过 PA0 AF2 输出 PWM)
// DMA:  CIRCULAR 双缓冲 (ping-pong), 中断回调更新最新值
//

#ifndef ITCAP_STM_CHECK_I_H
#define ITCAP_STM_CHECK_I_H

#include <stdint.h>

/* ========================================================================== */
/*                        通道映射                                            */
/* ========================================================================== */

/* ADC1 序列 (Rank):  1=IN6  2=IN7  3=IN8  4=IN9 */
#define I_CHECK_M1_A     0       /* s_dma_buf[] 偏移: 电机1 相A (IN6) */
#define I_CHECK_M1_B     3       /*                    电机1 相B (IN9) — 硬件实际接 PB1 */
#define I_CHECK_M2_A     2       /*                    电机2 相A (IN8) */
#define I_CHECK_M2_B     1       /*                    电机2 相B (IN7) — 硬件实际接 PA7 */

#define I_CHECK_CH_NUM   2       /* 序列通道数 */
#define I_CHECK_BUF_LEN  (I_CHECK_CH_NUM * 2)   /* ping-pong 双缓冲长度 */

/* ========================================================================== */
/*                              API                                           */
/* ========================================================================== */

/**
  * @brief  初始化 ADC1 + DMA 电流检测
  *         不依赖 CubeMX 的 MX_ADC1_Init / MX_DMA_Init
  *         调用前 TIM3 时钟必须已使能 (MX_TIM3_Init)
  */
void I_Check_Init(void);

/**
  * @brief  启动 ADCDMA 循环采样 (TIM5 TRGO 触发)
  */
void I_Check_Start(void);

/**
  * @brief  停止 ADC DMA
  */
void I_Check_Stop(void);

/**
  * @brief  零点校准 (两电机四通道同时校准)
  * @param  n  采样次数, 推荐 200~500
  */
void I_Check_Calibrate(uint16_t n);

/**
  * @brief  ADC DMA 半传输完成中断处理
  *         放在 HAL_ADC_ConvHalfCpltCallback() 中调用
  */
void I_Check_HalfCplt_Handler(void);

/**
  * @brief  ADC DMA 传输完成中断处理
  *         放在 HAL_ADC_ConvCpltCallback() 中调用
  */
void I_Check_Cplt_Handler(void);

/* ===================== 读最新电流值 (已减零点偏移) ===================== */

int16_t  I_Check_Get_M1_A(void);
int16_t  I_Check_Get_M1_B(void);
void I_check_M1(int16_t* m1a,int16_t *m1b);
int16_t  I_Check_Get_M2_A(void);
int16_t  I_Check_Get_M2_B(void);
void I_check_M2(int16_t* m2a,int16_t *m2b);

/* ===================== 零点偏移 (只读) ===================== */

int16_t  I_Check_Get_Offset_M1_A(void);
int16_t  I_Check_Get_Offset_M1_B(void);
int16_t  I_Check_Get_Offset_M2_A(void);
int16_t  I_Check_Get_Offset_M2_B(void);

/* ===================== 原始 DMA 缓冲 ===================== */

volatile uint16_t* I_Check_Get_Buf(void);

#endif /* ITCAP_STM_CHECK_I_H */
