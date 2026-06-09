//
// Created by zhang on 2026/6/9.
//

#ifndef ITCAP_STM_CHECK_I_H
#define ITCAP_STM_CHECK_I_H

#include <stdint.h>

/* ========================================================================== */
/*                         配置 (与 CubeMX ADC 保持一致)                       */
/* ========================================================================== */

#define I_CHECK_CH_NUM      4           /* ADC 序列通道数 (IN6/7/8/9)        */
#define I_CHECK_BUF_LEN     (I_CHECK_CH_NUM * 2)    /* ping-pong 双缓冲长度  */

/* IN6 / IN7 在 ADC 序列中的 Rank (CubeMX Rank1=IN6, Rank2=IN7) */
#define I_CHECK_RANK_IA     0           /* adc_dma_buf[] 中相A偏移          */
#define I_CHECK_RANK_IB     1           /* adc_dma_buf[] 中相B偏移          */

/* ========================================================================== */
/*                               API                                          */
/* ========================================================================== */

/**
  * @brief  初始化电流检测模块
  *         必须在 MX_ADC1_Init / MX_DMA_Init 之后调用
  */
void I_Check_Init(void);

/**
  * @brief  启动 ADC DMA 循环采样 (TIM3 TRGO 触发)
  *         通常在 main.c USER CODE BEGIN 2 中调用
  */
void I_Check_Start(void);

/**
  * @brief  零点校准: 电机静止时采样取平均, 存入偏移量
  * @param  n  采样次数 (推荐 200~500)
  */
void I_Check_Calibrate(uint16_t n);

/**
  * @brief  ADC 半传输完成中断处理
  *         放到 HAL_ADC_ConvHalfCpltCallback() 中调用
  */
void I_Check_HalfCplt_Handler(void);

/**
  * @brief  ADC 传输完成中断处理
  *         放到 HAL_ADC_ConvCpltCallback() 中调用
  */
void I_Check_Cplt_Handler(void);

/* ======================== 读取最新电流值 ======================== */

/** 最新相A电流 (原始 ADC 值, 已减零点偏移) */
int16_t  I_Check_Get_Ia(void);

/** 最新相B电流 (原始 ADC 值, 已减零点偏移) */
int16_t  I_Check_Get_Ib(void);

/** 零点偏移值 (可用于调试观察) */
int16_t  I_Check_Get_OffsetA(void);
int16_t  I_Check_Get_OffsetB(void);

/** DMA 双缓冲原始数据指针 (8 个 uint16_t) */
volatile uint16_t* I_Check_Get_Buf(void);

#endif /* ITCAP_STM_CHECK_I_H */