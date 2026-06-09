//
// Created by zhang on 2026/6/9.
//

#include "check_I.h"
#include "adc.h"          // hadc1
#include "dma.h"
#include <string.h>

/* ========================================================================== */
/*                       DMA 双缓冲 & 内部状态                                 */
/* ========================================================================== */

/**
  * DMA 循环写入此缓冲区 (4ch × 2 半区 = 8 uint16_t):
  *
  *   TIM3 TRGO ↑
  *     → ADC 启动序列转换 IN6 → IN7 → IN8 → IN9
  *     → DMA 写入 buf[0..3]
  *     → 半传输中断: buf[0..3] 就绪
  *     → DMA 继续写入 buf[4..7]
  *     → 传输完成中断: buf[4..7] 就绪
  *     → 循环...
  */
static __IO uint16_t s_dma_buf[I_CHECK_BUF_LEN] = {0};

/* 最新有效值 (已减零点偏移) */
static volatile int16_t s_ia = 0;
static volatile int16_t s_ib = 0;

/* 零点偏移 */
static int16_t s_offset_a = 2048;    /* 默认 1/2 VREF ≈ 2048 */
static int16_t s_offset_b = 2048;

/* 当前哪个半区最新: 1=前半, 2=后半 */
static volatile uint8_t s_active_half = 0;

/* ========================================================================== */
/*                          内部工具                                          */
/* ========================================================================== */

/* 根据 s_active_half 取出对应半区的相电流原始值 */
static inline uint16_t raw_ia(void)
{
    if (s_active_half == 1)
        return s_dma_buf[I_CHECK_RANK_IA];               /* 前半 buf[0] */
    else
        return s_dma_buf[I_CHECK_RANK_IA + I_CHECK_CH_NUM]; /* 后半 buf[4] */
}

static inline uint16_t raw_ib(void)
{
    if (s_active_half == 1)
        return s_dma_buf[I_CHECK_RANK_IB];
    else
        return s_dma_buf[I_CHECK_RANK_IB + I_CHECK_CH_NUM];
}

/* ========================================================================== */
/*                          DMA 中断处理                                      */
/* ========================================================================== */

/**
  * @brief  ADC DMA 半传输完成 → 前半区数据就绪
  *         需要在 HAL_ADC_ConvHalfCpltCallback 中调用
  */
void I_Check_HalfCplt_Handler(void)
{
    s_active_half = 1;

    s_ia = (int16_t)s_dma_buf[I_CHECK_RANK_IA] - s_offset_a;
    s_ib = (int16_t)s_dma_buf[I_CHECK_RANK_IB] - s_offset_a;
}

/**
  * @brief  ADC DMA 传输完成 → 后半区数据就绪
  *         需要在 HAL_ADC_ConvCpltCallback 中调用
  */
void I_Check_Cplt_Handler(void)
{
    s_active_half = 2;

    uint16_t idx = I_CHECK_CH_NUM;
    s_ia = (int16_t)s_dma_buf[idx + I_CHECK_RANK_IA] - s_offset_a;
    s_ib = (int16_t)s_dma_buf[idx + I_CHECK_RANK_IB] - s_offset_a;
}

/* ========================================================================== */
/*                          公开 API                                         */
/* ========================================================================== */

/**
  * @brief  初始化: 清状态, 设默认偏移为 VREF/2
  */
void I_Check_Init(void)
{
    s_active_half = 0;
    s_ia = 0;
    s_ib = 0;
    /* 默认偏移 2048 (3.3V 中点), 后续可通过 Calibrate 覆盖 */
    s_offset_a = 2048;
    s_offset_b = 2048;
    memset((void*)s_dma_buf, 0, sizeof(s_dma_buf));
}

/**
  * @brief  启动 ADC DMA 循环采样
  *         TIM3 必须已在外部启动 (HAL_TIM_Base_Start_IT), 否则不会触发
  */
void I_Check_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_dma_buf, I_CHECK_BUF_LEN);
}

/**
  * @brief  零点校准: 电机静止时连续采样 n 次, 取平均做偏移
  */
void I_Check_Calibrate(uint16_t n)
{
    if (n == 0) n = 200;

    /* 短暂运行 DMA 收集 n 组样本 */
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint16_t  cnt   = 0;

    /* 等待 DMA 开始传输 */
    for (uint16_t i = 0; i < n; i++)
    {
        /* 等待新数据到来 (超时保护) */
        uint32_t timeout = 10000;
        uint8_t  prev = s_active_half;

        while (s_active_half == prev && timeout--)
            for (volatile uint32_t d = 0; d < 200; d++);   /* 粗略延时 */

        if (timeout == 0) break;    /* 超时退出 */

        sum_a += raw_ia();
        sum_b += raw_ib();
        cnt++;
    }

    if (cnt > 0)
    {
        s_offset_a = (int16_t)(sum_a / cnt);
        s_offset_b = (int16_t)(sum_b / cnt);
    }

    s_ia = 0;
    s_ib = 0;
}

/* ======================== Getter ======================== */

int16_t I_Check_Get_Ia(void)        { return s_ia; }
int16_t I_Check_Get_Ib(void)        { return s_ib; }
int16_t I_Check_Get_OffsetA(void)   { return s_offset_a; }
int16_t I_Check_Get_OffsetB(void)   { return s_offset_b; }
volatile uint16_t* I_Check_Get_Buf(void) { return s_dma_buf; }