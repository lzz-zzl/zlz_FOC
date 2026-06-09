//
// Created by zhang on 2026/6/9.
//
// 完全自包含的 ADC + DMA 电流采样模块.
// 不依赖 CubeMX 生成的 adc.c / dma.c, 所有 GPIO / DMA / ADC 初始化在此完成.
//
// 双电机 × 两相:
//   电机1: IN6(PA6)=A相, IN7(PA7)=B相
//   电机2: IN8(PB0)=A相, IN9(PB1)=B相
// 触发:  TIM5_CH1 OC1REF 上升沿 (PA0 AF2 PWM, 由 ADC_EXTERNALTRIGCONV_T5_CC1 选择)
//        TIM5 Period=5599 → 84MHz/5600 ≈ 15kHz
// DMA:   CIRCULAR 双缓冲 (ping-pong)
//
//

#include "check_I.h"
#include "main.h"
#include <string.h>

/* ========================================================================== */
/*                       句柄 (static)                                        */
/* ========================================================================== */

static ADC_HandleTypeDef  s_hadc1;
static DMA_HandleTypeDef  s_hdma_adc1;

/* ========================================================================== */
/*                       DMA 双缓冲 & 内部状态                                 */
/* ========================================================================== */

static __IO uint16_t s_dma_buf[I_CHECK_BUF_LEN] = {0};

/* 四路电流 (已减零点偏移) */
static volatile int16_t s_m1_a = 0;
static volatile int16_t s_m1_b = 0;
static volatile int16_t s_m2_a = 0;
static volatile int16_t s_m2_b = 0;

/* 零点偏移 */
static int16_t s_off_m1_a = 2048;
static int16_t s_off_m1_b = 2048;
static int16_t s_off_m2_a = 2048;
static int16_t s_off_m2_b = 2048;

static volatile uint8_t s_active_half = 0;
static uint8_t s_inited = 0;

/* ========================================================================== */
/*                          内部工具                                          */
/* ========================================================================== */

/* 从最新半区取原始值 */
static inline uint16_t raw(uint8_t rank)
{
    if (s_active_half == 1)
        return s_dma_buf[rank];
    else
        return s_dma_buf[rank + I_CHECK_CH_NUM];
}

/* ========================================================================== */
/*                    GPIO / DMA / ADC 自初始化                               */
/* ========================================================================== */

static void adc_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA0 = TIM5_CH1 (AF2) — OC1REF 脉冲触发 ADC */
    gpio.Pin       = GPIO_PIN_0;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA1 = ADC1_IN9 — 电机2 相B (保留, 未来用) */
    gpio.Pin       = GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_ANALOG;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA6/PA7 = ADC1_IN6/IN7 — 电机1 相A/B */
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB0/PB1 = ADC1_IN8/IN9 — 电机2 相A/B */
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void adc_dma_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    s_hdma_adc1.Instance                 = DMA2_Stream0;
    s_hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    s_hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&s_hdma_adc1) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc1);
}

/**
  * @brief  配置 TIM5_CH1 输出 PWM, 产生 OC1REF 上升沿.
  *         OC1REF 内部连接到 ADC 触发选择器 (ADC_EXTERNALTRIGCONV_T5_CC1).
  *         PA0 配置为 AF2, 外部引脚不受影响.
  *         TIM5 必须在外部已由 MX_TIM5_Init() 初始化.
  */
static void adc_tim5_trigger_init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    extern TIM_HandleTypeDef htim5;       /* 由 CubeMX tim.c 定义 */

    /* 使能 TIM5 PWM 子模式 (基模式已由 MX_TIM5_Init 完成) */
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
        Error_Handler();

    /* CH1: PWM1, Pulse=Period/2, 每个周期产生 OC1REF 翻转 */
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = htim5.Init.Period / 2;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    /* 启动 PWM 输出 (OC1REF 开始跳变) */
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
}

static void adc_init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    s_hadc1.Instance                   = ADC1;
    s_hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV6;
    s_hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode          = ENABLE;
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T5_CC1;
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion       = 4;
    s_hadc1.Init.DMAContinuousRequests = ENABLE;
    s_hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&s_hadc1) != HAL_OK)
        Error_Handler();

    /* Rank 1: IN6 — 电机1 相A */
    ch.Channel      = ADC_CHANNEL_6;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &ch) != HAL_OK)
        Error_Handler();

    /* Rank 2: IN7 — 电机1 相B */
    ch.Channel      = ADC_CHANNEL_7;
    ch.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &ch) != HAL_OK)
        Error_Handler();

    /* Rank 3: IN8 — 电机2 相A */
    ch.Channel      = ADC_CHANNEL_8;
    ch.Rank         = 3;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &ch) != HAL_OK)
        Error_Handler();

    /* Rank 4: IN9 — 电机2 相B */
    ch.Channel      = ADC_CHANNEL_9;
    ch.Rank         = 4;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &ch) != HAL_OK)
        Error_Handler();
}

/* ========================================================================== */
/*                          DMA 中断处理                                      */
/* ========================================================================== */

void I_Check_HalfCplt_Handler(void)
{
    s_active_half = 1;

    s_m1_a = (int16_t)s_dma_buf[I_CHECK_M1_A] - s_off_m1_a;
    s_m1_b = (int16_t)s_dma_buf[I_CHECK_M1_B] - s_off_m1_b;
    s_m2_a = (int16_t)s_dma_buf[I_CHECK_M2_A] - s_off_m2_a;
    s_m2_b = (int16_t)s_dma_buf[I_CHECK_M2_B] - s_off_m2_b;
}

void I_Check_Cplt_Handler(void)
{
    s_active_half = 2;
    uint16_t idx = I_CHECK_CH_NUM;

    s_m1_a = (int16_t)s_dma_buf[idx + I_CHECK_M1_A] - s_off_m1_a;
    s_m1_b = (int16_t)s_dma_buf[idx + I_CHECK_M1_B] - s_off_m1_b;
    s_m2_a = (int16_t)s_dma_buf[idx + I_CHECK_M2_A] - s_off_m2_a;
    s_m2_b = (int16_t)s_dma_buf[idx + I_CHECK_M2_B] - s_off_m2_b;
}

/* ========================================================================== */
/*                          公开 API                                         */
/* ========================================================================== */

void I_Check_Init(void)
{
    memset((void*)s_dma_buf, 0, sizeof(s_dma_buf));
    s_active_half = 0;
    s_m1_a = s_m1_b = s_m2_a = s_m2_b = 0;

    s_off_m1_a = 2048;
    s_off_m1_b = 2048;
    s_off_m2_a = 2048;
    s_off_m2_b = 2048;

    adc_gpio_init();
    adc_dma_init();
    adc_tim5_trigger_init();    /* TIM5_CH1 OC1REF → ADC 触发 */
    adc_init();                 /* ADC 选择 T5_CC1 触发源 */

    s_inited = 1;
}

void I_Check_Start(void)
{
    if (!s_inited) return;

    if (HAL_ADC_Start_DMA(&s_hadc1,
                          (uint32_t*)s_dma_buf,
                          I_CHECK_BUF_LEN) != HAL_OK)
        Error_Handler();
}

void I_Check_Stop(void)
{
    HAL_ADC_Stop_DMA(&s_hadc1);
}

/**
  * @brief  零点校准: 四通道同时采集 n 次取平均
  */
void I_Check_Calibrate(uint16_t n)
{
    if (n == 0) n = 200;

    uint32_t sum[4] = {0, 0, 0, 0};
    uint16_t cnt = 0;

    for (uint16_t i = 0; i < n; i++)
    {
        uint32_t timeout = 50000;
        uint8_t  prev    = s_active_half;

        while (s_active_half == prev && timeout--)
            for (volatile uint32_t d = 0; d < 50; d++) {}

        if (timeout == 0) break;

        sum[0] += raw(I_CHECK_M1_A);
        sum[1] += raw(I_CHECK_M1_B);
        sum[2] += raw(I_CHECK_M2_A);
        sum[3] += raw(I_CHECK_M2_B);
        cnt++;
    }

    if (cnt > 0)
    {
        s_off_m1_a = (int16_t)(sum[0] / cnt);
        s_off_m1_b = (int16_t)(sum[1] / cnt);
        s_off_m2_a = (int16_t)(sum[2] / cnt);
        s_off_m2_b = (int16_t)(sum[3] / cnt);
    }

    s_m1_a = s_m1_b = s_m2_a = s_m2_b = 0;
}

/* ======================== Getter ======================== */

int16_t I_Check_Get_M1_A(void)      { return s_m1_a; }
int16_t I_Check_Get_M1_B(void)      { return s_m1_b; }
int16_t I_Check_Get_M2_A(void)      { return s_m2_a; }
int16_t I_Check_Get_M2_B(void)      { return s_m2_b; }

int16_t I_Check_Get_Offset_M1_A(void)  { return s_off_m1_a; }
int16_t I_Check_Get_Offset_M1_B(void)  { return s_off_m1_b; }
int16_t I_Check_Get_Offset_M2_A(void)  { return s_off_m2_a; }
int16_t I_Check_Get_Offset_M2_B(void)  { return s_off_m2_b; }

volatile uint16_t* I_Check_Get_Buf(void) { return s_dma_buf; }
