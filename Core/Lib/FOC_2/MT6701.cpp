//
// Created by zhang on 2026/6/9.
//
// MT6701 磁编码器 — SSI 通信.
//
// SSI 时序 (CS 低电平有效, SCK 连续时钟):
//   1 START bit(1) + 12 angle bits(A11..A0) + 1 PAR bit + 1 DO bit = 15 bits
//
// SPI 配置: 16bit 数据宽度, CPOL=0 CPHA=0 (Mode 0)
//   发送 1 个半字 (dummy 0x0000) 产生 16 个 SCK 脉冲,
//   收到的 16 位中: bit[15]=START, bit[14:3]=角度, bit[2]=奇偶, bit[1]=DO
//

#include "MT6701.h"

/* ========================================================================== */
/*                              构造函数                                       */
/* ========================================================================== */

MT6701::MT6701(const Config &cfg)
    : m_hspi(cfg.hspi),
      m_cs_port(cfg.cs_port),
      m_cs_pin(cfg.cs_pin),
      m_angle_12bit(0),
      m_last_angle_12bit(0),
      m_last_speed_rpm(0.0f),
      m_rotation_count(0),
      m_adaptive_dif_count(0),
      m_last_tick(0),
      m_last_diag(1)
{
    /* 配置 CS 引脚为输出 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = m_cs_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(m_cs_port, &GPIO_InitStruct);

    cs_deassert();
}

/* ========================================================================== */
/*                         CS 引脚控制                                         */
/* ========================================================================== */

void MT6701::cs_assert()  {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
}

void MT6701::cs_deassert() {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

/* ========================================================================== */
/*                         SSI 原始读取                                        */
/* ========================================================================== */

/**
  * @brief  通过 SSI 协议读取 12bit 角度及诊断位.
  *         SPI 配置为 16bit 数据宽度, CPOL=0, CPHA=0.
  * @retval 12bit 角度值 (0~4095), 诊断位保存在 m_last_diag
  */
int16_t MT6701::ssi_read_angle() {
    cs_assert();

    /* 延时 T_CS_LOAD ≈ 1us (MT6701 要求 CS 低后等待 ~1us) */
    for (volatile uint32_t d = 0; d < 20; d++) {}

    uint16_t tx = 0x0000;   /* dummy, 产生 SCK 时钟 */
    uint16_t rx = 0;

    HAL_SPI_TransmitReceive(m_hspi, (uint8_t*)&tx, (uint8_t*)&rx, 1, HAL_MAX_DELAY);

    cs_deassert();

    /* SSI 帧格式 (16bit SPI 收到的数据):
     *   bit[15] = START (1)
     *   bit[14:3] = 12-bit 角度
     *   bit[2]   = 奇偶校验
     *   bit[1]   = DO (1=正常, 0=诊断)
     *   bit[0]   = 多余位 (第 16 个 SCK, 忽略)
     */
    m_last_diag = (uint8_t)((rx >> 1) & 1);
    return (int16_t)((rx >> 3) & 0x0FFF);
}

int16_t MT6701::read_raw_angle() {
    return ssi_read_angle();
}

/* ========================================================================== */
/*                      角度刷新方法                                           */
/* ========================================================================== */

int16_t MT6701::refresh_raw() {
    m_angle_12bit = ssi_read_angle();
    return m_angle_12bit;
}

int16_t MT6701::refresh_low_pass(float alpha) {
    int16_t raw = ssi_read_angle();
    m_angle_12bit = (int16_t)((float)raw * alpha + (float)m_angle_12bit * (1.0f - alpha));
    return m_angle_12bit;
}

int16_t MT6701::refresh_adaptive(float max_diff, uint8_t max_dif_count) {
    int16_t raw = ssi_read_angle();
    int16_t diff = raw - m_last_angle_12bit;

    if (diff > max_diff || diff < -max_diff) {
        m_angle_12bit = m_last_angle_12bit;
        m_adaptive_dif_count++;
        if (m_adaptive_dif_count >= max_dif_count) {
            m_angle_12bit = raw;
            m_adaptive_dif_count = 0;
        }
    } else {
        m_angle_12bit = raw;
    }

    m_last_angle_12bit = m_angle_12bit;
    return m_angle_12bit;
}

int16_t MT6701::refresh_rate_limit(float max_diff) {
    int16_t raw = ssi_read_angle();
    int16_t diff = raw - m_last_angle_12bit;

    if (diff > max_diff) {
        m_angle_12bit = m_last_angle_12bit + (int16_t)max_diff;
    } else if (diff < -max_diff) {
        m_angle_12bit = m_last_angle_12bit - (int16_t)max_diff;
    } else {
        m_angle_12bit = raw;
    }

    m_last_angle_12bit = m_angle_12bit;
    return m_angle_12bit;
}

/* ========================================================================== */
/*                      角度值获取                                             */
/* ========================================================================== */

float MT6701::get_angle_deg() const {
    return (float)m_angle_12bit * 360.0f / RESOLUTION;
}

int16_t MT6701::get_angle_12bit() const {
    return m_angle_12bit;
}

/* ========================================================================== */
/*                      速度计算（基于 HAL_GetTick() 差值）                     */
/* ========================================================================== */

float MT6701::calc_speed_rpm() {
    uint32_t now     = HAL_GetTick();
    uint32_t dt_ms   = now - m_last_tick;

    if (m_last_tick == 0 || dt_ms == 0) {
        m_last_tick = now;
        m_last_angle_12bit = m_angle_12bit;
        return 0.0f;
    }

    int16_t delta = m_angle_12bit - m_last_angle_12bit;
    int16_t error = 2;  // 死区，忽略微小抖动

    if (delta < error && delta > -error) {
        delta = 0;
    } else if (delta < -RESOLUTION * 8 / 10) {
        m_rotation_count += 1;
        delta = RESOLUTION + delta;
    } else if (delta > RESOLUTION * 8 / 10) {
        m_rotation_count -= 1;
        delta = delta - RESOLUTION;
    }

    m_last_angle_12bit = m_angle_12bit;
    m_last_tick       = now;

    return (float)delta * 60000.0f / (float)RESOLUTION / (float)dt_ms;
}

float MT6701::calc_speed_rpm_low_pass(float alpha) {
    float current_rpm = calc_speed_rpm();
    m_last_speed_rpm = current_rpm * alpha + m_last_speed_rpm * (1.0f - alpha);
    return m_last_speed_rpm;
}

/* ========================================================================== */
/*                      圈数 & 状态                                            */
/* ========================================================================== */

int32_t MT6701::get_rotation_count() const {
    return m_rotation_count;
}

uint8_t MT6701::get_status() const {
    return m_last_diag;
}
