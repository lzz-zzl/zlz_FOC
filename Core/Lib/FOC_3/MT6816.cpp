//
// Created by zhang on 2026/6/3.
//

#include "stm32f4xx_hal.h"
// #include "spi.h"
#include "MT6816.h"

/* ========================================================================== */
/*                              构造函数                                       */
/* ========================================================================== */

MT6816::MT6816(const Config &cfg)
    : m_hspi(cfg.hspi),
      m_cs_port(cfg.cs_port),
      m_cs_pin(cfg.cs_pin),
      m_angle_14bit(0),
      m_last_angle_14bit(0),
      m_last_speed_rpm(0.0f),
      m_rotation_count(0),
      m_adaptive_dif_count(0),
      m_last_tick(0)
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

void MT6816::cs_assert()  {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET);
}

void MT6816::cs_deassert() {
    HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);
}

/* ========================================================================== */
/*                         SPI 原始读取                                        */
/* ========================================================================== */

int16_t MT6816::spi_read_angle() {
    cs_assert();

    uint16_t swap_data[2] = {0x8300, 0x8400};
    uint8_t  data_get[2]  = {0, 0};

    HAL_SPI_TransmitReceive(m_hspi, (uint8_t *)&swap_data[0], &data_get[0], 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(m_hspi, (uint8_t *)&swap_data[1], &data_get[1], 1, HAL_MAX_DELAY);

    cs_deassert();

    /* 组合两个字节为 14bit 角度值 */
    return (int16_t)((uint16_t)data_get[0] << 6 | data_get[1] >> 2);
}

int16_t MT6816::read_raw_angle() {
    return spi_read_angle();
}

/* ========================================================================== */
/*                      角度刷新方法                                           */
/* ========================================================================== */

int16_t MT6816::refresh_raw() {
    m_angle_14bit = spi_read_angle();
    return m_angle_14bit;
}

int16_t MT6816::refresh_low_pass(float alpha) {
    int16_t raw = spi_read_angle();
    /* 一阶低通滤波 */
    m_angle_14bit = (int16_t)((float)raw * alpha + (float)m_angle_14bit * (1.0f - alpha));
    return m_angle_14bit;
}

int16_t MT6816::refresh_adaptive(float max_diff, uint8_t max_dif_count) {
    int16_t raw = spi_read_angle();
    int16_t diff = raw - m_last_angle_14bit;

    if (diff > max_diff || diff < -max_diff) {
        m_angle_14bit = m_last_angle_14bit;
        m_adaptive_dif_count++;
        if (m_adaptive_dif_count >= max_dif_count) {
            m_angle_14bit = raw;
            m_adaptive_dif_count = 0;
        }
    } else {
        m_angle_14bit = raw;
    }

    m_last_angle_14bit = m_angle_14bit;
    return m_angle_14bit;
}

int16_t MT6816::refresh_rate_limit(float max_diff) {
    int16_t raw = spi_read_angle();
    int16_t diff = raw - m_last_angle_14bit;

    if (diff > max_diff) {
        m_angle_14bit = m_last_angle_14bit + (int16_t)max_diff;
    } else if (diff < -max_diff) {
        m_angle_14bit = m_last_angle_14bit - (int16_t)max_diff;
    } else {
        m_angle_14bit = raw;
    }

    m_last_angle_14bit = m_angle_14bit;
    return m_angle_14bit;
}

/* ========================================================================== */
/*                      角度值获取                                             */
/* ========================================================================== */

float MT6816::get_angle_deg() const {
    return (float)m_angle_14bit * 360.0f / RESOLUTION;
}

int16_t MT6816::get_angle_14bit() const {
    return m_angle_14bit;
}

/* ========================================================================== */
/*                      速度计算（基于 HAL_GetTick() 差值）                     */
/* ========================================================================== */

float MT6816::calc_speed_rpm() {
    uint32_t now     = HAL_GetTick();
    uint32_t dt_ms   = now - m_last_tick;

    if (m_last_tick == 0 || dt_ms == 0) {
        m_last_tick = now;
        m_last_angle_14bit = m_angle_14bit;
        return 0.0f;
    }

    int16_t delta = m_angle_14bit - m_last_angle_14bit;
    int16_t error = 5;  // 死区，忽略微小抖动

    if (delta < error && delta > -error) {
        delta = 0;
    } else if (delta < -RESOLUTION * 8 / 10) {
        /* 正向过零 */
        m_rotation_count += 1;
        delta = RESOLUTION + delta;
    } else if (delta > RESOLUTION * 8 / 10) {
        /* 反向过零 */
        m_rotation_count -= 1;
        delta = delta - RESOLUTION;
    }

    m_last_angle_14bit = m_angle_14bit;
    m_last_tick       = now;

    /* RPM = (delta / RESOLUTION) * (60000 / dt_ms) */
    return (float)delta * 60000.0f / (float)RESOLUTION / (float)dt_ms;
}

float MT6816::calc_speed_rpm_low_pass(float alpha) {
    float current_rpm = calc_speed_rpm();
    m_last_speed_rpm = current_rpm * alpha + m_last_speed_rpm * (1.0f - alpha);
    return m_last_speed_rpm;
}

/* ========================================================================== */
/*                      圈数                                                   */
/* ========================================================================== */

int32_t MT6816::get_rotation_count() const {
    return m_rotation_count;
}
