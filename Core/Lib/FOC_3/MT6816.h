//
// Created by zhang on 2026/6/3.
//

#ifndef FOC_RTOS_MT6816_H
#define FOC_RTOS_MT6816_H

#include "stm32f4xx_hal.h"

class MT6816 {
public:
    /* ---------- SPI / CS 引脚参数 ---------- */
    struct Config {
        SPI_HandleTypeDef *hspi;
        GPIO_TypeDef       *cs_port;
        uint16_t            cs_pin;
    };

    explicit MT6816(const Config &cfg);

    /* ---------- 原始 SPI 读取 ---------- */
    int16_t read_raw_angle();

    /* ---------- 带滤波的角度刷新（返回 14bit 角度） ---------- */
    int16_t refresh_raw();                         // 直接返回
    int16_t refresh_low_pass(float alpha = 0.05f);  // alpha 越小滤波越强
    int16_t refresh_adaptive(float max_diff = 5.0f, uint8_t max_dif_count = 12);
    int16_t refresh_rate_limit(float max_diff = 5.0f);

    /* ---------- 角度值 ---------- */
    float   get_angle_deg()      const;  // 0~360 度
    int16_t get_angle_14bit()    const;  // 0~16383

    /* ---------- 速度 ---------- */
    float   calc_speed_rpm();     // 通过 HAL_GetTick() 差值计算，单位 RPM
    float   calc_speed_rpm_low_pass(float alpha = 0.1f);

    /* ---------- 圈数 ---------- */
    int32_t get_rotation_count()  const;

    static constexpr uint16_t RESOLUTION = 16384;  // 14 bit

private:
    SPI_HandleTypeDef *m_hspi;
    GPIO_TypeDef      *m_cs_port;
    uint16_t           m_cs_pin;

    int16_t  m_angle_14bit;         // 当前角度
    int16_t  m_last_angle_14bit;    // 上一次角度（用于差速 / 圈数累计）
    float    m_last_speed_rpm;      // 上一次速度（低通用）

    int32_t  m_rotation_count;      // 累计圈数
    uint8_t  m_adaptive_dif_count;  // 自适应滤波异常计数器

    uint32_t m_last_tick;           // 上一次读取的系统 tick (ms)

    void cs_assert();
    void cs_deassert();
    int16_t spi_read_angle();
};

#endif //FOC_RTOS_MT6816_H
