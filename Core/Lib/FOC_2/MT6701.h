//
// Created by zhang on 2026/6/9.
//
// MT6701 磁编码器 — SSI 通信.
//
// SSI 协议 (25 bit 帧):
//   START(1) + 14-bit angle + 4-bit status + 6-bit CRC = 25 bits total
//   使用 16-bit SPI 读取, 提取角度: (rx >> 2) & 0x3FFF
//
// ⚠️ SPI 必须配置为 Mode 2 (CPOL=1, CPHA=0):
//    CubeMX → SPI → Parameter Settings:
//      Clock Polarity (CPOL)  = High
//      Clock Phase (CPHA)    = 1 Edge
//

#ifndef ITCAP_STM_MT6701_H
#define ITCAP_STM_MT6701_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ========================================================================== */
/*               extern "C" 兼容层 (C / C++ 均可调用)                         */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

void    MT6701_Create(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
void    MT6701_Refresh(void);
int16_t MT6701_GetAngle(void);
float   MT6701_GetDeg(void);
int32_t MT6701_GetTurns(void);

#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/*               C++ 类 (仅 C++ 可见)                                         */
/* ========================================================================== */

#ifdef __cplusplus

class MT6701 {
public:
    struct Config {
        SPI_HandleTypeDef *hspi;
        GPIO_TypeDef       *cs_port;
        uint16_t            cs_pin;
    };

    explicit MT6701(const Config &cfg);

    int16_t read_raw_angle();

    int16_t refresh_raw();
    int16_t refresh_low_pass(float alpha = 0.05f);
    int16_t refresh_adaptive(float max_diff = 5.0f, uint8_t max_dif_count = 12);
    int16_t refresh_rate_limit(float max_diff = 5.0f);

    float   get_angle_deg()      const;
    int16_t get_angle_14bit()    const;

    float   calc_speed_rpm();
    float   calc_speed_rpm_low_pass(float alpha = 0.1f);

    int32_t get_rotation_count()  const;
    uint8_t get_status()          const;

    static constexpr uint16_t RESOLUTION = 16384;

private:
    SPI_HandleTypeDef *m_hspi;
    GPIO_TypeDef      *m_cs_port;
    uint16_t           m_cs_pin;

    int16_t  m_angle_14bit;
    int16_t  m_last_angle_14bit;
    float    m_last_speed_rpm;

    int32_t  m_rotation_count;
    uint8_t  m_adaptive_dif_count;

    uint32_t m_last_tick;
    uint8_t  m_last_diag;

    void cs_assert();
    void cs_deassert();
    int16_t ssi_read_angle();
};

#endif /* __cplusplus */

#endif /* ITCAP_STM_MT6701_H */
