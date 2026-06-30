//
// Created by zhang on 2026/6/9.
//
// MT6701 磁编码器 — SSI 通信.
//
// 时序:
//   1) CS↓ (assert)
//   2) 等待 T_CLK_SETUP (~2 µs)
//   3) 发送 16 个 SCK 脉冲 (SPI 16-bit dummy transfer), 同时接收 MISO 数据
//   4) CS↑ (deassert)
//   5) 等待 T_CS (~4 µs) 后允许下一次读取
//
// SSI 帧 (25 bits, MSB first):
//   bit[24]    = START (always 1)
//   bit[23:10] = 14-bit angle [13:0]
//   bit[9:6]   = 4-bit status (field strength, etc.)
//   bit[5:0]   = 6-bit CRC
//
// 16-bit SPI 读取截取前 16 bits:
//   read[15]   = START
//   read[14:1] = angle [13:0]
//   read[0]    = 第一个 status 位
//   角度提取:  (rx >> 2) & 0x3FFF
//
// ⚠️ CubeMX 中使用的 SPI 必须设为 Mode 2 (CPOL=High, CPHA=1Edge):
//    Clock Polarity (CPOL)  = High
//    Clock Phase   (CPHA)   = 1 Edge
//

#include "MT6701.h"

/* ========================================================================== */
/*                          微秒级延时 (死循环近似)                            */
/* ========================================================================== */

/* 粗略延时: 168MHz 下约 1 µs (根据实际编译优化可能需要调整) */
static inline void delay_us(uint32_t us) {
    for (uint32_t i = 0; i < us * 42; i++) {
        __NOP();
    }
}

/* ========================================================================== */
/*                              构造函数                                       */
/* ========================================================================== */

MT6701::MT6701(const Config &cfg)
    : m_hspi(cfg.hspi),
      m_cs_port(cfg.cs_port),
      m_cs_pin(cfg.cs_pin),
      m_angle_14bit(0),
      m_last_angle_14bit(0),
      m_last_speed_rpm(0.0f),
      m_rotation_count(0),
      m_adaptive_dif_count(0),
      m_last_tick(0),
      m_last_diag(1)
{
    /* 配置 CS 引脚为推挽输出 */
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
/*                         SSI 原始读取 (16-bit SPI)                          */
/* ========================================================================== */

/**
  * @brief  通过 SSI 读取 14-bit 角度值.
  *
  * SPI 配置要求:
  *   - 数据宽度: 8-bit 或 16-bit (16-bit 效率更高)
  *   - CPOL=1, CPHA=0 (Mode 2): SCK 空闲高, 下降沿采样
  *   - MSB first
  *   - 速率建议 1~5 MHz
  *
  * @retval 14-bit 角度值 (0~16383)
  */
int16_t MT6701::ssi_read_angle() {
    cs_assert();

    /* T_CLK_SETUP: CS 低后等待 ~2 µs 让传感器准备好 */
    delay_us(2);

    /* 发送 dummy 并接收数据: 8-bit 宽度传 2 字节 (兼容所有 SPI 配置) */
    uint8_t tx_b[2] = {0x00, 0x00};
    uint8_t rx_b[2] = {0, 0};
    if (HAL_SPI_TransmitReceive(m_hspi, tx_b, rx_b, 2, 100) != HAL_OK) {
        cs_deassert();
        return m_angle_14bit;  /* 读取失败, 返回上次值 */
    }
    uint16_t rx = ((uint16_t)rx_b[0] << 8) | rx_b[1];

    cs_deassert();

    /* T_CS: CS 高后保持至少 ~3 µs 再允许下次读取 */
    delay_us(3);

    /* 角度提取:
     *   rx[15]   = START (同步位, 恒为 1)
     *   rx[14:1] = 14-bit angle [13:0]
     *   rx[0]    = status[3] (第一个 status 位)
     *
     *   (rx >> 1)  将 14-bit angle (rx[14:1]) 移至 bits[13:0]
     *   & 0x3FFF  保留低 14 位, 丢弃 START (移位后在 bit 14)
     *
     *   ⚠️ 保持 8-bit SPI 模式 (SPI_DATASIZE_8BIT):
     *      HAL_SPI_TransmitReceive(..., 2, ...) = 16 SCK pulses ✅
     *      16-bit 模式下需改 count=1, 否则 32 SCK pulses 读过头!
     */
    return (int16_t)((rx >> 1) & 0x3FFF);
}

int16_t MT6701::read_raw_angle() {
    return ssi_read_angle();
}

/* ========================================================================== */
/*                      角度刷新方法                                           */
/* ========================================================================== */

int16_t MT6701::refresh_raw() {
    m_angle_14bit = ssi_read_angle();
    return m_angle_14bit;
}

int16_t MT6701::refresh_low_pass(float alpha) {
    int16_t raw = ssi_read_angle();
    m_angle_14bit = (int16_t)((float)raw * alpha + (float)m_angle_14bit * (1.0f - alpha));
    return m_angle_14bit;
}

int16_t MT6701::refresh_adaptive(float max_diff, uint8_t max_dif_count) {
    int16_t raw = ssi_read_angle();
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

int16_t MT6701::refresh_rate_limit(float max_diff) {
    int16_t raw = ssi_read_angle();
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

float MT6701::get_angle_deg() const {
    return (float)m_angle_14bit * 360.0f / RESOLUTION;
}

int16_t MT6701::get_angle_14bit() const {
    return m_angle_14bit;
}

/* ========================================================================== */
/*                      速度计算（基于 HAL_GetTick() 差值）                     */
/* ========================================================================== */

float MT6701::calc_speed_rpm() {
    uint32_t now     = HAL_GetTick();
    uint32_t dt_ms   = now - m_last_tick;

    if (m_last_tick == 0 || dt_ms == 0) {
        m_last_tick = now;
        m_last_angle_14bit = m_angle_14bit;
        return 0.0f;
    }

    int16_t delta = m_angle_14bit - m_last_angle_14bit;
    int16_t error = 2;  // 死区，忽略微小抖动

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

/**
  * @brief  诊断状态.
  *         当前仅返回默认值 1 (正常).
  *         如需完整解析状态/CRC, 需改用 24-bit SPI 读取完整 SSI 帧.
  */
uint8_t MT6701::get_status() const {
    return m_last_diag;
}

/* ========================================================================== */
/*               extern "C" 包装 (供 C 文件调用)                               */
/* ========================================================================== */

/* 全局静态实例: 在 MT6701_Create() 中构造 */
static MT6701 *g_mt6701 = nullptr;

extern "C" {

void MT6701_Create(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    static MT6701 s_inst({hspi, cs_port, cs_pin});
    g_mt6701 = &s_inst;
}

void MT6701_Refresh(void)
{
    if (g_mt6701) g_mt6701->refresh_raw();
}

int16_t MT6701_GetAngle(void)
{
    return g_mt6701 ? g_mt6701->get_angle_14bit() : 0;
}

float MT6701_GetDeg(void)
{
    return g_mt6701 ? g_mt6701->get_angle_deg() : 0.0f;
}

int32_t MT6701_GetTurns(void)
{
    return g_mt6701 ? g_mt6701->get_rotation_count() : 0;








































































































































































































































































}

} /* extern "C" */
