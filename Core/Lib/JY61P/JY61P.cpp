//
// Created on 2026/7/2.
//
// JY61P 串口六轴姿态传感器 — UART 通信.
//
// USART6 DMA CIRCULAR 连续接收, Refresh() 中扫描 DMA 缓冲解析最新帧.
//
// 帧格式 (11 bytes):
//   HEADER(2) + DATA(6) + T(1) + SUM(1)
//
//   0x55 0x51 + axL axH ayL ayH azL azH + T + Sum = 加速度
//   0x55 0x52 + wxL wxH wyL wyH wzL wzH + T + Sum = 角速度
//   0x55 0x53 + rlL rlH ptL ptH ywL ywH + T + Sum = 角度
//
//   校验: Sum = 0x55 + type + data[0..5] + T (单字节累加)
//

#include "JY61P.h"
#include <string.h>

/* ========================================================================== */
/*                              常量                                            */
/* ========================================================================== */

static constexpr float ACCEL_SCALE = 16.0f / 32768.0f;   // ±16g
static constexpr float GYRO_SCALE  = 2000.0f / 32768.0f; // ±2000°/s
static constexpr float ANGLE_SCALE = 180.0f / 32768.0f;  // ±180°

/* ========================================================================== */
/*                              JY61P 类                                      */
/* ========================================================================== */

JY61P::JY61P(const Config &cfg)
    : m_huart(cfg.huart),
      m_dma_buf(cfg.dma_buf),
      m_buf_len(cfg.buf_len),
      m_acc_x(0), m_acc_y(0), m_acc_z(0),
      m_gyro_x(0), m_gyro_y(0), m_gyro_z(0),
      m_roll(0), m_pitch(0), m_yaw(0),
      m_temp(0)
{
}

/**
  * @brief  JY61P 帧校验和。
  *         Sum = 帧头2字节 + 数据6字节 + 温度1字节, 取低8位。
  */
uint8_t JY61P::sum_check(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
  * @brief  在 DMA 缓冲中搜索指定类型的 JY61P 帧。
  * @param  type  帧类型: 0x51=accel, 0x52=gyro, 0x53=angle
  * @param  v1,v2,v3  解析出的三个 short 值
  * @param  temp      温度字节
  * @retval true  解析成功
  *         false 未找到有效帧
  */
bool JY61P::parse_frame(uint8_t type, int16_t &v1, int16_t &v2, int16_t &v3, uint8_t &temp)
{
    /* 获取 DMA 当前传输位置 (NDTR = 还剩多少没传) */
    uint16_t ndtr = (uint16_t)(__HAL_DMA_GET_COUNTER(m_huart->hdmarx));
    uint16_t write_idx = m_buf_len - ndtr;  // DMA 当前写入位置

    /* 在缓冲中向前搜索 0x55 0x{type} 帧头 */
    for (uint16_t i = 0; i < m_buf_len - 11; i++) {
        /* 从 write_idx 倒回去搜索, 找最新的一帧 */
        uint16_t idx = (write_idx >= i) ? (write_idx - i) : (m_buf_len - (i - write_idx));
        if (idx > m_buf_len - 11) continue;

        /* 检查帧头 */
        if (m_dma_buf[idx] != 0x55) continue;
        if (m_dma_buf[idx + 1] != type) continue;

        /* 校验和 */
        uint8_t calc = sum_check(&m_dma_buf[idx], 9);  // header(2) + data(6) + T(1)
        if (calc != m_dma_buf[idx + 10]) continue;      // checksum mismatch

        /* 解析 (小端) */
        v1   = (int16_t)(m_dma_buf[idx + 2] | (m_dma_buf[idx + 3] << 8));
        v2   = (int16_t)(m_dma_buf[idx + 4] | (m_dma_buf[idx + 5] << 8));
        v3   = (int16_t)(m_dma_buf[idx + 6] | (m_dma_buf[idx + 7] << 8));
        temp = m_dma_buf[idx + 8];
        return true;
    }
    return false;  /* 没找到有效帧 */
}

/**
  * @brief  刷新所有数据.
  *         在 DMA 缓冲中搜索最新的 accel/gyro/angle 帧并更新成员.
  *         通常在任务循环中周期性调用 (如 50ms).
  */
void JY61P::refresh()
{
    int16_t v1, v2, v3;
    uint8_t temp;

    /* 加速度 */
    if (parse_frame(0x51, v1, v2, v3, temp)) {
        m_acc_x = (float)v1 * ACCEL_SCALE;
        m_acc_y = (float)v2 * ACCEL_SCALE;
        m_acc_z = (float)v3 * ACCEL_SCALE;
        m_temp  = (float)temp;
    }

    /* 角速度 */
    if (parse_frame(0x52, v1, v2, v3, temp)) {
        m_gyro_x = (float)v1 * GYRO_SCALE;
        m_gyro_y = (float)v2 * GYRO_SCALE;
        m_gyro_z = (float)v3 * GYRO_SCALE;
    }

    /* 欧拉角 */
    if (parse_frame(0x53, v1, v2, v3, temp)) {
        m_roll  = (float)v1 * ANGLE_SCALE;
        m_pitch = (float)v2 * ANGLE_SCALE;
        m_yaw   = (float)v3 * ANGLE_SCALE;
    }
}

/* ========================================================================== */
/*               extern "C" 包装 (供 C 文件调用)                               */
/* ========================================================================== */

static JY61P *g_jy61p = nullptr;

extern "C" {

void JY61P_Create(UART_HandleTypeDef *huart, uint8_t *dma_buf, uint16_t buf_len)
{
    static JY61P s_inst({huart, dma_buf, buf_len});
    g_jy61p = &s_inst;
}

void JY61P_Refresh(void)
{
    if (g_jy61p) g_jy61p->refresh();
}

float JY61P_GetAccX(void)     { return g_jy61p ? g_jy61p->get_acc_x()     : 0; }
float JY61P_GetAccY(void)     { return g_jy61p ? g_jy61p->get_acc_y()     : 0; }
float JY61P_GetAccZ(void)     { return g_jy61p ? g_jy61p->get_acc_z()     : 0; }
float JY61P_GetGyroX(void)    { return g_jy61p ? g_jy61p->get_gyro_x()    : 0; }
float JY61P_GetGyroY(void)    { return g_jy61p ? g_jy61p->get_gyro_y()    : 0; }
float JY61P_GetGyroZ(void)    { return g_jy61p ? g_jy61p->get_gyro_z()    : 0; }
float JY61P_GetRoll(void)     { return g_jy61p ? g_jy61p->get_roll()      : 0; }
float JY61P_GetPitch(void)    { return g_jy61p ? g_jy61p->get_pitch()     : 0; }
float JY61P_GetYaw(void)      { return g_jy61p ? g_jy61p->get_yaw()       : 0; }
float JY61P_GetTemp(void)     { return g_jy61p ? g_jy61p->get_temp()      : 0; }

} /* extern "C" */
