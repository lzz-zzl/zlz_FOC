//
// Created on 2026/7/2.
//
// JY61P 串口六轴姿态传感器 — UART 中断接收.
//
// 帧格式 (11 bytes):
//   0x55 + type + data(6) + temp(2) + Sum(1)
//
//   状态机: 掉头 0x55 → 收 type + 9 字节 → 校验 → 解析
//

#include "JY61P.h"
#include <string.h>

/* ========================================================================== */
/*                              常量                                            */
/* ========================================================================== */

static constexpr float ACCEL_SCALE = 16.0f / 32768.0f;
static constexpr float GYRO_SCALE  = 2000.0f / 32768.0f;
static constexpr float ANGLE_SCALE = 180.0f / 32768.0f;

/* ========================================================================== */
/*                              JY61P 类                                      */
/* ========================================================================== */

JY61P::JY61P(UART_HandleTypeDef *huart)
    : m_huart(huart),
      m_acc_x(0), m_acc_y(0), m_acc_z(0),
      m_gyro_x(0), m_gyro_y(0), m_gyro_z(0),
      m_roll(0), m_pitch(0), m_yaw(0),
      m_temp(0),
      m_frame_idx(0),
      m_frame_state(0)
{
}

/**
  * @brief  校验和: Sum = 帧头2 + 数据6 + 温度2 = 10 字节累加, 取低 8 位.
  */
static uint8_t sum_check(const uint8_t *data)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++) {
        sum += data[i];
    }
    return sum;
}

/**
  * @brief  解析 11 字节完整帧, 更新对应数据.
  */
void JY61P::parse_11bytes(const uint8_t *buf)
{
    uint8_t type = buf[1];

    /* 提取三个 short (小端) */
    int16_t v1 = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t v2 = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t v3 = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t temp_raw = (int16_t)(buf[8] | (buf[9] << 8));

    switch (type) {
    case 0x51:  /* 加速度 */
        m_acc_x = (float)v1 * ACCEL_SCALE;
        m_acc_y = (float)v2 * ACCEL_SCALE;
        m_acc_z = (float)v3 * ACCEL_SCALE;
        m_temp  = (float)temp_raw;
        break;

    case 0x52:  /* 角速度 */
        m_gyro_x = (float)v1 * GYRO_SCALE;
        m_gyro_y = (float)v2 * GYRO_SCALE;
        m_gyro_z = (float)v3 * GYRO_SCALE;
        break;

    case 0x53:  /* 欧拉角 */
        m_roll  = (float)v1 * ANGLE_SCALE;
        m_pitch = (float)v2 * ANGLE_SCALE;
        m_yaw   = (float)v3 * ANGLE_SCALE;
        break;

    default:
        break;
    }
}

/**
  * @brief  中断中喂一个字节.
  *         状态机: 掉头 0x55 → 收满 11 字节 → 校验 → 解析.
  *         必须在中断回调中调用 (如 HAL_UART_RxCpltCallback).
  */
void JY61P::rx_byte(uint8_t data)
{
    switch (m_frame_state) {

    case 0:  /* 等待帧头 0x55 */
        if (data == 0x55) {
            m_frame_buf[0] = data;
            m_frame_idx = 1;
            m_frame_state = 1;
        }
        break;

    case 1:  /* 等待 type (0x51/0x52/0x53) */
        if (data == 0x51 || data == 0x52 || data == 0x53) {
            m_frame_buf[1] = data;
            m_frame_idx = 2;
            m_frame_state = 2;
        } else if (data == 0x55) {
            /* 连续 0x55, 重置起始位置 */
            m_frame_buf[0] = 0x55;
            m_frame_idx = 1;
        } else {
            m_frame_state = 0;  /* 无效, 重新掉头 */
        }
        break;

    case 2:  /* 收剩余 9 字节 (data[6] + T[1] + Sum[1]) */
        m_frame_buf[m_frame_idx++] = data;
        if (m_frame_idx >= 11) {
            /* 校验和 */
            if (sum_check(m_frame_buf) == m_frame_buf[10]) {
                parse_11bytes(m_frame_buf);
            }
            m_frame_state = 0;  /* 继续下一帧 */
        }
        break;

    default:
        m_frame_state = 0;
        break;
    }
}

/**
  * @brief  refresh 函数 (保留兼容, 当前无 DMA 缓冲, 数据已在中断中在线解析).
  *         后续可在此加滤波或降采样.
  */
void JY61P::refresh()
{
    /* 数据已在 rx_byte 中实时解析, 无需额外操作 */
}

/* ========================================================================== */
/*               extern "C" 包装                                              */
/* ========================================================================== */

static JY61P *g_jy61p = nullptr;

extern "C" {

void JY61P_Init(UART_HandleTypeDef *huart)
{
    static JY61P s_inst(huart);
    g_jy61p = &s_inst;
}

void JY61P_RxCallback(uint8_t data)
{
    if (g_jy61p) g_jy61p->rx_byte(data);
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
