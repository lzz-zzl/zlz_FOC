//
// Created on 2026/7/2.
//
// JY61P 串口六轴姿态传感器 — UART 中断接收.
//
// 帧格式 (11 bytes/帧):
//   0x55 0x51 + ax(short) + ay(short) + az(short) + T + Sum = 加速度
//   0x55 0x52 + wx(short) + wy(short) + wz(short) + T + Sum = 角速度
//   0x55 0x53 + Roll(short)+Pitch(short)+Yaw(short)+ T + Sum = 角度
//
// 缩放:
//   accel: raw / 32768 * 16 (g)
//   gyro:  raw / 32768 * 2000 (°/s)
//   angle: raw / 32768 * 180 (°)
//

#ifndef ITCAP_STM_JY61P_H
#define ITCAP_STM_JY61P_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    JY61P_Init(UART_HandleTypeDef *huart);
void    JY61P_RxCallback(uint8_t data);    /* 在 HAL_UART_RxCpltCallback 中调用 */
void    JY61P_Refresh(void);               /* 解析最新帧 */

float   JY61P_GetAccX(void);
float   JY61P_GetAccY(void);
float   JY61P_GetAccZ(void);
float   JY61P_GetGyroX(void);
float   JY61P_GetGyroY(void);
float   JY61P_GetGyroZ(void);
float   JY61P_GetRoll(void);
float   JY61P_GetPitch(void);
float   JY61P_GetYaw(void);
float   JY61P_GetTemp(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class JY61P {
public:
    explicit JY61P(UART_HandleTypeDef *huart);

    void rx_byte(uint8_t data);   /* 中断中喂字节 */
    void refresh();                /* 任务中调用, 更新数据 */

    float get_acc_x()     const { return m_acc_x; }
    float get_acc_y()     const { return m_acc_y; }
    float get_acc_z()     const { return m_acc_z; }
    float get_gyro_x()    const { return m_gyro_x; }
    float get_gyro_y()    const { return m_gyro_y; }
    float get_gyro_z()    const { return m_gyro_z; }
    float get_roll()      const { return m_roll; }
    float get_pitch()     const { return m_pitch; }
    float get_yaw()       const { return m_yaw; }
    float get_temp()      const { return m_temp; }

private:
    UART_HandleTypeDef *m_huart;

    /* 帧解码结果 */
    float m_acc_x, m_acc_y, m_acc_z;
    float m_gyro_x, m_gyro_y, m_gyro_z;
    float m_roll, m_pitch, m_yaw;
    float m_temp;

    /* 状态机 */
    uint8_t m_frame_buf[11];
    uint8_t m_frame_idx;
    uint8_t m_frame_state;  /* 0=wait 0x55, 1=wait type, 2=collect, 3=done */

    void parse_11bytes(const uint8_t *buf);
};

#endif /* __cplusplus */

#endif /* ITCAP_STM_JY61P_H */
