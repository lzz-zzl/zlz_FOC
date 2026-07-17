//
// Created by zhang on 2026/7/2.
//

#include "JY61P.h"
#include "Serial.h"
#include <string.h>
#include <math.h>



// 接收缓冲区
uint8_t jy61p_buffer[11];
JY61P_Data_t jy61p_data;

/**
 * @brief JY61P 数据接收处理 (由串口回调调用)
 * @param data 接收到的数据
 * @param len 数据长度
 */
void jy61p_reflash(uint8_t * data) {
    for (uint8_t i = 0; i < 11; i++) {
        jy61p_buffer[i] = data[i];
    }
}

/**
 * @brief 解析加速度+温度数据包 (0x52)
 * @param buf 数据缓冲区 (11字节)
 */


void JY61P_ParseAccel(uint8_t *buf) {
    if (buf[0] != 0x55 || buf[1] != 0x52) return;

    // 提取原始数据 (小端模式)
    int16_t ax_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t ay_raw = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t az_raw = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t temp_raw = (int16_t)(buf[8] | (buf[9] << 8));

    // 存储原始16位数据
    jy61p_data.ax_16bit = ax_raw;
    jy61p_data.ay_16bit = ay_raw;
    jy61p_data.az_16bit = az_raw;
    jy61p_data.temp_raw = temp_raw;
}
void JY61P_ParseAccel_16bit(uint8_t *buf){
    // if (buf[0] != 0x55 || buf[1] != 0x52) return;

    int16_t ax_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t ay_raw = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t az_raw = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t temp_raw = (int16_t)(buf[8] | (buf[9] << 8));

    jy61p_data.ax = ax_raw / 32768.0f * 16.0f;
    jy61p_data.ay = ay_raw / 32768.0f * 16.0f;
    jy61p_data.az = az_raw / 32768.0f * 16.0f;
    jy61p_data.temp = temp_raw * 0.1f;
}

/**
 * @brief 解析角速度数据包 (0x53)
 * @param buf 数据缓冲区 (10字节)
 */
// static void JY61P_ParseGyro(uint8_t *buf) {
//     if (buf[0] != 0x55 || buf[1] != 0x53) return;
//
//     // 提取原始数据 (小端模式)
//     int16_t gx_raw = (int16_t)(buf[2] | (buf[3] << 8));
//     int16_t gy_raw = (int16_t)(buf[4] | (buf[5] << 8));
//     int16_t gz_raw = (int16_t)(buf[6] | (buf[7] << 8));
//
//     // 存储原始16位数据
//     jy61p_data.gx_raw = gx_raw;
//     jy61p_data.gy_raw = gy_raw;
//     jy61p_data.gz_raw = gz_raw;
// }
void JY61P_ParseGyro_16bit(uint8_t *buf) {
    // 提取原始数据 (小端模式)
    int16_t gx_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t gy_raw = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t gz_raw = (int16_t)(buf[6] | (buf[7] << 8));

    // 存储原始值
    jy61p_data.gx_raw = gx_raw;
    jy61p_data.gy_raw = gy_raw;
    jy61p_data.gz_raw = gz_raw;

    // 角速度: 32768 = 2000°/s (默认±2000°/s量程)
    jy61p_data.gx = gx_raw / 32768.0f * 2000.0f;
    jy61p_data.gy = gy_raw / 32768.0f * 2000.0f;
    jy61p_data.gz = gz_raw / 32768.0f * 2000.0f;
}

/**
 * @brief 解析角度数据包 (0x54)
 * @param buf 数据缓冲区 (10字节)
 */
void JY61P_ParseAngle_16bit(uint8_t *buf) {
    // if (buf[0] != 0x55 || buf[1] != 0x53) return;

    // 提取原始数据 (小端模式)
    int16_t roll_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t pitch_raw = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t yaw_raw = (int16_t)(buf[6] | (buf[7] << 8));

    // 存储原始16位数据
    jy61p_data.roll_raw = roll_raw;
    jy61p_data.pitch_raw = pitch_raw;
    jy61p_data.yaw_raw = yaw_raw;

    // 角度: 32768 = 180°
    jy61p_data.roll = roll_raw / 32768.0f * 180.0f;
    jy61p_data.pitch = pitch_raw / 32768.0f * 180.0f;
    jy61p_data.yaw = yaw_raw / 32768.0f * 180.0f;
}
// static void JY61P_ParseAngle(uint8_t *buf) {
//     // 角度: 32768 = 180°
//     if (buf[0] != 0x55 || buf[1] != 0x54) return;
//
//     // 提取原始数据 (小端模式)
//     int16_t roll_raw = (int16_t)(buf[2] | (buf[3] << 8));
//     int16_t pitch_raw = (int16_t)(buf[4] | (buf[5] << 8));
//     int16_t yaw_raw = (int16_t)(buf[6] | (buf[7] << 8));
//
//     jy61p_data.roll = roll_raw / 32768.0f * 180.0f;
//     jy61p_data.pitch = pitch_raw / 32768.0f * 180.0f;
//     jy61p_data.yaw = yaw_raw / 32768.0f * 180.0f;
// }

/**
 * @brief 获取加速度数据 (单位: g)
 * @param ax X轴加速度指针
 * @param ay Y轴加速度指针
 * @param az Z轴加速度指针
 */
void JY61P_GetAccel(float *ax, float *ay, float *az) {
    if (ax) *ax = jy61p_data.ax;
    if (ay) *ay = jy61p_data.ay;
    if (az) *az = jy61p_data.az;
}

/**
 * @brief 获取加速度原始数据 (16位ADC值)
 * @param ax X轴原始值指针
 * @param ay Y轴原始值指针
 * @param az Z轴原始值指针
 */
void JY61P_GetAccelRaw(int16_t *ax, int16_t *ay, int16_t *az) {
    if (ax) *ax = jy61p_data.ax_16bit;
    if (ay) *ay = jy61p_data.ay_16bit;
    if (az) *az = jy61p_data.az_16bit;
}

/**
 * @brief 获取角速度数据 (单位: °/s)
 * @param gx X轴角速度指针
 * @param gy Y轴角速度指针
 * @param gz Z轴角速度指针
 */
void JY61P_GetGyro(float *gx, float *gy, float *gz) {
    if (gx) *gx = jy61p_data.gx;
    if (gy) *gy = jy61p_data.gy;
    if (gz) *gz = jy61p_data.gz;
}

/**
 * @brief 获取角速度原始数据 (16位ADC值)
 * @param gx X轴原始值指针
 * @param gy Y轴原始值指针
 * @param gz Z轴原始值指针
 */
void JY61P_GetGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz) {
    if (gx) *gx = jy61p_data.gx_raw;
    if (gy) *gy = jy61p_data.gy_raw;
    if (gz) *gz = jy61p_data.gz_raw;
}



/**
 * @brief 获取角度数据 (单位: °)
 * @param roll 横滚角指针
 * @param pitch 俯仰角指针
 * @param yaw 偏航角指针
 */
void JY61P_GetAngle(float *roll, float *pitch, float *yaw) {
    if (roll) *roll = jy61p_data.roll;
    if (pitch) *pitch = jy61p_data.pitch;
    if (yaw) *yaw = jy61p_data.yaw;
}

/**
 * @brief 获取角度原始数据 (16位ADC值)
 * @param roll 横滚角原始值指针
 * @param pitch 俯仰角原始值指针
 * @param yaw 偏航角原始值指针
 */
void JY61P_GetAngleRaw(int16_t *roll, int16_t *pitch, int16_t *yaw) {
    if (roll) *roll = jy61p_data.roll_raw;
    if (pitch) *pitch = jy61p_data.pitch_raw;
    if (yaw) *yaw = jy61p_data.yaw_raw;
}

/**
 * @brief 获取温度数据 (单位: ℃)
 * @return 温度值
 */
float JY61P_GetTemp(void) {
    return jy61p_data.temp;
}

/**
 * @brief 获取温度原始数据 (16位ADC值)
 * @return 温度原始值
 */
int16_t JY61P_GetTempRaw(void) {
    return jy61p_data.temp_raw;
}

/**
 * @brief 获取帧计数
 * @return 帧计数
 */
uint8_t JY61P_GetFrameCount(void) {
    return jy61p_data.frame_count;
}

/**
 * @brief 获取所有数据 (一次性获取全部)
 * @param data 数据指针
 */
void JY61P_GetAllData(JY61P_Data_t *data) {
    if (data) {
        memcpy(data, &jy61p_data, sizeof(JY61P_Data_t));
    }
}

/**
 * @brief 重置帧计数
 */
void JY61P_ResetFrameCount(void) {
    jy61p_data.frame_count = 0;
}

// 状态机相关变量
typedef enum {
    STATE_IDLE,          // 空闲状态，等待帧头
    STATE_HEADER,        // 已接收帧头0x55，等待数据类型
    STATE_DATA           // 接收数据中
} JY61P_RxState_t;

static JY61P_RxState_t rx_state = STATE_IDLE;
static uint8_t rx_index = 0;
static uint8_t expected_len = 11;  // 固定11字节

void jy61p_byte_receive(uint8_t data) {
    switch (rx_state) {
        case STATE_IDLE:
            // 等待帧头 0x55
            if (data == 0x55) {
                jy61p_buffer[0] = data;
                rx_index = 1;
                rx_state = STATE_HEADER;
            }
            break;

        case STATE_HEADER:
            // 已收到0x55，检查第二个字节是否是有效数据类型 (0x51-0x54)
            if ((data >= 0x51 && data <= 0x54)) {
                jy61p_buffer[1] = data;
                rx_index = 2;
                rx_state = STATE_DATA;
            } else if (data == 0x55) {
                // 连续两个0x55，重新开始
                jy61p_buffer[0] = data;
                rx_index = 1;
                rx_state = STATE_HEADER;
            } else {
                // 无效数据，回到空闲状态
                rx_state = STATE_IDLE;
                rx_index = 0;
            }
            break;

        case STATE_DATA:
            // 接收剩余数据 (第2到第10字节)
            jy61p_buffer[rx_index++] = data;

            // 判断是否接收完整一帧 (11字节)
            if (rx_index >= 11) {
                uint8_t type = jy61p_buffer[1];
                if (type == 0x51) {
                    JY61P_ParseAccel_16bit(jy61p_buffer);
                } else if (type == 0x52) {
                    JY61P_ParseGyro_16bit(jy61p_buffer);
                } else if (type == 0x53) {
                    JY61P_ParseAngle_16bit(jy61p_buffer);
                }
                jy61p_data.frame_count++;

                // 重置状态机
                rx_state = STATE_IDLE;
                rx_index = 0;
            }
            break;

        default:
            rx_state = STATE_IDLE;
            rx_index = 0;
            break;
    }
}