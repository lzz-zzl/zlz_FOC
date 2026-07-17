//
// Created by zhang on 2026/7/2.
//

#ifndef TI_FOC_JY61P_H
#define TI_FOC_JY61P_H

#include <stdint.h>

// JY61P 数据结构
typedef struct {
    //加速度
    float ax;      // X轴加速度 (g)
    float ay;      // Y轴加速度 (g)
    float az;      // Z轴加速度 (g)

    int16_t ax_16bit;
    int16_t ay_16bit;
    int16_t az_16bit;

     // 温度 (℃)
    float temp;

    //角度
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;

    float roll;
    float pitch;
    float yaw;

    //角速度 (°/s)
    float gx;
    float gy;
    float gz;

    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;

    int16_t temp_raw;
    uint8_t frame_count;
} JY61P_Data_t;

extern JY61P_Data_t jy61p_data;

// 函数声明
void jy61p_byte_receive(uint8_t data);

void jy61p_reflash(uint8_t *data);
void JY61P_ParseAccel(uint8_t *buf);
void JY61P_ParseAccel_16bit(uint8_t *buf);
void JY61P_ParseGyro_16bit(uint8_t *buf);
void JY61P_ParseAngle_16bit(uint8_t *buf);

// 获取数据函数
void JY61P_GetAccel(float *ax, float *ay, float *az);
void JY61P_GetAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);
void JY61P_GetGyro(float *gx, float *gy, float *gz);
void JY61P_GetGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);
void JY61P_GetAngle(float *roll, float *pitch, float *yaw);
void JY61P_GetAngleRaw(int16_t *roll, int16_t *pitch, int16_t *yaw);
float JY61P_GetTemp(void);
int16_t JY61P_GetTempRaw(void);
uint8_t JY61P_GetFrameCount(void);
void JY61P_GetAllData(JY61P_Data_t *data);
void JY61P_ResetFrameCount(void);

#endif //TI_FOC_JY61P_H