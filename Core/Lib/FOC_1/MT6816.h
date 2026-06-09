//
// Created by zhang on 2026/3/15.
//

#ifndef BLDC_1_BLDC_MP6816_H
#define BLDC_1_BLDC_MP6816_H

#include "stm32f4xx_hal.h"
extern int16_t angle_14bit;
extern const uint16_t MT6816_limit ;

extern float angle_MT6816;
extern float Speed_rad_MT6816;
extern int32_t MT6816_rotation;

void MT6816_Init();
void MT6816_test();

//角度获取
int16_t MT6816_GetDate();
int16_t MT6816_angle_14bit_refresh();                  //会SPI,不用重复调用上一个
int16_t MT6816_angle_14bit_lowpass_refresh();
int16_t MT6816_angle_14bit_adaptive_refresh();
int16_t MT6816_angle_14bit_rate_limit_2_refresh();

float MT6816_Get_Angle();

float MT6816_Speed_rad();
float MT6816_Speed_lowpass();

#endif //BLDC_1_BLDC_MP6816_H