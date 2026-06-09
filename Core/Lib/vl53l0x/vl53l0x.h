//
// Created by zhang on 2026/4/6.
//

#ifndef FOC_1_VL53L0X_H
#define FOC_1_VL53L0X_H

#include "stm32f4xx_hal.h"

extern uint16_t VL53L0X_distance_origin;
extern uint16_t VL53L0X_distance;
extern uint16_t VL53L0X_get_data_Count;
extern uint8_t VL53L0X_state;

void VL53L0X_Init(void);
uint8_t VL53L0X_ReadReg(uint8_t RegAddress);
uint8_t VL53L0X_GetID(void);
uint16_t VL53L0X_ReadDistance(void);
void VL53L0X_refresh_distance(void);
uint16_t VL53L0X_CalibrateDistance(uint16_t rawDistance);

extern uint8_t VL53L0X_Flag_state;

#endif //FOC_1_VL53L0X_H