//
// Created by zhang on 2026/3/23.
//

#ifndef BLDC_1_DRV8313_H
#define BLDC_1_DRV8313_H

#include "stm32f4xx_hal.h"

void DRV8313_Init(void);
void DRV8313_CHOOSE_ENABLE(uint8_t if_enable);

#endif //BLDC_1_DRV8313_H