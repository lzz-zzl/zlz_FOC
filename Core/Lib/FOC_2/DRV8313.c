//
// Created by zhang on 2026/3/23.
//

#include "DRV8313.h"
#include "stm32f4xx_hal.h"

#define DVR8313_ENABLE_PORT          GPIOE
#define DVR8313_ENABLE_PIN           GPIO_PIN_8

#define DVR8313_FAULT_PORT           GPIOA
#define DVR8313_FAULT_PIN            GPIO_PIN_11

#define DVR8313_RESET_PORT           GPIOA
#define DVR8313_RESET_PIN            GPIO_PIN_1

#define DVR8313_SLEEP_PORT           GPIOA
#define DVR8313_SLEEP_PIN            GPIO_PIN_2







void DRV8313_RESET() {
    HAL_GPIO_WritePin(DVR8313_RESET_PORT, DVR8313_RESET_PIN, 0);
    HAL_Delay(1);
    HAL_GPIO_WritePin(DVR8313_RESET_PORT, DVR8313_RESET_PIN, 1);
}

void DRV8313_CHOOSE_ENABLE(uint8_t if_enable) {
    HAL_GPIO_WritePin(DVR8313_ENABLE_PORT, DVR8313_ENABLE_PIN, if_enable);   //置1位启动
}

void DRV8313_Init() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DVR8313_ENABLE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DVR8313_ENABLE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DVR8313_FAULT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DVR8313_FAULT_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DVR8313_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DVR8313_RESET_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DVR8313_SLEEP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DVR8313_SLEEP_PORT, &GPIO_InitStruct);

    DRV8313_RESET();

}