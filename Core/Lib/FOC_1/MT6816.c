//
// Created by zhang on 2026/3/15.
//
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "MT6816.h"


#define  MT6816_Hardsoft_SPI         hspi1

#define MT6816_SPI_SCK_PORT          GPIOA
#define MT6816_SPI_SCK_PIN           GPIO_PIN_5

#define MT6816_SPI_MISO_PORT         GPIOA
#define MT6816_SPI_MISO_PIN          GPIO_PIN_6

#define MT6816_SPI_MOSI_PORT         GPIOB
#define MT6816_SPI_MOSI_PIN          GPIO_PIN_5

#define MT6816_SPI_CS_PORT           GPIOA
#define MT6816_SPI_CS_PIN            GPIO_PIN_4

#define CS_MT6816_start              HAL_GPIO_WritePin(MT6816_SPI_CS_PORT,MT6816_SPI_CS_PIN,GPIO_PIN_RESET );
#define CS_MT6816_stop               HAL_GPIO_WritePin(MT6816_SPI_CS_PORT,MT6816_SPI_CS_PIN,GPIO_PIN_SET );

#define limit_Getdata       16384


//角度
const uint16_t MT6816_limit = 16384;
int16_t angle_14bit;
int16_t temp_angle_14bit;
int16_t last_angle_14bit_rl = 0;
float angle_MT6816;
//角速度
int16_t Speed_14bit;
float temp_Speed_rad= 0;
float Speed_rad_MT6816;              //单位每秒0.01圈
int32_t MT6816_rotation =0;



void MT6816_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = MT6816_SPI_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MT6816_SPI_CS_PORT, &GPIO_InitStruct);


}



int16_t MT6816_GetDate() {
    CS_MT6816_start
    uint16_t swap_data[2] = {0x8300,0x8400};
    uint8_t data_get[2] = {0,0};
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)&swap_data[0],&data_get[0],1,HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)&swap_data[1],&data_get[1],1,HAL_MAX_DELAY);

    CS_MT6816_stop
    uint16_t temp =data_get[0]<<6 | data_get[1]>>2 ;
    return temp ;
}

int16_t MT6816_angle_14bit_refresh() {
   angle_14bit =   MT6816_GetDate();
    return  angle_14bit;
}


int16_t MT6816_angle_14bit_lowpass_refresh() {
    angle_14bit =   MT6816_GetDate()*1/20+angle_14bit*19/20;
    return  angle_14bit;
}

#define MAX_difference     5     //请自行计算

int16_t MT6816_angle_14bit_rate_limit_refresh() {    //
    int16_t raw = MT6816_GetDate();
    int16_t diff = raw - last_angle_14bit_rl;
    if (diff > MAX_difference) {
        angle_14bit = last_angle_14bit_rl + MAX_difference;
    } else if (diff < -MAX_difference) {
        angle_14bit = last_angle_14bit_rl - MAX_difference;
    } else {
        angle_14bit = raw;
    }

    last_angle_14bit_rl = angle_14bit;
    return angle_14bit;
}
#define  MAX_dif_count    12
int16_t MT6816_angle_14bit_adaptive_refresh() {    //
    static uint8_t dif_count = 0;
    int16_t raw = MT6816_GetDate();
    int16_t diff = raw - last_angle_14bit_rl;
    if (diff > MAX_difference || diff < -MAX_difference) {
        angle_14bit = last_angle_14bit_rl;
        dif_count ++;
        if (dif_count >= MAX_dif_count) {
            angle_14bit = raw;
            dif_count = 0;
        }
    }
    else {
        angle_14bit = raw;
    }

    last_angle_14bit_rl = angle_14bit;
    return angle_14bit;
}

int16_t angle_14bit_average[10];
uint16_t  angle_average_p = 0;

uint16_t MT6816_angle_14bit_ave_refresh() {
    angle_14bit_average[angle_average_p] = MT6816_GetDate();
    return angle_14bit;
}

float MT6816_Get_Angle() {
    angle_MT6816 = (float)angle_14bit*360/16384 ;
    return angle_MT6816;
}


void MT6816_test() {
    CS_MT6816_start

    CS_MT6816_stop
}

//测速度
#define     error   0
#define     frq   500

float MT6816_Speed_rad() {           //只能按周期调用
    int16_t temp_speed = angle_14bit - temp_angle_14bit;
    if (temp_speed< error && temp_speed>-error) {
        temp_speed = 0;
    }
    else if (temp_speed<-limit_Getdata*8/10) {
        MT6816_rotation += 1 ;
        temp_speed = limit_Getdata + temp_speed;
    }
    else if (temp_speed>limit_Getdata*8/10) {
        MT6816_rotation -= 1 ;
        temp_speed = temp_speed -limit_Getdata ;
    }
    temp_angle_14bit = angle_14bit;
    return (float)(temp_speed)*frq/limit_Getdata;
}
#define alpha 0.8
float MT6816_Speed_lowpass() {
    temp_Speed_rad = MT6816_Speed_rad()*1/10 +temp_Speed_rad*9/10;
    return temp_Speed_rad;
}