//
// Created by zhang on 2026/6/1.
//

#include "cmsis_os.h"
#include "app_task.h"
#include "OLED.h"
#include "Serial.h"
#include "FOC_2.h"
#include "main.h"
// #include "check_I.h"
// #include "MT6816.h"
#include "MT6701.h"
#include "spi.h"

extern int16_t Uq, Ud;
extern float FOC_speed;
extern uint8_t Serial_key[];
extern uint16_t Serial_value[];

static uint32_t oled_update_tick = 0;
static uint32_t serial_process_tick = 0;
static uint32_t button_scan_tick = 0;
static uint32_t debug_output_tick = 0;

#define OLED_UPDATE_PERIOD    50
#define SERIAL_PROCESS_PERIOD 10
#define BUTTON_SCAN_PERIOD    20
#define DEBUG_OUTPUT_PERIOD   100




void Task_Manager_Init(void) {
    oled_update_tick = HAL_GetTick();
    serial_process_tick = HAL_GetTick();
    button_scan_tick = HAL_GetTick();
    debug_output_tick = HAL_GetTick();
}

void FOC_error_handle_task() {
    FOC_error_handle();
}

void Task_OLED_Update(void) {
    //第一行
    OLED_ShowNum(1, 12, FOC_GET_state(), 3);
    OLED_ShowNum(2, 1, 1, 3);
    //第二行
    OLED_ShowNum(2, 5, FOC_GET_elect_angle(), 3);
    OLED_ShowNum(2, 12, FOC_location, 5);
    //第三行
    OLED_ShowNum(3, 1,elect_angle_zero, 5);
    OLED_ShowNum(3, 12, FOC_GET_elect_angle_zero(), 5);
    //第四行
    OLED_ShowNum(4, 1, Uq, 5);
    OLED_ShowNum(4, 8, Ud, 5);
}


void Task_Serial_Process(void) {
    Serial_GET_order();
    static uint16_t Serial_count =0;
    Serial_count++;
    OLED_ShowNum(4, 15,Serial_count , 2);
    FOC_SET_state(Serial_value[1]);
    Serial_Printf("[plot,%d,%d,%d]",1,10,100);
    FOC_Set_Uq(Serial_slider[0] * 100);

    if ((FOC_GET_state() == 4 || FOC_GET_state() == 5) && Serial_key[1] != 1) {
        FOC_SET_rela_location((int16_t)(Serial_slider[2]));
    }

    FOC_SET_speed_target(Serial_slider[1]);
}


void Serial_Task(void *argument) {
    Serial_Init();
    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);
        Task_Serial_Process();
    }
}
void OLED_Task(void *argument) {
    OLED_Init();
    for (;;) {
        extern uint16_t FOC_count;
    OLED_ShowNum(1,1,FOC_count, 5);
        Task_OLED_Update();
        osDelay(100);  // 延时 50ms


    }
}
void FOC_Task(void *argument) {
    FOC_Init();
    for (;;) {
        FOC_error_handle_task();
        osDelay(50);

        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        // osDelay(200);
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        // osDelay(100);
    }
}

/* ======================== MT6701 编码器测试任务 ======================== */

/**
  * @brief  MT6701 编码器测试.
  *         读取角度并显示到 OLED 第四行,
  *         串口每 1s 输出一次角度值.
  *
  * ⚠️ MT6701 的 SPI 必须配为 Mode 2 (CPOL=High, CPHA=1Edge)
  *
  * 接线:
  *   SPI2_SCK  (PB10) → MT6701 CLK
  *   SPI2_MISO (PC2)  → MT6701 DO
  *   PB11 (GPIO)      → MT6701 CSN
  */
void MT6701_Test_Task(void *argument)
{
    /* 等待外设稳定 */
    osDelay(10);

    /* 创建 MT6701 全局实例: SPI2, CS=PB11 */
    MT6701_Create(&hspi2, GPIOB, GPIO_PIN_11);

    /* 先空读一次, 让传感器稳定 */
    MT6701_Refresh();
    osDelay(5);

    for (;;)
    {
        MT6701_Refresh();
        int16_t raw   = MT6701_GetAngle();
        float   deg   = MT6701_GetDeg();
        int32_t turns = MT6701_GetTurns();

        /* OLED 第四行显示角度 */
        OLED_ShowNum(4, 1, raw, 5);           /* 14-bit 原始值 */
        OLED_ShowNum(4, 7, (uint16_t)deg, 3); /* 0~360 度 */
        OLED_ShowSignedNum(4, 11, (int16_t)(deg * 10.0f) % 10, 2); /* 小数位 */

        /* 串口每 10 次输出一次 */
        static uint8_t cnt = 0;
        cnt++;
        if (cnt >= 10) {
            cnt = 0;
            Serial_Printf("[mt6701,raw=%d,deg=%.1f,turns=%ld]\r\n",
                          raw, deg, turns);
        }

        osDelay(100);
    }
}




void App_Task_Create(void) {
    osThreadId_t appTaskHandle;
    const osThreadAttr_t appTask_attributes = {
        .name = "AppTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) 32,
    };
    appTaskHandle = osThreadNew(Serial_Task, NULL, &appTask_attributes);

    const osThreadAttr_t OLED_task_attributes = {
        .name = "OLED_task",
        .stack_size = 512  * 4,
        .priority = (osPriority_t) 40,
    };
    osThreadId_t OLED_taskHandle = osThreadNew(OLED_Task, NULL, &OLED_task_attributes);

    const osThreadAttr_t FOC_task_attributes = {
        .name = "FOC_task",
        .stack_size = 512  * 4,
        .priority = (osPriority_t) 41,
    };
    osThreadId_t FOC_taskHandle = osThreadNew(FOC_Task, NULL, &FOC_task_attributes);

    /* MT6701 编码器测试任务 */
    const osThreadAttr_t mt6701_test_attributes = {
        .name = "MT6701_Test",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) 30,
    };
    osThreadNew(MT6701_Test_Task, NULL, &mt6701_test_attributes);

}
