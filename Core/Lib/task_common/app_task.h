
// Created by zhang on 2026/6/1.
//

#ifndef FOC_RTOS_APP_TASK_H
#define FOC_RTOS_APP_TASK_H


#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "Control.h"

extern int16_t test_count121;

    extern control_object ball_location_O;
    extern control_PID ball_balance_PID;

    /* 按键事件标志 (PE1=KEY1, PE3=KEY3) — 消费后请手动清 0 */
    extern volatile uint8_t KEY1_pressed;
    extern volatile uint8_t KEY3_pressed;
    void Key_Scan(void);

void Task_Manager_Init(void);
void Task_Manager_Run(void);

void Task_OLED_Update(void);
void Task_Serial_Process(void);

void App_Task(void *argument);
void App_Task_Create(void);

    void Sin_control();
  void  control_trace();
    void ball_balance();

#ifdef __cplusplus
}
#endif



#endif //FOC_RTOS_APP_TASK_H