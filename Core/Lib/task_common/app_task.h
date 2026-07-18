
// Created by zhang on 2026/6/1.
//

#ifndef FOC_RTOS_APP_TASK_H
#define FOC_RTOS_APP_TASK_H


#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Task_Manager_Init(void);
void Task_Manager_Run(void);

void Task_OLED_Update(void);
void Task_Serial_Process(void);

void App_Task(void *argument);
void App_Task_Create(void);

    void Sin_control();

#ifdef __cplusplus
}
#endif



#endif //FOC_RTOS_APP_TASK_H