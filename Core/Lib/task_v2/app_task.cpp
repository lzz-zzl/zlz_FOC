//
// Created by zhang on 2026/6/1.
// ———— C++ 版本 ————
//

#include "app_task.h"      // 已有 extern "C" 守卫
#include "cmsis_os.h"      // CMSIS 头文件自带 extern "C"

// C 库头文件 — 没有 extern "C" 守卫，需要手动包裹
extern "C" {
#include "OLED.h"
#include "Serial.h"
#include "FOC_2.h"
#include "MT6701.h"
#include "JY61P.h"
#include "spi.h"
#include "usart.h"
}

#include <cstdint>

// ====================================================================
// 配置常量
// ====================================================================
namespace config {
    constexpr uint32_t OLED_UPDATE_MS    = 50;
    constexpr uint32_t SERIAL_POLL_MS    = 10;
    constexpr uint32_t BUTTON_SCAN_MS    = 20;
    constexpr uint32_t DEBUG_OUTPUT_MS   = 100;

    constexpr uint32_t SERIAL_STACK_SIZE = 256 * 4;
    constexpr uint32_t OLED_STACK_SIZE   = 512 * 4;
    constexpr uint32_t FOC_STACK_SIZE    = 512 * 4;
    constexpr uint32_t MT6701_STACK_SIZE = 256 * 4;

    constexpr uint8_t  SERIAL_PRIORITY   = 20;
    constexpr uint8_t  OLED_PRIORITY     = 38;
    constexpr uint8_t  FOC_PRIORITY      = 41;
    constexpr uint8_t  MT6701_PRIORITY   = 30;
}

// ====================================================================
// 外部全局变量声明
// ====================================================================
extern int16_t  Uq, Ud;
extern float    FOC_speed;
extern uint16_t FOC_count;
extern uint8_t  Serial_key[];
extern uint16_t Serial_value[];

// ====================================================================
// 任务函数声明
// ====================================================================
static void SerialTaskFunc(void* argument);
static void OLEDTaskFunc(void* argument);
static void FOCTaskFunc(void* argument);
static void MT6701TaskFunc(void* argument);
static void JY61PTestTask(void* argument);

// ====================================================================
// 任务函数实现
// ====================================================================

static void OLEDTaskFunc(void* /*argument*/) {
    OLED_Init();
    for (;;) {
        OLED_ShowNum(1, 1, FOC_count, 5);

        OLED_ShowNum(2, 12, FOC_GET_state(), 3);
        OLED_ShowString(2, 5, "deg");
        OLED_ShowNum(3, 1, FOC_location, 5);
        OLED_ShowNum(2, 1, FOC_GET_elect_angle(), 3);

        osDelay(100);
    }
}

static void SerialTaskFunc(void* /*argument*/) {
    Serial_Init();
    // osDelay(200);  // 等外设稳定
    Serial_Printf("[boot]USART1 ready at 115200\r\n");
    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);

        Serial_GET_order();

        static uint16_t serial_count = 0;
        serial_count++;
        OLED_ShowNum(4, 15, serial_count, 2);

        FOC_SET_state(Serial_value[1]);
        Serial_Printf("[plot,%d,%d,%d]", 1, 10, 100);
        FOC_Set_Uq(Serial_slider[0] * 100);

        if ((FOC_GET_state() == 4 || FOC_GET_state() == 5) && Serial_key[1] != 1) {
            FOC_SET_rela_location(static_cast<int16_t>(Serial_slider[2]));
        }

        FOC_SET_speed_target(Serial_slider[1]);
    }
}

static void Serial_debug(void * /*argument*/) {
    for (;;) {
        static  float count = 0;
        count+= 0.01;
        if (count>3) {
            count = -3;
        }
        Serial_JustFloat(&count,1);
        osDelay(10);
    }
}


static void FOCTaskFunc(void* /*argument*/) {
    FOC_Init();
    for (;;) {
        FOC_error_handle();
        Serial_Printf("[plot,%d,%d,%d]\r\n", FOC_Ua, FOC_Ub, FOC_Uc);
        osDelay(50);
    }
}

static void MT6701TaskFunc(void* /*argument*/) {
    osDelay(10);
    MT6701_Create(&hspi1, GPIOD, GPIO_PIN_7);
    MT6701_Refresh();
    osDelay(5);

    for (;;) {
        MT6701_Refresh();
        int16_t raw   = MT6701_GetAngle();
        float   deg   = MT6701_GetDeg();
        MT6701_GetTurns();

        OLED_ShowNum(3, 7,MT6701_GetAngle(), 5);
        // OLED_ShowSignedNum(4, 11,
        //     static_cast<int16_t>(static_cast<int>(deg * 10.0f) % 10), 2);

        Serial_Printf("[plot,%d]",raw);
        osDelay(40);
    }
}

static void JY61PTestTask(void* /*argument*/) {
    osDelay(200);
    for (;;) {
        // OLED_ShowNum(4, 1, static_cast<uint16_t>(JY61P_GetRoll() * 10), 4);
        // OLED_ShowNum(4, 7, static_cast<uint16_t>(JY61P_GetPitch() * 10), 4);
        // OLED_ShowNum(4, 13, static_cast<uint16_t>(JY61P_GetYaw() * 10), 4);
        // Serial_Printf("[jy61p]r=%.1f p=%.1f y=%.1f\r\n",
        //               (double)JY61P_GetRoll(),
        //               (double)JY61P_GetPitch(),
        //               (double)JY61P_GetYaw());
        osDelay(100);
    }
}

// ====================================================================
// FreeRTOS 任务创建入口
// ====================================================================
static void CreateTasks() {
    struct TaskDef {
        const char*    name;
        osThreadFunc_t func;
        uint32_t       stack;
        uint8_t        priority;
    };

    constexpr TaskDef tasks[] = {
        // { "SerialTask",  &SerialTaskFunc,  config::SERIAL_STACK_SIZE, config::SERIAL_PRIORITY },
        {"Serial_debug", &Serial_debug,     256 * 4,                   40},
        { "OLED_Task",   &OLEDTaskFunc,    config::OLED_STACK_SIZE,   config::OLED_PRIORITY },
        // { "FOC_Task",    &FOCTaskFunc,     config::FOC_STACK_SIZE,    config::FOC_PRIORITY },
        { "MT6701_Test", &MT6701TaskFunc,  config::MT6701_STACK_SIZE, config::MT6701_PRIORITY },
        // { "JY61P_Test",  &JY61PTestTask,   256 * 4,                  30 },
    };

    for (const auto& t : tasks) {
        osThreadAttr_t attr{};
        attr.name       = t.name;
        attr.stack_size = t.stack;
        attr.priority   = osPriority_t(t.priority);
        osThreadNew(t.func, nullptr, &attr);
    }
}

// ====================================================================
// extern "C" 接口 — 与 C 头文件声明匹配
// ====================================================================
void Task_Manager_Init(void)  { /* reserved */ }
void Task_Manager_Run(void)   { /* reserved */ }
void Task_OLED_Update(void)   { /* removed — OLED update moved to OLEDTaskFunc */ }
void Task_Serial_Process(void){ /* removed — serial process moved to SerialTaskFunc */ }
void App_Task(void* argument) { /* deprecated — tasks created in App_Task_Create */ }
void App_Task_Create(void)    { CreateTasks(); }