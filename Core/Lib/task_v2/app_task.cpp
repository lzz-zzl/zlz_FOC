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
#include "spi.h"
}

#include <cstdint>

// ====================================================================
// 配置常量 (constexpr 替代 #define)
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

    constexpr uint8_t  SERIAL_PRIORITY   = 32;
    constexpr uint8_t  OLED_PRIORITY     = 40;
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
// 任务管理器 — 封装所有任务状态和逻辑
// ====================================================================
namespace {

class TaskManager {
public:
    // ---- 公开 API（由 extern "C" 包装调用） ----
    static void Init() {
        oled_update_tick_    = HAL_GetTick();
        serial_process_tick_ = HAL_GetTick();
        button_scan_tick_    = HAL_GetTick();
        debug_output_tick_   = HAL_GetTick();
    }

    static void CreateTasks() {
        struct TaskDef {
            const char*    name;
            osThreadFunc_t func;
            uint32_t       stack;
            uint8_t        priority;
        };

        constexpr TaskDef tasks[] = {
            { "SerialTask",  &SerialTaskFunc,  config::SERIAL_STACK_SIZE, config::SERIAL_PRIORITY },
            { "OLED_Task",   &OLEDTaskFunc,    config::OLED_STACK_SIZE,   config::OLED_PRIORITY },
            { "FOC_Task",    &FOCTaskFunc,     config::FOC_STACK_SIZE,    config::FOC_PRIORITY },
            { "MT6701_Test", &MT6701TaskFunc,  config::MT6701_STACK_SIZE, config::MT6701_PRIORITY },
        };

        for (const auto& t : tasks) {
            osThreadAttr_t attr{};
            attr.name       = t.name;
            attr.stack_size = t.stack;
            attr.priority   = osPriority_t(t.priority);
            osThreadNew(t.func, nullptr, &attr);
        }
    }

    static void FOCErrorHandleTask() {
        FOC_error_handle();
    }

    static void OLEDUpdate() {
        OLED_ShowNum(2, 12, FOC_GET_state(), 3);
        OLED_ShowNum(2, 1, FOC_GET_elect_angle(), 3);
        OLED_ShowString(2, 5, "deg");
    }

    static void SerialProcess() {
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

private:
    // ---- 静态状态（取代全局 static 变量） ----
    static uint32_t oled_update_tick_;
    static uint32_t serial_process_tick_;
    static uint32_t button_scan_tick_;
    static uint32_t debug_output_tick_;

    // ---- FreeRTOS 任务函数 ----
    static void SerialTaskFunc(void* /*argument*/) {
        /* Serial_Init() 在 main.c 中已调用, 这里不再重复 */
        for (;;) {
            osSemaphoreAcquire(uart_sem, osWaitForever);
            SerialProcess();
        }
    }

    static void OLEDTaskFunc(void* /*argument*/) {
        OLED_Init();
        for (;;) {
            OLED_ShowNum(1, 1, FOC_count, 5);
            OLEDUpdate();
            osDelay(100);
        }
    }

    static void FOCTaskFunc(void* /*argument*/) {
        FOC_Init();
        for (;;) {
            FOCErrorHandleTask();
            // Serial_Printf("[plot,%d,%d,%d]\r\n", FOC_Ua, FOC_Ub, FOC_Uc);

            osDelay(50);
        }
    }

    static void MT6701TaskFunc(void* /*argument*/) {
        osDelay(10);
        MT6701_Create(&hspi2, GPIOB, GPIO_PIN_11);
        MT6701_Refresh();
        osDelay(5);

        for (;;) {
            MT6701_Refresh();
            int16_t raw   = MT6701_GetAngle();
            float   deg   = MT6701_GetDeg();
            MT6701_GetTurns();

            OLED_ShowNum(3, 1, raw, 5);
            // OLED_ShowNum(3, 7, static_cast<uint16_t>(deg), 3);
            // OLED_ShowSignedNum(4, 11,
            //     static_cast<int16_t>(static_cast<int>(deg * 10.0f) % 10), 2);


            // Serial_Printf("[plot,%d]",raw);
            osDelay(40);
        }
    }
};

// ---- 静态成员定义 ----
uint32_t TaskManager::oled_update_tick_    = 0;
uint32_t TaskManager::serial_process_tick_ = 0;
uint32_t TaskManager::button_scan_tick_    = 0;
uint32_t TaskManager::debug_output_tick_   = 0;

} // anonymous namespace

// ====================================================================
// extern "C" 接口 — 与 C 头文件声明匹配，main.c 无需任何修改
// ====================================================================
void Task_Manager_Init(void)  { TaskManager::Init(); }
void Task_Manager_Run(void)   { /* reserved */ }
void Task_OLED_Update(void)   { TaskManager::OLEDUpdate(); }
void Task_Serial_Process(void){ TaskManager::SerialProcess(); }
void App_Task(void* argument) { /* deprecated — tasks created in App_Task_Create */ }
void App_Task_Create(void)    { TaskManager::CreateTasks(); }