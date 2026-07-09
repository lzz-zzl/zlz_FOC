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
#include "check_I.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
}


//test
extern uint16_t tim5_count;
extern uint16_t adc_dma_count;

#include <cstdint>
#include "FOC_3.h"
#include "Control.h"

uint8_t *test_data = NULL;
void set_testdata(uint8_t * p) {
    test_data = p;
}

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
static void FOC_3_test_task(void* argument);

// ====================================================================
// 任务函数实现
// ====================================================================

static void OLEDTaskFunc(void* /*argument*/) {
    OLED_Init();
    for (;;) {

        FOC_3 *m = FOC_3::instances[FOC_3::motorID_debug];
        if (m) {
            static uint8_t count = 0;
            count++;
            OLED_ShowNum(1,1,count,3);

            OLED_ShowNum(1, 5, m->m_per_count/15000, 3);          /* 累计帧计数 — 保留 */
            /* 行1: M0 st:3 spd:1500 */
            OLED_ShowString(1, 9, "M");
            OLED_ShowNum(1, 10, m->get_id(), 1);
            OLED_ShowString(1, 12, "S");
            OLED_ShowNum(1, 14, m->get_state(), 2);

            /* 行2: ang:123 loc:4000 */
            OLED_ShowString(2, 1, "AG");
            OLED_ShowNum(2, 4, m->get_elect_angle(), 3);
            OLED_ShowString(2, 8, "LO");
            OLED_ShowNum(2, 11, m->get_location(), 5);

            /* 行3: IqR:800 Iq:780 */
            OLED_ShowSignedNum(3, 1, m->get_iq_q15() / 16, 4);

            /* 行4: Ia:+123 Ib:-045 */
            OLED_ShowNum(4, 1, m->get_elect_angle_zero(), 5);
            OLED_ShowNum(4, 8,adc_dma_count, 3);
            OLED_ShowNum(4, 13,FOC_3::pid_debug, 3);
        } else {
            OLED_ShowString(1, 8, "no motor");
            static uint8_t count = 0;
            count++;
            OLED_ShowNum(2,8, count,3);
        }
        osDelay(50);
    }
}

static void SerialTaskFunc(void* /*argument*/) {
    Serial_Init();
    // osDelay(200);  // 等外设稳定
    // Serial_Printf("[boot]USART1 ready at 115200\r\n");
    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);

        // Serial_GET_order();


        static uint16_t serial_count = 0;
        serial_count++;
        OLED_ShowNum(4, 15, serial_count, 2);

        FOC_SET_state(Serial_value[1]);
        FOC_Set_Uq(Serial_slider[0] * 100);

        if ((FOC_GET_state() == 4 || FOC_GET_state() == 5) && Serial_key[1] != 1) {
            FOC_SET_rela_location(static_cast<int16_t>(Serial_slider[2]));
        }

        FOC_SET_speed_target(Serial_slider[1]);
    }
}




static void FOC_3_test_task(void* /*argument*/) {

    Serial_Init();
    /* 创建 PID */
    static control_PID speed_pid, loc_pid, curq_pid, curd_pid, direct_pid;

    /* 初始化三环 PID 参数 */
    Control_PID_Init(&speed_pid,   1200.0f, 20.0f, 0.0f, -32767.0f, 32767.0f, 10000.0f);
    Control_PID_Init(&loc_pid,     0.02f, 0.000005f, 0.001f, -4.0f, 4.0f, 15.0f);
    Control_PID_Init(&curq_pid,    40.0f, 0.3f, 0.0f, -32767.0f, 32767.0f, 15000);
    Control_PID_Init(&curd_pid,    10.0f, 0.6f, 0.0f, -32767.0f, 32767.0f, 15000);
    Control_PID_Init(&direct_pid,  15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f);

    /* 创建电机 */
    FOC_3::PIDCtrl p;
    p.speed     = &speed_pid;
    p.location  = &loc_pid;
    p.direct    = &direct_pid;
    p.current_q = &curq_pid;
    p.current_d = &curd_pid;

    static MT6701 enc({&hspi1, GPIOD, GPIO_PIN_7});
    // static FOC_3 motor({&enc,I_check_lowpass_M2, &htim2, &htim5, 11, 12.0f, 7.5f, 1.7f, 0, "motor1"});
    static FOC_3 motor({&enc,I_check_lowpass_M1, &htim1, &htim5, 11, 12.0f, 7.5f, 1.7f, 0, "motor1"});
    motor.init(p);

    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);
        FOC_3_parse_keyValue(kv_buf);
        motor.error_handle();
        // osDelay(10);
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

        OLED_ShowNum(2, 7,MT6701_GetAngle(), 5);
        // OLED_ShowSignedNum(4, 11,
        //     static_cast<int16_t>(static_cast<int>(deg * 10.0f) % 10), 2);

        Serial_JustFloat(&deg,1);
        osDelay(10);
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

extern int16_t test_u_a;
extern int16_t test_u_b;
extern int16_t test_u_c;
extern int16_t test_i_d;
extern int16_t test_i_q;


static void task_printf(void * /*argument*/) {
    osDelay(10);
    for (;;) {
        float datas[1];
        datas[0] = FOC_speed;
        // Serial_JustFloat(datas,1);
        int16_t ia = I_Check_Get_M1_A();
        int16_t ib = I_Check_Get_M1_B();
        // int16_t ia2 = I_Check_Get_M2_A();
        // int16_t ib2 = I_Check_Get_M2_B();

        float temp[4];
        // temp[0] = FOC_3::instances[0]->get_elect_angle_A() *0.001;
        // temp[1] = FOC_3::instances[0]->get_elect_angle() *0.001;
        temp[0] = test_i_q;
        temp[1] = test_i_d;
        // temp[2] = FOC_3::instances[0]->get_iq_target() ;
        // temp[3] = FOC_3::instances[0]->get_uq()*1.0 ;
        // temp[0] = ia*0.001;
        // temp[1] = ib*0.001;
        temp[2] = test_u_a;
        temp[3] = test_u_b;


        Serial_JustFloat(temp,4);
        osDelay(1);
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
        { "OLED_Task",   &OLEDTaskFunc,    config::OLED_STACK_SIZE,   40 },
        // { "FOC_Task",    &FOCTaskFunc,     config::FOC_STACK_SIZE,    config::FOC_PRIORITY },
        { "task_printf", &task_printf,  256 * 4 , 40 },
        // { "JY61P_Test",  &JY61PTestTask,   256 * 4,                  30 },
        { "FOC_3_test_task",    &FOC_3_test_task,     512*4,    45 },
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

