//
// Created by zhang on 2026/6/1.
// ———— C++ 版本 ————
//

#include "app_task.h"      // 已有 extern "C" 守卫
#include "cmsis_os.h"      // CMSIS 头文件自带 extern "C"
#include "FOC_3.h"

// C 库头文件 — 没有 extern "C" 守卫，需要手动包裹
extern "C" {
#include "OLED.h"
#include "Serial.h"
#include "MT6701.h"
#include "JY61P.h"
#include "check_I.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "Control.h"
}


//test
extern uint16_t tim5_count;
extern uint16_t adc_dma_count;


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
            OLED_ShowSignedNum(2, 11, m->get_location(), 5);

            /* 行3: IqR:800 Iq:780 */
            OLED_ShowNum(3, 1, get_tim_dif(0), 4);
            OLED_ShowNum(3, 9, m->get_encode_val(), 5);

            /* 行4: Ia:+123 Ib:-045 */
            OLED_ShowNum(4, 1,get_tim_dif(1), 5);
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



static void FOC_3_test_task(void* /*argument*/) {

    Serial_Init();
    control_tim_start();

    //电机1
    static control_PID m1_speed_pid, m1_loc_pid, m1_curq_pid, m1_curd_pid, m1_direct_pid;
    /* 初始化三环 PID 参数 */
    Control_PID_Init(&m1_speed_pid,   3000.0f, 200.0f, 0.0f, -32767.0f, 32767.0f, 20000.0f,20000.0f);
    Control_PID_Init(&m1_loc_pid,     0.008f, 0.0f, 0.01f, -4.0f, 4.0f,1.0f, 15.0f);
    Control_PID_Init(&m1_curq_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 250,20000);
    // lowpass_Filter curOutFilter = {0.1f,0.0f};
    // PID_EnableOutputFilter(&m1_curq_pid, lowpass_filter,&curOutFilter);

    Control_PID_Init(&m1_curd_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 5000,5000);
    Control_PID_Init(&m1_direct_pid,  15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f,5000);
    /* 创建电机 */
    FOC_3::PIDCtrl m1_pid;
    m1_pid.speed     = &m1_speed_pid;
    m1_pid.location  = &m1_loc_pid;
    m1_pid.direct    = &m1_direct_pid;
    m1_pid.current_q = &m1_curq_pid;
    m1_pid.current_d = &m1_curd_pid;
    static MT6701 m1_enc({&hspi1, GPIOD, GPIO_PIN_7});
    static FOC_3 motor1({&m1_enc,I_check_lowpass_M1, &htim1, &htim3, 11, 12.0f, 7.5f, 1.7f, 0, "motor1"});
    motor1.init(m1_pid);
    motor1.set_location_zero(5700);
    // motor1.set_location_limit(2000,-4000);

    //电机2
    /* 创建 PID */
    static control_PID m2_speed_pid, m2_loc_pid, m2_curq_pid, m2_curd_pid, m2_direct_pid;
    /* 初始化三环 PID 参数 */
    Control_PID_Init(&m2_speed_pid,   3000.0f, 200.0f, 0.0f, -32767.0f, 32767.0f, 20000.0f,2000.0f);
    Control_PID_Init(&m2_loc_pid,     0.0005f, 0.0f, 0.01f, -4.0f, 8.0f,1.0f, 15.0f);
    Control_PID_Init(&m2_curq_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 250,20000);
    // lowpass_Filter curOutFilter = {0.1f,0.0f};
    // PID_EnableOutputFilter(&m2_curq_pid, lowpass_filter,&curOutFilter);

    Control_PID_Init(&m2_curd_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 5000,5000);
    Control_PID_Init(&m2_direct_pid,  15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f,5000);
    /* 创建电机 */
    FOC_3::PIDCtrl m2_pid;
    m2_pid.speed     = &m2_speed_pid;
    m2_pid.location  = &m2_loc_pid;
    m2_pid.direct    = &m2_direct_pid;
    m2_pid.current_q = &m2_curq_pid;
    m2_pid.current_d = &m2_curd_pid;

    static MT6701 m2_enc({&hspi2, GPIOB, GPIO_PIN_12});
    static FOC_3 motor2({&m2_enc,I_check_lowpass_M2, &htim2, &htim3, 11, 12.0f, 7.5f, 1.7f, 1, "motor1"});
    motor2.init(m2_pid);
    motor2.set_location_limit(4000,-4000);

    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);
        FOC_3_parse_keyValue(kv_buf);
        // motor1.error_handle();
        // motor2.error_handle();
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
    /* 手动初始化 PB12 为推挽输出 */
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_12;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    for (;;) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        osDelay(500);
    }
}

extern int16_t test_u_a;
extern int16_t test_u_b;
extern int16_t test_u_c;
extern int16_t test_i_d;
extern int16_t test_i_q;
extern int16_t test_i_a,test_i_b;
extern float test_curdata[3];

static void task_printf(void * /*argument*/) {
    osDelay(10);
    for (;;) {
        float datas[1];
        // Serial_JustFloat(datas,1);
        int16_t ia = I_Check_Get_M1_A();
        int16_t ib = I_Check_Get_M1_B();
        int16_t ia2 = I_Check_Get_M2_A();
        int16_t ib2 = I_Check_Get_M2_B();

        float temp[6];
        // temp[0] = FOC_3::instances[0]->get_elect_angle_A() *0.001;
        // temp[1] = FOC_3::instances[0]->get_elect_angle() *0.001;
        // temp[0] = test_i_q;
        // temp[1] = test_i_d;
        // temp[2] = FOC_3::instances[0]->get_iq_target() ;
        // temp[3] = FOC_3::instances[0]->get_uq()*1.0 ;
        // temp[2] = ia;
        // temp[3] = ib;
        // temp[2] = test_u_a;
        // temp[3] = test_u_b;
        if (FOC_3::pid_debug == 0) {
        FOC_3::showPID_cur_q(temp);
        }else if (FOC_3::pid_debug == 1) {
        FOC_3::showPID_cur_d(temp);
        }
        else if (FOC_3::pid_debug == 2) {
        FOC_3::showPID_speed(temp);
        }else if (FOC_3::pid_debug == 3) {
        FOC_3::showPID_location(temp);
        }
        temp[4] = test_curdata[1];
        // temp[4] = FOC_3::instances[0]->get_iq_target();
        // temp[5] = test_curdata[2];


        Serial_JustFloat(temp,5);
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
        { "task_printf", &task_printf,  256 * 4 , 42 },
        // { "JY61P_Test",  &JY61PTestTask,   256 * 4,                  30 },
        { "FOC_3_test_task",    &FOC_3_test_task,     512*4,    40 },
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

