//
// Created by zhang on 2026/6/1.
// ———— C++ 版本 ————
//

#include "app_task.h"      // 已有 extern "C" 守卫
#include "cmsis_os.h"      // CMSIS 头文件自带 extern "C"
#include "FOC_3.h"

#include <cstring>
#include <cstdlib>

/* 共用的键值对解析: 将 "key:value\0" 原地断开 */
static void parse_kv(char *cmd, char **key, char **val_str) {
    *key = cmd;
    *val_str = NULL;
    char *p = strchr(cmd, ':');
    if (p) {
        *p = '\0';
        *val_str = p + 1;
    }
}

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

int16_t test_count121 = 0;


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
static void KEYTaskFunc(void* argument);


// ====================================================================
// 任务函数实现
// ====================================================================

int16_t temp[1];
int16_t world_dir = 0;
int16_t world_dir_target = 0;
bool en_trace = false;

void start_trace(int8_t EN) {
    en_trace = EN;
    world_dir_target = world_dir;
}
void control_trace_init() {
    world_dir_target = world_dir;
}
int16_t world_location_target;
void control_trace() {
    if (en_trace){
        FOC_3*  m = FOC_3::instances[1];
        JY61P_GetAngleRaw(&temp[0],&temp[1],&world_dir);
        int16_t diff_location = (world_dir_target - world_dir)/4;
        world_location_target = diff_location + m->get_location();
        if (world_location_target>8191){world_location_target -= 16384;}
        else if (world_location_target<-8191){world_location_target += 16384;}
        m->set_location_target(world_location_target);
    }
}

uint8_t TI_state =0;
float ball_location = 0;
float ball_location_target = 0;
bool ball_balance_en = false;

control_object ball_location_O;
control_PID ball_balance_PID;
void ball_balance_init() {
    Control_PID_Init(&ball_balance_PID,   150.0f, 0.35f, 16000.0f, -2000.0f, 2000.0f, 200.0f,200.0f);

}
void testdd() {
    ball_balance_PID.kp = 0.1f;
}

#define alpha_ball_lac 0.1
float out_ball_loc  = 0;
void ball_balance() {
    if (ball_balance_en) {
        if (TI_state == 1) {
            ball_location_target = 5;
            if (ball_location- 5 <1 && ball_location - 5 > -1) {
                TI_state =2;
            }
        }else if (TI_state ==2) {
            ball_location_target = -5;
        }
        ball_location = -(float)vision_center_x/100;
        ball_location_O.Actual = ball_location;
        ball_location_O.Target = ball_location_target;
        out_ball_loc = PID_Calculate_DerivativeOnMeasure(&ball_balance_PID,&ball_location_O)*alpha_ball_lac + out_ball_loc*(1-alpha_ball_lac);
        FOC_3::instances[0]->set_location_target((int16_t)out_ball_loc);
    }
}


/* ======================== 正弦信号控制器命令解析 ======================== */
/* 串口格式 (VOFA+ 键值对):  key:value
 *   sin_start    — 启动输出
 *   sin_stop     — 停止输出
 *   sin_amp:2.0  — 设置振幅
 *   sin_period:5.0 — 设置周期 (秒)
 *   sin_offset:1.0 — 设置直流偏置
 */
static SineGen   g_sin    = {0.5f, 5.0f, 0.0f, 0.0f};
static bool      g_sin_en    = false;
static bool      g_square_en = false;

void Sin_Parse(const char *key, const char *val_str) {
    float val = val_str ? atof(val_str) : 0.0f;

    if (strcmp(key, "sin_start") == 0) {
        g_sin_en = true;
        g_square_en = false;
    }
    else if (strcmp(key, "sin_stop") == 0) {
        g_sin_en = false;
        g_square_en = false;
    }else if (strcmp(key, "BALL") == 0) {
        ball_balance_en = (bool)val;
    }
    else if (strcmp(key, "sin_amp") == 0) {
        g_sin.amplitude = val;
    }
    else if (strcmp(key, "sin_period") == 0) {
        g_sin.period = (val > 0.0001f) ? val : 0.001f;   /* 防设成 0 */
    }
    else if (strcmp(key, "sin_offset") == 0) {
        g_sin.offset = val;
    }
    /* 方波 — 共用 g_sin 的幅度/周期/偏置/相位，占空比固定 0.5 */
    else if (strcmp(key, "square_start") == 0) {
        g_square_en = true;
        g_sin_en = false;
    }
    else if (strcmp(key, "square_stop") == 0) {
        g_square_en = false;
    }
    else if (strcmp(key, "en_trace") == 0) {
        start_trace((int8_t)val);
    }
    else if (strcmp(key, "ball_loc") == 0) {
        ball_location_target = val;
    }
    else if (strcmp(key, "ball_state") == 0) {
        TI_state = (uint8_t)val;
    }
}

float test_sin;
void Sin_control() {
    const float dt = 0.005f;
    float temp;

    if (g_sin_en) {
        temp = SineGen_Calc(&g_sin);
    } else if (g_square_en) {
        /* 直接基于 g_sin 的参数内联计算方波，相位共用 */
        if (g_sin.period <= 0.0001f) return;
        float norm = g_sin.phase / (2.0f * 3.14159265f);
        temp = (norm < 0.5f) ? (g_sin.amplitude + g_sin.offset)
                             : (-g_sin.amplitude + g_sin.offset);
        g_sin.phase += (2.0f * 3.14159265f / g_sin.period) * dt;
        if (g_sin.phase >= 2.0f * 3.14159265f) g_sin.phase -= 2.0f * 3.14159265f;
    } else {
        return;
    }

    test_sin = temp;
    FOC_3::instances[0]->set_speed_target(test_sin);
}


/* ====================================================================
 * 按键扫描框架 (PE1=KEY1, PE3=KEY3)
 *   - 轮询 + 软件消抖: 连续 KEY_DEBOUNCE_N 次反相才确认电平翻转
 *   - 边沿检测: 只在"按下"瞬间置一次事件标志, 按住不重复触发
 *   - 消费方式: 在主循环检查 KEYx_pressed, 用后手动清 0
 *   - 电平: 板载按键按下接 GND → 低电平有效
 * ==================================================================== */
#define KEY_DEBOUNCE_N   3       /* 连续采样次数 (×20ms ≈ 60ms 消抖) */

volatile uint8_t KEY1_pressed = 0;   /* 按下事件, 消费后清 0 */
volatile uint8_t KEY3_pressed = 0;

typedef struct {
    GPIO_TypeDef  *port;
    uint16_t       pin;
    uint8_t        state;           /* 当前稳定电平 (1=释放 0=按下) */
    uint8_t        cnt;             /* 连续反相计数 */
    volatile uint8_t *pressed;      /* 指向对应事件标志 */
} key_desc_t;

static key_desc_t keys[] = {
    { KEY1_GPIO_Port, KEY1_Pin, 1, 0, &KEY1_pressed },
    { KEY3_GPIO_Port, KEY3_Pin, 1, 0, &KEY3_pressed },
};

void Key_Scan()
{
    for (auto &k : keys) {
        uint8_t level = HAL_GPIO_ReadPin(k.port, k.pin);
        if (level == k.state) {
            k.cnt = 0;                       /* 电平未变, 计数归零 */
        } else if (++k.cnt >= KEY_DEBOUNCE_N) {
            k.state = level;                 /* 连续 N 次反相, 确认翻转 */
            if (k.state == 0) {
                *k.pressed = 1;              /* 下降沿 = 按下, 只报一次 */
            }
            k.cnt = 0;
        }
    }
}

static void KEYTaskFunc(void* /*argument*/)
{
    for (;;) {
        Key_Scan();
        osDelay(config::BUTTON_SCAN_MS);     /* 20ms 轮询 */
    }
}




static void OLEDTaskFunc(void* /*argument*/) {
    OLED_Init();
    for (;;) {

        /* 按键 → 串口: 按下时发送按键标记 (消费后清 0) */
        if (KEY1_pressed) {
            KEY1_pressed = 0;
            Serial_SendVar("KEY1", 1);
        }
        if (KEY3_pressed) {
            KEY3_pressed = 0;
            Serial_SendVar("KEY3", 1);
        }

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
            OLED_ShowNum(4, 7,m->get_elect_angle_zero(), 5);
            OLED_ShowNum(4, 1,test_count121, 3);
            // OLED_ShowNum(4, 13,FOC_3::pid_debug, 3);
        } else {
            OLED_ShowString(1, 8, "no motor");
            static uint8_t count = 0;
            count++;
            OLED_ShowNum(2,8, count,3);
        }
        osDelay(50);
    }
}

static void FOC_3_task(void* /*argument*/) {
    ball_balance_init();
    Serial_Init();
    control_tim_start();

    //电机1
    static control_PID m1_speed_pid, m1_loc_pid, m1_curq_pid, m1_curd_pid, m1_direct_pid;
    /* 初始化三环 PID 参数 */
    Control_PID_Init(&m1_speed_pid,   3000.0f, 200.0f, 0.0f, -32767.0f, 32767.0f, 20000.0f,20000.0f);
    m1_speed_pid.Er_weaken = 0.98;
    // Control_PID_Init(&m1_loc_pid,     0.0012f, 0.0f, 0.0002f, -4.0f, 4.0f,1.0f, 15.0f);                     //精确稳健
    Control_PID_Init(&m1_loc_pid,     0.0075f, 0.0f, 0.001f, -4.0f, 4.0f,4.0f, 15.0f);     //响应速度

    Control_PID_Init(&m1_curq_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 250,20000);
    // lowpass_Filter curOutFilter = {0.1f,0.0f};
    // PID_EnableOutputFilter(&m1_curq_pid, lowpass_filter,&curOutFilter);

    Control_PID_Init(&m1_curd_pid,    10.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 5000,5000);
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
    motor1.set_location_limit(2000,-2000);

    Serial_SendVar("zero",motor1.get_elect_angle_zero());
    // motor1.set_cogging_enable(false);

    // //电机2
    // /* 创建 PID */
    // static control_PID m2_speed_pid, m2_loc_pid, m2_curq_pid, m2_curd_pid, m2_direct_pid;
    // /* 初始化三环 PID 参数 */
    // Control_PID_Init(&m2_speed_pid,   20000.0f, 230.0f, 0.0f, -30000.0f, 30000.0f, 1000.0f,2000.0f);
    // // Control_PID_Init(&m2_loc_pid,     0.003f, 0.0f, 0.002f, -4.0f, 4.0f,1.0f, 15.0f);                     //齿槽补偿
    // Control_PID_Init(&m2_loc_pid,     0.005f, 0.0f, 0.00f, -4.0f, 4.0f,1.0f, 15.0f);
    // Control_PID_Init(&m2_curq_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 250,20000);
    // // lowpass_Filter curOutFilter = {0.1f,0.0f};
    // // PID_EnableOutputFilter(&m2_curq_pid, lowpass_filter,&curOutFilter);
    //
    // Control_PID_Init(&m2_curd_pid,    1.0f, 0.04f, 0.0f, -32767.0f, 32767.0f, 5000,5000);
    // Control_PID_Init(&m2_direct_pid,  15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f,5000);
    // /* 创建电机 */
    // FOC_3::PIDCtrl m2_pid;
    // m2_pid.speed     = &m2_speed_pid;
    // m2_pid.location  = &m2_loc_pid;
    // m2_pid.direct    = &m2_direct_pid;
    // m2_pid.current_q = &m2_curq_pid;
    // m2_pid.current_d = &m2_curd_pid;
    //
    // static MT6701 m2_enc({&hspi2, GPIOB, GPIO_PIN_12});
    // static FOC_3 motor2({&m2_enc,I_check_lowpass_M2, &htim2, &htim3, 11, 12.0f, 7.5f, 1.7f, 1, "motor1"});
    // motor2.init(m2_pid);

    // motor2.set_location_zero(-4500);
    // motor2.set_location_limit(4000,-4000);
    control_trace_init();
    for (;;) {
        osSemaphoreAcquire(uart_sem, osWaitForever);
        /* 关中断复制 kv_buf，防止 ISR 在 parse_kv 期间覆写 */
        char local_kv[64];
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        strcpy(local_kv, kv_buf);
        __set_PRIMASK(primask);

        char *key, *val;
        parse_kv(local_kv, &key, &val);
        FOC_3_parse_keyValue(key, val);
        Sin_Parse(key, val);

         int32_t dt =get_tim_dif(1);

        // Serial_Printf("4: %d\n",get_tim_dif(4));
        // Serial_Printf("3: %d\n",get_tim_dif(3));
        // Serial_Printf("2: %d\n",get_tim_dif(2));
        // Serial_Printf("5: %d\n",get_tim_dif(5));
        // Serial_Printf("1: %d\n",dt);
        // Serial_Printf("0: %d\n",get_tim_dif(0));

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

        osDelay(1);
    }
}

int16_t gx, gy, gz, roll, pitch, yaw;
float out[6] ;
static void JY61PTestTask(void* /*argument*/) {
    /* USART6 接收已在 Serial_Init() 中启动 */
    osDelay(500);  // 等模块上电稳定

    uint32_t last_log = 0;

    for (;;) {
        /* 读陀螺仪数据 (°/s) */
        JY61P_GetGyroRaw(&gx, &gy, &gz);
        JY61P_GetAngleRaw(&roll, &pitch, &yaw);
        out[0] = (float)gx;
        out[1] = (float)gy;
        out[2] = (float)gz;
        out[3] = (float)roll;
        out[4] = (float)pitch;
        out[5] = (float)yaw;


        /* VOFA+ justfloat 协议: gx gy gz roll pitch yaw */
        // Serial_JustFloat(out, 3);

        // OLED_ShowSignedNum(4,1,pitch,6);
        osDelay(10);
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
        else if (FOC_3::pid_debug == 6) {
            temp[0] = ball_location_O.Target;
            temp[1] = ball_location_O.Actual;
            temp[2] = ball_location_O.Out;
        }
        temp[4] = ball_location;
        // temp[3] = world_location_target;
        // temp[4] = FOC_3::instances[0]->get_iq_target();
        temp[3] = test_sin;


        Serial_JustFloat(temp,5);
        // temp[0] = (float)out[1];
        // Serial_JustFloat(out,6);
        osDelay(5);
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
        { "OLED_Task",   &OLEDTaskFunc,    config::OLED_STACK_SIZE,   30 },
        { "KEY_Task",    &KEYTaskFunc,     128 * 4,                  35 },
        // { "FOC_Task",    &FOCTaskFunc,     config::FOC_STACK_SIZE,    config::FOC_PRIORITY },
        { "task_printf", &task_printf,  256 * 4 , 45 },
        { "JY61P_Test",  &JY61PTestTask,   256 * 4,                  40 },
        { "FOC_3_task",    &FOC_3_task,     512*4,    45 },
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

