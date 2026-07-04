//
// Created by zhang on 2026/6/3.
//

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "spi.h"
#include "FOC_2.h"
#include "DRV8313.h"
#include "Control.h"
#include <math.h>

/* check_I.h 没有 extern "C" 守卫, C++ 需手动包裹 */
extern "C" {
#include "check_I.h"
}


/* ========================================================================== */
/*                      常量 & 宏                                              */
/* ========================================================================== */

#define SQRT_3             0.8660254f
#define VOLTAGE_Q15_MAX    32767
#define VOLTAGE_Q15_MIN    (-32768)
#define Q16                65536
#define Q16_D_115          56987   // Q16/1.15
#define Q15_SCALE          32767
#define TABLE_SIZE         360
#define PWM_PERIOD         699

#define ENCODER_BIT        16383

#define DRIVE_INIT         DRV8313_Init()
#define DRIVE_ENABLE        DRV8313_CHOOSE_ENABLE(1)
#define DRIVE_DISABLE       DRV8313_CHOOSE_ENABLE(0)

#define CONSTRAIN(val, lo, hi) \
    ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))

#define ERR_LOCATION       10
#define LOCATION_TIPPING   200
#define INTERRUPT_PERIOD   3000  // us
#define PLL                (INTERRUPT_PERIOD / 1000)

/* ========================================================================== */
/*                      全局默认实例 (extern "C" 兼容层)                        */
/* ========================================================================== */

static FOC_2 *g_defaultFOC = nullptr;

void   FOC_2_set_default_instance(FOC_2 *inst) { g_defaultFOC = inst; }
FOC_2 *FOC_2_get_default_instance()             { return g_defaultFOC; }

/* 兼容 FOC_1 的全局变量 — 全部指向默认实例 */
int16_t  FOC_Uq, FOC_Ud;
uint16_t FOC_Ua, FOC_Ub, FOC_Uc;
uint8_t  FOC_state;
int16_t  FOC_location;
int16_t  FOC_location_target;
int8_t   FOC_location_state;
float    FOC_speed;
uint16_t elect_angle_zero;
int16_t  FOC_rela_location;

/* FOC_1 兼容全局变量 — 供 app_task.c 等 C 文件使用 */
int16_t Uq = 0;
int16_t Ud = 0;

static inline void sync_globals() {
    if (!g_defaultFOC) return;
    FOC_Uq   = g_defaultFOC->get_uq();
    FOC_Ud   = g_defaultFOC->get_ud();
    FOC_Ua   = g_defaultFOC->get_ua();
    FOC_Ub   = g_defaultFOC->get_ub();
    FOC_Uc   = g_defaultFOC->get_uc();
    FOC_state = g_defaultFOC->get_state();
    FOC_location = g_defaultFOC->get_location();
    FOC_location_target = g_defaultFOC->get_location_target();
    FOC_speed = g_defaultFOC->get_speed();
    elect_angle_zero = (uint16_t)g_defaultFOC->get_elect_angle_zero();
    FOC_location_state = 0;
    Uq = FOC_Uq;                     // 同步 FOC_1 兼容变量
    Ud = FOC_Ud;
    uint16_t debug_elect_angle = g_defaultFOC->get_elect_angle();
}

/* ---------- extern "C" 函数实现 ---------- */

extern "C" {

void FOC_Init(void) {
    if (!g_defaultFOC) FOC_2_create_default();
    if (g_defaultFOC) g_defaultFOC->init();
}

void FOC_period(void) {
    if (g_defaultFOC) {
        g_defaultFOC->period();
        sync_globals();
    }
}

void FOC_Set_Uq(int16_t v)       { if (g_defaultFOC) g_defaultFOC->set_uq(v); }
void FOC_Set_Ud(int16_t v)       { if (g_defaultFOC) g_defaultFOC->set_ud(v); }
void FOC_SET_state(uint8_t s)    { if (g_defaultFOC) g_defaultFOC->set_state(s); }
uint8_t FOC_GET_state(void)      { return g_defaultFOC ? g_defaultFOC->get_state() : 0; }
uint16_t FOC_GET_elect_angle(void){ return g_defaultFOC ? g_defaultFOC->get_elect_angle() : 0; }
int16_t FOC_GET_elect_angle_zero(void) { return g_defaultFOC ? g_defaultFOC->get_elect_angle_zero() : 0; }
void FOC_SET_speed_target(float v){ if (g_defaultFOC) g_defaultFOC->set_speed_target(v); }
float FOC_GET_speed_target(void) { return g_defaultFOC ? g_defaultFOC->get_speed_target() : 0.0f; }
void FOC_SET_rela_location(int16_t v) { if (g_defaultFOC) g_defaultFOC->set_rela_location(v); }
void FOC_SET_location_target(int16_t v) { if (g_defaultFOC) g_defaultFOC->set_location_target(v); }
int16_t FOC_GET_location_target(void) { return g_defaultFOC ? g_defaultFOC->get_location_target() : 0; }
void FOC_SET_location_limit(int16_t lo, int16_t hi) {
    if (g_defaultFOC) g_defaultFOC->set_location_limit(lo, hi);
}
void FOC_error_handle(void)      { if (g_defaultFOC) g_defaultFOC->error_handle(); }
void FOC_elect_angle_MT6701(void){ /* 内部已处理，保留空壳兼容 */ }
int16_t foc_test(int16_t a)      { return a; }

void FOC_2_create_default(void) {
    static MT6701 s_encoder({&hspi1, GPIOD, GPIO_PIN_7});

    /* 注：s_foc 的构造函数会调用 FOC_2_set_default_instance(this) */
    static FOC_2 s_foc({
        &s_encoder,
        &htim1,
        &htim5,       /* 用 TIM5 (15kHz) 做控制周期，TIM3 留给编码器 */
        11,        // pole_pairs
        12.0f,    // u_bus
        7.5f,     // r_wire
        1.7f      // i_max
    });

    (void)s_foc;  // 防止未使用变量警告
}

} /* extern "C" */

/* ========================================================================== */
/*                      构造函数                                                */
/* ========================================================================== */

FOC_2::FOC_2(const Config &cfg)
    : m_encoder(cfg.encoder),
      m_htim_pwm(cfg.htim_pwm),
      m_htim_ctrl(cfg.htim_ctrl),
      m_pole_pairs(cfg.pole_pairs),
      m_u_bus(cfg.u_bus),
      m_r_wire(cfg.r_wire),
      m_i_max(cfg.i_max),
      m_uq(0), m_ud(0),
      m_u_alpha(0), m_u_beta(0),
      m_ua(0), m_ub(0), m_uc(0),
      m_wire_swapped(false),
      m_elect_angle(0),
      m_state(0),
      m_location(0),
      m_location_target(4000),
      m_location_target_state(0),
      m_max_location(16383),
      m_min_location(0),
      m_rela_location_zero(11595),
      m_elect_angle_zero(11101),
      m_speed(0.0f),
      m_speed_target(0.0f),
      m_current_q_pid{}, m_current_d_pid{},
      m_current_q_obj{}, m_current_d_obj{},
      m_i_a_fb(0.0f), m_i_b_fb(0.0f), m_i_c_fb(0.0f),
      m_i_alpha_fb(0.0f), m_i_beta_fb(0.0f),
      m_i_q_fb(0.0f), m_i_d_fb(0.0f),
      m_i_q_ref(0.0f), m_i_d_ref(0.0f),
      m_adc_zero_a(2048), m_adc_zero_b(2048),
      m_start_angle(0),
      m_per_count(0),
      m_check_wire_count(0),
      m_elect_angle_analog(0.0f)
{
    /* sin/cos 表在 init() 中初始化 */
    for (int i = 0; i < 360; i++) {
        m_sin_table[i] = 0;
        m_cos_table[i] = 0;
    }

    FOC_2_set_default_instance(this);
}

/* ========================================================================== */
/*                      静态工具函数                                            */
/* ========================================================================== */

inline int16_t FOC_2::mul_q15(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b) >> 15);
}

inline uint16_t FOC_2::min_three(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    return m;
}

/* ========================================================================== */
/*                      sin/cos 查表初始化                                       */
/* ========================================================================== */

void FOC_2::table_init() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float rad = (float)i * 3.1415926f / 180.0f;
        m_cos_table[i] = (int16_t)(cosf(rad) * Q15_SCALE);
        m_sin_table[i] = (int16_t)(sinf(rad) * Q15_SCALE);
    }
}

/* ========================================================================== */
/*                      PID 参数初始化                                          */
/* ========================================================================== */

void FOC_2::pid_param_init() {
    Control_PID_Init(&m_speed_pid, 1200.0f, 20.0f, 0.0f, -32767.0f, 32767.0f, 10000.0f);
    Control_PID_Init(&m_location_pid, 0.02f, 0.000005f, 0.001f, -4.0f, 4.0f, 15.0f);

    m_derivative_filter.alpha = 0.3f;
    m_derivative_filter.last_value = 0.0f;

    Control_PID_Init(&m_direct_location0_pid, 15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f);
    Control_PID_Init(&m_direct_location1_pid, 45.0f, 0.4f, 100.0f, -32767.0f, 32767.0f, 8000.0f);
    PID_EnableActualFilter(&m_direct_location0_pid, lowpass_filter, &m_derivative_filter);

    /* 电流环 PI — ADC 单位→Q15: ±2048→±32767, 因此 Kp≈16 */
    Control_PID_Init(&m_current_q_pid, 8.0f, 0.1f, 0.0f, -32767.0f, 32767.0f, 10000.0f);
    Control_PID_Init(&m_current_d_pid, 8.0f, 0.1f, 0.0f, -32767.0f, 32767.0f, 10000.0f);
}

/* ========================================================================== */
/*                      电压输出 & 坐标变换                                      */
/* ========================================================================== */

void FOC_2::set_volt(int16_t dc_Ua, int16_t dc_Ub, int16_t dc_Uc) {
    /* 换线处理：B/C 相交换时互换通道 */
    if (m_wire_swapped) {
        int16_t tmp = dc_Ub;
        dc_Ub = dc_Uc;
        dc_Uc = tmp;
    }
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_1, (uint32_t)dc_Ua * PWM_PERIOD / Q16_D_115);
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_2, (uint32_t)dc_Ub * PWM_PERIOD / Q16_D_115);
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_3, (uint32_t)dc_Uc * PWM_PERIOD / Q16_D_115);
}

void FOC_2::inv_park_clark_q15(int16_t use_elect_angle) {
    int16_t cos_theta = m_cos_table[use_elect_angle];
    int16_t sin_theta = m_sin_table[use_elect_angle];

    /* 逆 Park */
    m_u_alpha = (mul_q15(-m_uq, cos_theta) - mul_q15(m_ud, sin_theta));
    m_u_beta  = (mul_q15(-m_uq, sin_theta) + mul_q15(m_ud, cos_theta));

    /* 逆 Clark */
    static const int16_t sqrt3_q15 = (int16_t)(SQRT_3 * Q15_SCALE);
    static const int16_t half_q15  = Q15_SCALE / 2;

    m_ua = m_u_alpha + VOLTAGE_Q15_MAX;
    m_ub = mul_q15((int16_t)(-m_u_alpha), half_q15) + mul_q15(m_u_beta, sqrt3_q15) + VOLTAGE_Q15_MAX;
    m_uc = mul_q15((int16_t)(-m_u_alpha), half_q15) - mul_q15(m_u_beta, sqrt3_q15) + VOLTAGE_Q15_MAX;
}

void FOC_2::svpwm_calculate() {
    uint16_t u_min = min_three(m_ua, m_ub, m_uc);
    m_ua -= u_min;
    m_ub -= u_min;
    m_uc -= u_min;
}

void FOC_2::move(int16_t angle) {
    inv_park_clark_q15(angle);
    svpwm_calculate();
    set_volt(m_ua, m_ub, m_uc);
}

void FOC_2::max_accelerate() {
    const int16_t acc = (int16_t)(m_r_wire * m_i_max * Q16 / m_u_bus);
    m_uq = acc;
}

/* ========================================================================== */
/*                      电流环 — Clarke / Park / PI                           */
/* ========================================================================== */

void FOC_2::current_clarke()
{
    /* 读 ADC -> 减去零点 */
    float a = (float)((int16_t)I_Check_Get_M1_A() - m_adc_zero_a);
    float b = (float)((int16_t)I_Check_Get_M1_B() - m_adc_zero_b);

    /* Clarke: 三相 -> αβ (保留 ADC 单位, PI 输出限幅已适应) */
    m_i_alpha_fb = a;
    m_i_beta_fb  = (a + 2.0f * b) * 0.57735027f;  /* ÷√3 */
}

void FOC_2::current_park()
{
    /* sin/cos 表是 Q15, 转浮点便于直接与电流值运算 */
    float cos_t = (float)m_cos_table[m_elect_angle] / Q15_SCALE;
    float sin_t = (float)m_sin_table[m_elect_angle] / Q15_SCALE;

    /* Park: αβ -> dq (纯浮点, F407 有 FPU 够快) */
    m_i_d_fb =  m_i_alpha_fb * cos_t + m_i_beta_fb * sin_t;
    m_i_q_fb = -m_i_alpha_fb * sin_t + m_i_beta_fb * cos_t;
}

void FOC_2::current_pi_loop()
{
    /* q 轴 PI: ref - feedback -> Vq */
    m_current_q_obj.Target = m_i_q_ref;
    m_current_q_obj.Actual = m_i_q_fb;
    m_uq = (int16_t)PID_Calculate(&m_current_q_pid, &m_current_q_obj);

    /* d 轴 PI: 目标始终为 0 (弱磁时才需 Id) */
    m_current_d_obj.Target = 0.0f;
    m_current_d_obj.Actual = m_i_d_fb;
    m_ud = (int16_t)PID_Calculate(&m_current_d_pid, &m_current_d_obj);
}

/* ========================================================================== */
/*                      电角度计算                                              */
/* ========================================================================== */

void FOC_2::elect_angle_from_encoder() {
    uint16_t temp = 0;
    if (m_location >= m_elect_angle_zero) {
        temp = m_location - m_elect_angle_zero;
    } else {
        temp = m_location + MT6701::RESOLUTION - m_elect_angle_zero;
    }
    uint32_t numerator = (uint32_t)temp * m_pole_pairs * 360;
    m_elect_angle = (int16_t)((numerator / (MT6701::RESOLUTION - 1)) % 360);
}

void FOC_2::elect_angle_analog(float rad_s) {
    m_elect_angle_analog += rad_s * (float)m_pole_pairs * 360.0f / 15000.0f;
    if (m_elect_angle_analog >= 360.0f) m_elect_angle_analog = 0.0f;
    if (m_elect_angle_analog < 0.0f)    m_elect_angle_analog = 360.0f;
    m_elect_angle = (int16_t)m_elect_angle_analog;
}

/* ========================================================================== */
/*                      零位校准 & 接线检查                                     */
/* ========================================================================== */

void FOC_2::elect_angle_zero_calibration() {
    /* 注意：这里需要 HAL 定时器操作，与 FOC_1.c 一致 */
    HAL_TIM_Base_Stop_IT(m_htim_ctrl);
    m_ud = 3000;
    m_uq = 0;
    DRIVE_ENABLE;
    move(0);
    osDelay(1000);
    m_elect_angle_zero = m_encoder->refresh_raw();
    m_elect_angle_zero = m_encoder->refresh_raw();
    DRIVE_DISABLE;
    m_uq = 0;
    m_ud = 0;
    set_volt(0, 0, 0);
    HAL_TIM_Base_Start_IT(m_htim_ctrl);
}

void FOC_2::check_wire() {
    m_state = 111;
    m_start_angle = m_encoder->refresh_low_pass();
    m_start_angle = m_encoder->refresh_low_pass();
}

/* ========================================================================== */
/*                      PID 控制环                                              */
/* ========================================================================== */

void FOC_2::speed_pid_loop() {
    m_speed_obj.Target = m_speed_target;
    m_speed_obj.Actual = m_speed;
    float output = PID_Calculate(&m_speed_pid, &m_speed_obj);
    m_ud = 0;
    m_i_q_ref = output;   /* 速度环输出 → 电流环 q 轴目标 */
}

void FOC_2::location_target_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    int16_t pos_err = m_location_target - m_location;
    if (pos_err > 8192)      pos_err -= 16384;
    else if (pos_err < -8192) pos_err += 16384;

    m_location_obj.Target = (float)(m_location + pos_err);
    m_location_obj.Actual = (float)m_location;

    float loc_speed = PID_Calculate(&m_location_pid, &m_location_obj);

    m_speed_obj.Target = loc_speed;
    m_speed_obj.Actual = m_speed;
    float output = PID_Calculate(&m_speed_pid, &m_speed_obj);
    m_i_q_ref = output;   /* 输出到电流环 */
}

void FOC_2::direct_location_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    m_location_obj.Target = (float)m_location_target;
    m_location_obj.Actual = (float)m_location;

    float output = 0;
    float diff = m_location_obj.Target - m_location_obj.Actual;
    if (diff < LOCATION_TIPPING && diff > -LOCATION_TIPPING) {
        output = PID_I_deadband_Calculate(&m_direct_location0_pid, &m_location_obj);
    } else {
        output = PID_I_deadband_Calculate(&m_direct_location1_pid, &m_location_obj);
    }
    m_i_q_ref = output;   /* 输出到电流环 */
}

/* ========================================================================== */
/*                      主周期函数 (15kHz ISR 调用)                             */
/* ========================================================================== */

void FOC_2::period() {
    /* 状态检测 */
    if (m_state != 0) {
        DRIVE_ENABLE;
    } else {
        DRIVE_DISABLE;
        m_location_target_state = 0;
        set_volt(0, 0, 0);
    }

    m_per_count++;

    /* 角度刷新 — 使用编码器类 */
    m_location = m_encoder->refresh_raw();
    (void)m_encoder->get_angle_deg();
    elect_angle_from_encoder();

    /* 速度环 / 位置环 — 每 30 次调用执行一次 (500Hz) */
    if (m_per_count >= 30) {
        m_per_count = 0;
        m_speed = m_encoder->calc_speed_rpm_low_pass() / 60.0f;

        if (m_state == 3) {
            speed_pid_loop();       /* 输出 m_i_q_ref */
        }
        if (m_state == 4) {
            location_target_pid();  /* 输出 m_i_q_ref */
            int16_t err = m_location_target - m_location;
            if (err < ERR_LOCATION && err > -ERR_LOCATION) {
                m_location_target_state = 1;
            }
        }
        if (m_state == 5) {
            direct_location_pid();  /* 输出 m_i_q_ref */
            int16_t err = m_location_target - m_location;
            if (err < ERR_LOCATION && err > -ERR_LOCATION) {
                m_location_target_state = 1;
            }
        }
    }

    /* 越界保护 */
    if (m_location > m_max_location || m_location < m_min_location) {
        m_i_q_ref = 0; m_uq = 0; m_ud = 0;
    }

    /* 电流环与状态机执行 */
    if (m_state == 1) {
        /* 开环 — 无电流环, 直接电压控制 */
        elect_angle_analog(m_speed_target);
        m_ud = 3000;
        move(m_elect_angle);
    }
    else if (m_state == 2) {
        /* 电压直接控制 — 外部设 Uq, 无电流环 */
        elect_angle_from_encoder();
        move(m_elect_angle);
    }
    else if (m_state == 3 || m_state == 4 || m_state == 5) {
        /* 闭环模式: 电流环每 15kHz 执行一次 */
        current_clarke();
        current_park();
        current_pi_loop();   /* 输出 m_uq, m_ud */

        move(m_elect_angle);
    }
    else if (m_state == 111) {
        m_check_wire_count++;
        m_uq = 4000;
        m_ud = 0;
        elect_angle_from_encoder();
        move(m_elect_angle);

        if (m_location - m_start_angle > 500 && m_location - m_start_angle < 3000) {
            m_state = 0;
        }
        if (m_check_wire_count >= 5000) {
            m_wire_swapped = true;
            m_state = 112;
            m_check_wire_count = 0;
        }
    }
    else if (m_state == 113) {
        m_uq = 0;
        m_ud = 0;
    }
}

/* ========================================================================== */
/*                      错误处理                                                */
/* ========================================================================== */

void FOC_2::error_handle() {
    if (m_state == 112) {
        elect_angle_zero_calibration();
        m_state = 0;
    }
    if (m_state == 113) {
        m_uq = 0;
        m_ud = 0;
        m_rela_location_zero = m_encoder->read_raw_angle();
        m_state = 0;
    }
}

/* ========================================================================== */
/*                      初始化                                                  */
/* ========================================================================== */

void FOC_2::init() {
    pid_param_init();
    m_elect_angle_zero = 11101;

    table_init();
    /* DRV8313_Init() 已由 main.c 中 GPIO 初始化完成 — PE8(EN) 已在 main.c 配好 */
    // DRV8313_Init();

    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_3);

    /* ADC 零点校准 (50 次平均) */
    I_Check_Calibrate(50);
    m_adc_zero_a = I_Check_Get_Offset_M1_A();
    m_adc_zero_b = I_Check_Get_Offset_M1_B();

    elect_angle_zero_calibration();
    set_volt(0, 0, 0);

    // check_wire();

    HAL_TIM_Base_Start_IT(m_htim_ctrl);
}

/* ========================================================================== */
/*                      Getter / Setter                                         */
/* ========================================================================== */

void    FOC_2::set_state(uint8_t s)          { m_state = s; }
uint8_t FOC_2::get_state() const             { return m_state; }

void    FOC_2::set_speed_target(float v)      { m_speed_target = v; }
float   FOC_2::get_speed_target() const       { return m_speed_target; }

void    FOC_2::set_location_target(int16_t v) { m_location_target = v; }
int16_t FOC_2::get_location_target() const    { return m_location_target; }

void FOC_2::set_rela_location(int16_t loc) {
    if (loc + m_rela_location_zero < ENCODER_BIT) {
        m_location_target = loc + m_rela_location_zero;
    } else {
        m_location_target = loc + m_rela_location_zero - ENCODER_BIT;
    }
}

void    FOC_2::set_location_limit(int16_t lo, int16_t hi) {
    m_max_location = hi > lo ? hi : lo;
    m_min_location = hi < lo ? hi : lo;
}

void    FOC_2::set_uq(int16_t v) { m_uq = v; }
void    FOC_2::set_ud(int16_t v) { m_ud = v; }
int16_t FOC_2::get_uq() const    { return m_uq; }
int16_t FOC_2::get_ud() const    { return m_ud; }

uint16_t FOC_2::get_elect_angle() const  { return (uint16_t)m_elect_angle; }
int16_t  FOC_2::get_location()    const { return m_location; }
float    FOC_2::get_speed()       const { return m_speed; }
int32_t  FOC_2::get_rotation_count() const { return m_encoder->get_rotation_count(); }
int16_t  FOC_2::get_elect_angle_zero() const { return m_elect_angle_zero; }

control_PID &FOC_2::speed_pid()            { return m_speed_pid; }
control_PID &FOC_2::location_pid()         { return m_location_pid; }
control_PID &FOC_2::direct_location0_pid() { return m_direct_location0_pid; }
control_PID &FOC_2::direct_location1_pid() { return m_direct_location1_pid; }

// s_FOC::m_elect_angle g_defaultFOC
// sync_globals()