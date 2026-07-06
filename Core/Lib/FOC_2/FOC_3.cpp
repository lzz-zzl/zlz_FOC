//
// Created on 2026/7/4.
//
// FOC_3 — 纯净 C++ FOC，无 C 兼容层、无全局变量、无 FOC_1 包袱.
//

#include "FOC_3.h"
#include "tim.h"
#include <string.h>
#include <math.h>

extern "C" {
#include "check_I.h"
#include "Serial.h"
}

/* ========================================================================== */
/*                      构造函数                                                */
/* ========================================================================== */
FOC_3* FOC_3::instances[16] = {nullptr};
uint8_t FOC_3::motorID_debug = 0;          // 初始化为 0
uint8_t FOC_3::pid_debug = 0;              // 初始化为 0

FOC_3::FOC_3(const Config &cfg)
    : m_encoder(cfg.encoder),
      m_get_current(cfg.get_current),
      m_htim_pwm(cfg.htim_pwm),
      m_htim_ctrl(cfg.htim_ctrl),
      m_pole_pairs(cfg.pole_pairs),
      m_u_bus(cfg.u_bus),
      m_r_wire(cfg.r_wire),
      m_i_max(cfg.i_max),
      m_id(cfg.id),
      m_name(cfg.name),
      m_state(0),
      m_location(0),
      m_location_target(4000),
      m_speed(0.0f),
      m_speed_target(0.0f),
      m_elect_angle(0),
      m_elect_angle_zero(11101),
      m_rela_location_zero(11595),
      m_max_location(16383),
      m_min_location(0),
      m_location_target_state(0),
      m_per_count(0),
      m_check_wire_count(0),
      m_elect_angle_analog(0.0f),
      m_start_angle(0),
      m_uq(0), m_ud(0),
      m_u_alpha(0), m_u_beta(0),
      m_ua(0), m_ub(0), m_uc(0),
      m_wire_swapped(false),
      m_i_alpha_q15(0), m_i_beta_q15(0),
      m_i_q_q15(0), m_i_d_q15(0),
      m_i_q_ref(0.0f), m_i_d_ref(0.0f),
      m_adc_zero_a(2048), m_adc_zero_b(2048)
{
    instances[cfg.id] = this;
    for (auto &v : m_sin_table) v = 0;
    for (auto &v : m_cos_table) v = 0;
}

/* ========================================================================== */
/*                      初始化                                                  */
/* ========================================================================== */

void FOC_3::table_init() {
    if (if_tableInit == false) {
        for (uint16_t i = 0; i < TABLE_SIZE; i++) {
            float rad = (float)i * 3.1415926f / 180.0f;
            m_cos_table[i] = (int16_t)(cosf(rad) * Q15_SCALE);
            m_sin_table[i] = (int16_t)(sinf(rad) * Q15_SCALE);
        }
        if_tableInit = true;
    }
}

void FOC_3::init(const PIDCtrl &pid) {
    m_current_q_pid = pid.current_q;
    m_current_d_pid = pid.current_d;
    m_speed_pid     = pid.speed;
    m_location_pid  = pid.location;
    m_direct_pid    = pid.direct;
    table_init();

    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(m_htim_pwm, TIM_CHANNEL_3);

    I_Check_Calibrate(50);
    m_adc_zero_a = I_Check_Get_Offset_M1_A();
    m_adc_zero_b = I_Check_Get_Offset_M1_B();

    elect_angle_zero_calibration();
    set_volt(0, 0, 0);

    HAL_TIM_Base_Start_IT(m_htim_pwm);
}

/* ========================================================================== */
/*                      零位校准                                                */
/* ========================================================================== */

void FOC_3::elect_angle_zero_calibration() {
    HAL_TIM_Base_Stop_IT(m_htim_ctrl);
    m_ud = 3000;
    m_uq = 0;
    move(0);
    osDelay(1000);
    m_elect_angle_zero = m_encoder->refresh_raw();
    m_elect_angle_zero = m_encoder->refresh_raw();
    m_uq = 0;
    m_ud = 0;
    set_volt(0, 0, 0);
    HAL_TIM_Base_Start_IT(m_htim_ctrl);
}

/* ========================================================================== */
/*                      电角度                                                  */
/* ========================================================================== */

void FOC_3::elect_angle_from_encoder() {
    uint16_t temp = 0;
    if (m_location >= m_elect_angle_zero)
        temp = m_location - m_elect_angle_zero;
    else
        temp = m_location + MT6701::RESOLUTION - m_elect_angle_zero;

    m_elect_angle = (int16_t)(((int32_t)temp * m_pole_pairs * 360 / (MT6701::RESOLUTION - 1)) % 360);
}

void FOC_3::elect_angle_analog(float rad_s) {
    m_elect_angle_analog += rad_s * (float)m_pole_pairs * 360.0f / 15000.0f;
    if (m_elect_angle_analog >= 360.0f) m_elect_angle_analog -= 360.0f;
    if (m_elect_angle_analog < 0.0f)    m_elect_angle_analog += 360.0f;
    m_elect_angle = (int16_t)m_elect_angle_analog;
}

/* ========================================================================== */
/*                      电流输入 & 坐标变换                                      */
/* ========================================================================== */
void FOC_3::current_handle() {
    int16_t i_a, i_b;
    m_get_current(&i_a, &i_b);

    current_clarke(i_a, i_b);
    current_park();
    current_pi_loop();
}


/* ========================================================================== */
/*                      电压输出 & 坐标变换                                      */
/* ========================================================================== */

void FOC_3::set_volt(int16_t ua, int16_t ub, int16_t uc) {
    if (m_wire_swapped) {
        int16_t tmp = ub; ub = uc; uc = tmp;
    }
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_1, (uint32_t)ua * PWM_PERIOD / Q16_D_115);
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_2, (uint32_t)ub * PWM_PERIOD / Q16_D_115);
    __HAL_TIM_SetCompare(m_htim_pwm, TIM_CHANNEL_3, (uint32_t)uc * PWM_PERIOD / Q16_D_115);
}

void FOC_3::inv_park_clark_q15(int16_t angle) {
    int16_t cos_t = m_cos_table[angle];
    int16_t sin_t = m_sin_table[angle];

    m_u_alpha = (mul_q15(-m_uq, cos_t) - mul_q15(m_ud, sin_t));
    m_u_beta  = (mul_q15(-m_uq, sin_t) + mul_q15(m_ud, cos_t));

    static const int16_t sqrt3_q15 = (int16_t)(SQRT_3 * Q15_SCALE);
    static const int16_t half_q15  = Q15_SCALE / 2;

    m_ua = m_u_alpha + VOLTAGE_Q15_MAX;
    m_ub = mul_q15((int16_t)(-m_u_alpha), half_q15) + mul_q15(m_u_beta, sqrt3_q15) + VOLTAGE_Q15_MAX;
    m_uc = mul_q15((int16_t)(-m_u_alpha), half_q15) - mul_q15(m_u_beta, sqrt3_q15) + VOLTAGE_Q15_MAX;
}

void FOC_3::svpwm_calculate() {
    uint16_t u_min = min_three(m_ua, m_ub, m_uc);
    m_ua -= u_min;
    m_ub -= u_min;
    m_uc -= u_min;
}

void FOC_3::move(int16_t angle) {
    inv_park_clark_q15(angle);
    svpwm_calculate();
    set_volt(m_ua, m_ub, m_uc);
}

/* ========================================================================== */
/*                      电流环 (Q15)                                            */
/* ========================================================================== */

void FOC_3::current_clarke(int16_t i_a, int16_t i_b) {
    /* ADC raw ±2048 → Q15: ×16 */
    int32_t ia_q15 = (int32_t)i_a * 16;
    int32_t ib_q15 = (int32_t)i_b * 16;

    m_i_alpha_q15 = (int16_t)ia_q15;
    m_i_beta_q15  = (int16_t)(((ia_q15 + 2 * ib_q15) * INV_SQRT3_Q15) >> 15);
}

void FOC_3::current_park() {
    int16_t cos_t = m_cos_table[m_elect_angle];
    int16_t sin_t = m_sin_table[m_elect_angle];

    m_i_q_q15 = (int16_t)((int32_t)-mul_q15(m_i_alpha_q15, sin_t) + mul_q15(m_i_beta_q15, cos_t));
    m_i_d_q15 = (int16_t)((int32_t) mul_q15(m_i_alpha_q15, cos_t) + mul_q15(m_i_beta_q15, sin_t));
}

void FOC_3::current_pi_loop() {
    /* Q15 → float (÷16 恢复到 ADC 单位范围) */
    m_current_q_obj.Target = m_i_q_ref;
    m_current_q_obj.Actual = (float)m_i_q_q15 / 16.0f;
    m_uq = (int16_t)PID_Calculate(m_current_q_pid, &m_current_q_obj);

    m_current_d_obj.Target = 0.0f;
    m_current_d_obj.Actual = (float)m_i_d_q15 / 16.0f;
    m_ud = (int16_t)PID_Calculate(m_current_d_pid, &m_current_d_obj);
}

/* ========================================================================== */
/*                      PID 控制环                                              */
/* ========================================================================== */

void FOC_3::speed_pid_loop() {
    m_speed_obj.Target = m_speed_target;
    m_speed_obj.Actual = m_speed;
    m_i_q_ref = PID_Calculate(m_speed_pid, &m_speed_obj);
}

void FOC_3::location_target_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    int16_t pos_err = m_location_target - m_location;
    if (pos_err > 8192)       pos_err -= 16384;
    else if (pos_err < -8192) pos_err += 16384;

    m_location_obj.Target = (float)(m_location + pos_err);
    m_location_obj.Actual = (float)m_location;

    float loc_speed = PID_Calculate(m_location_pid, &m_location_obj);
    m_speed_obj.Target = loc_speed;
    m_speed_obj.Actual = m_speed;
    m_i_q_ref = PID_Calculate(m_speed_pid, &m_speed_obj);
}

void FOC_3::direct_location_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    m_location_obj.Target = (float)m_location_target;
    m_location_obj.Actual = (float)m_location;

    m_i_q_ref = PID_Calculate(m_direct_pid, &m_location_obj);
}

/* ========================================================================== */
/*                      主周期 (15kHz ISR)                                     */
/* ========================================================================== */

void FOC_3::period() {
    m_location = m_encoder->refresh_raw();
    // elect_angle_from_encoder();
    // current_handle();
    int16_t m_i_a, m_i_b;

    m_per_count++;
    // if (m_per_count >= 30) {
    //     m_per_count = 0;
    //     m_speed = m_encoder->calc_speed_rpm_low_pass() / 60.0f;
    //
    //     if (m_state == 3) speed_pid_loop();
    //     if (m_state == 4) location_target_pid();
    //     if (m_state == 5) direct_location_pid();
    // }

    // if (m_location > m_max_location+100 || m_location < m_min_location-100) {
    //     m_i_q_ref = 0; m_uq = 0; m_ud = 0;
    // }
    //
    // if (m_state == 1) {
    //     elect_angle_analog(m_speed_target);
    //     m_ud = 3000;
    //     move(m_elect_angle);
    // }
    // else if (m_state == 2) {
    //     elect_angle_from_encoder();
    //     move(m_elect_angle);
    // }
    // else if (m_state == 3 || m_state == 4 || m_state == 5) {
    //     current_handle();
    //     move(m_elect_angle);
    // }
    //
    //
    // else if (m_state == 111) {
    //     m_check_wire_count++;
    //     m_uq = 4000; m_ud = 0;
    //     elect_angle_from_encoder();
    //     move(m_elect_angle);
    //     if (m_location - m_start_angle > 500 && m_location - m_start_angle < 3000) m_state = 0;
    //     if (m_check_wire_count >= 5000) { m_wire_swapped = true; m_state = 112; m_check_wire_count = 0; }
    // }
    // else if (m_state == 113) {
    //     m_uq = 0; m_ud = 0;
    // }
}

/* ========================================================================== */
/*                      接线检测                                                */
/* ========================================================================== */

void FOC_3::check_wire() {
    m_state = 111;
    m_start_angle = m_encoder->refresh_low_pass();
    m_start_angle = m_encoder->refresh_low_pass();
}

/* ========================================================================== */
/*                      错误处理                                                */
/* ========================================================================== */

void FOC_3::error_handle() {
    if (m_state == 112) {
        elect_angle_zero_calibration();
        m_state = 0;
    }
    if (m_state == 113) {
        m_uq = 0; m_ud = 0;
        m_rela_location_zero = m_encoder->read_raw_angle();
        m_state = 0;
    }
}

/* ========================================================================== */
/*                  串口命令解析 (VOFA+ 键值对)                                */
/* ========================================================================== */

void FOC_3::parse_keyValue(char * cmd)
{
    char *key = strtok(cmd, " :");  // 支持空格、逗号、等号分隔
    char *val_str = strtok(NULL, " :");
    float val = atof(val_str);
    if (strcmp(key, "M_debug") == 0) {
        motorID_debug = (uint8_t)val;
    }
    FOC_3 *m = instances[motorID_debug];
    {
        /* 选择 PID 环: M_pid:0~4 */
        if (strcmp(key, "M_pid") == 0) {
            pid_debug = (uint8_t)val;
        }
        else if (strcmp(key, "iq") == 0) {
            m->m_uq = (int16_t)val;          /* 电流 Q15 */
        }
        else if (strcmp(key, "speed") == 0) {
            m->m_speed_target = val;          /* 速度闭环目标 (RPM) */
        }
        else if (strcmp(key, "drag") == 0) {
            m->m_speed_target = val;          /* 开环拖拽速度 */
        }
        else if (strcmp(key, "pos") == 0) {
            m->m_location_target = (int16_t)val;  /* 位置目标 (14-bit) */
        }
        /* 调参 kp/ki/kd — 作用于选中的 PID */
        else if (strcmp(key, "kp") == 0) {
            control_PID *pid = nullptr;
            switch (pid_debug) {
                case 0: pid = m->m_current_q_pid; break;
                case 1: pid = m->m_current_d_pid; break;
                case 2: pid = m->m_speed_pid;     break;
                case 3: pid = m->m_location_pid;  break;
                case 4: pid = m->m_direct_pid;    break;
            }
            if (pid) pid->kp = val;
            Serial_SendVar("pid",pid_debug);
            Serial_SendVar("kp", pid->kp);
        }
        else if (strcmp(key, "ki") == 0) {
            control_PID *pid = nullptr;
            switch (pid_debug) {
                case 0: pid = m->m_current_q_pid; break;
                case 1: pid = m->m_current_d_pid; break;
                case 2: pid = m->m_speed_pid;     break;
                case 3: pid = m->m_location_pid;  break;
                case 4: pid = m->m_direct_pid;    break;
            }
            if (pid) pid->ki = val;
            Serial_SendVar("pid",pid_debug);
            Serial_SendVar("ki", pid->ki);
        }
        else if (strcmp(key, "kd") == 0) {
            control_PID *pid = nullptr;
            switch (pid_debug) {
                case 0: pid = m->m_current_q_pid; break;
                case 1: pid = m->m_current_d_pid; break;
                case 2: pid = m->m_speed_pid;     break;
                case 3: pid = m->m_location_pid;  break;
                case 4: pid = m->m_direct_pid;    break;
            }
            if (pid) pid->kd = val;
            Serial_SendVar("pid",pid_debug);
            Serial_SendVar("kd", pid->kd);
        }
    }
}

void FOC_3::Motors_period() {
    for (uint8_t i = 0; i < 16; i++) {
        if (instances[i]) {
            instances[i]->period();
        }
    }
}

extern "C" void FOC_3_period() {
    FOC_3::Motors_period();
}

void FOC_3_parse_keyValue(char * cmd) {
    FOC_3::parse_keyValue(cmd);
}

