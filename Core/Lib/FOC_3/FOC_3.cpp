//
// Created on 2026/7/4.
//
// FOC_3 — 纯净 C++ FOC，无 C 兼容层、无全局变量、无 FOC_1 包袱.
//

#include "FOC_3.h"
#include "tim.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern "C" {
#include "check_I.h"
#include "Serial.h"
}

#define M_ENABLE HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
#define M_DISABLE HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);

/* ========================================================================== */
/*                      构造函数                                                */
/* ========================================================================== */
FOC_3* FOC_3::instances[4] = {nullptr};
uint8_t FOC_3::motorID_debug = 0;          // 初始化为 0
uint8_t FOC_3::motorID_list[4] = {255,255,255,255};
uint8_t FOC_3::pid_debug = 0;              // 初始化为 0
bool FOC_3::driver_en = false;

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
      m_rela_location_zero(0),
      m_max_location(8191),
      m_min_location(-8192),
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
      m_i_q_target(0), m_i_d_target(0),
      m_i_q_ref(0.0f), m_i_d_ref(0.0f),
      m_adc_zero_a(2048), m_adc_zero_b(2048)
{
    instances[cfg.id] = this;
}

/* ========================================================================== */
/*                      初始化                                                  */
/* ========================================================================== */
void FOC_3::m_enable() {
    M_ENABLE ;
}
void FOC_3::m_disable() {
    M_DISABLE ;
}

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
    HAL_TIM_Base_Start_IT(m_htim_ctrl);
}

/* ========================================================================== */
/*                      零位校准                                                */
/* ========================================================================== */

void FOC_3::elect_angle_zero_calibration() {
    HAL_TIM_Base_Stop_IT(m_htim_pwm);

    m_enable();
    m_state = 1;
    m_ud = 6000;
    m_uq = 0;
    move(0);
    osDelay(600);
    m_elect_angle_zero = m_encoder->refresh_raw();
    while (m_elect_angle_zero == 16383) {
    m_elect_angle_zero = m_encoder->refresh_raw();
    }
    m_uq = 0;
    m_ud = 0;
    set_volt(0, 0, 0);
    m_state = 0;
    m_disable();
    HAL_TIM_Base_Start_IT(m_htim_ctrl);
    int16_t before = m_encoder->refresh_raw();  // 使能前读一次
    Serial_SendVar("before", before);
}

/* ========================================================================== */
/*                      电角度                                                  */
/* ========================================================================== */

void FOC_3::elect_angle_from_encoder() {
    int16_t raw = m_encoder->get_angle_14bit();
    uint16_t temp = 0;
    if (raw >= m_elect_angle_zero)
        temp = raw - m_elect_angle_zero;
    else
        temp = raw + MT6701::RESOLUTION - m_elect_angle_zero;

    m_elect_angle = (int16_t)(((int32_t)temp * m_pole_pairs * 360 / (MT6701::RESOLUTION - 1)) % 360);
}

void FOC_3::elect_angle_analog(float rad_s) {
    m_elect_angle_analog += rad_s * (float)m_pole_pairs * 360.0f / 15000.0f;
    if (m_elect_angle_analog >= 360.0f) m_elect_angle_analog -= 360.0f;
    if (m_elect_angle_analog < 0.0f)    m_elect_angle_analog += 360.0f;
    m_elect_angle_A = (int16_t)m_elect_angle_analog;
}

/* ========================================================================== */
/*                      电流输入 & 坐标变换                                      */
/* ========================================================================== */

int16_t test_i_a,test_i_b,test_i_c;
void FOC_3::current_handle() {
    m_get_current(&m_i_a, &m_i_b);

    test_i_a = m_i_a;
    test_i_b = m_i_b;

    current_clarke(m_i_a, m_i_b);
    current_park();
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
    int16_t test =  m_cos_table[100];
    int16_t cos_t = m_cos_table[angle];
    int16_t sin_t = m_sin_table[angle];

    /* 逆 Park (与正 Park 的 Iq 符号一致): V_alpha = Ud*cos + Uq*sin,  V_beta = Ud*sin - Uq*cos */
    m_u_alpha = mul_q15(m_ud, cos_t) + mul_q15(-m_uq, sin_t);
    m_u_beta  = mul_q15(m_ud, sin_t) - mul_q15(-m_uq, cos_t);

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

int16_t test_u_a,test_u_b;

void FOC_3::move(int16_t angle) {
    inv_park_clark_q15(angle);
    svpwm_calculate();
    set_volt(m_ua, m_ub, m_uc);
    test_u_a = m_ua;
    test_u_b = m_ub;
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


int16_t test_i_q,test_i_d;
void FOC_3::current_park() {
    int16_t cos_t = m_cos_table[m_elect_angle];
    int16_t sin_t = m_sin_table[m_elect_angle];

    m_i_q_q15 = (int16_t)-((int32_t) mul_q15(m_i_alpha_q15, sin_t) - mul_q15(m_i_beta_q15, cos_t));
    m_i_d_q15 = (int16_t)((int32_t) mul_q15(m_i_alpha_q15, cos_t) + mul_q15(m_i_beta_q15, sin_t));
    test_i_q = m_i_q_q15;
    test_i_d = m_i_d_q15;
}

    float test_curdata[3];
void FOC_3::current_pi_loop() {
    /* Q15 */
    float target = (float)m_i_q_target;
    if (m_cogging_enable)
        target += (float)m_cogging_table[m_elect_angle];
    m_current_q_obj.Target = target;
    m_current_q_obj.Actual = (float)m_i_q_q15;
    m_uq = m_i_q_target +(int16_t)PID_Calculate_debug(m_current_q_pid, &m_current_q_obj,test_curdata) ;

    m_current_d_obj.Target = 0.0f;
    m_current_d_obj.Actual = (float)m_i_d_q15;
    m_ud = (int16_t)PID_Calculate(m_current_d_pid, &m_current_d_obj);
}

/* ========================================================================== */
/*                      PID 控制环                                              */
/* ========================================================================== */


void FOC_3::speed_pid_loop() {
    m_speed_obj.Target = m_speed_target;
    if (m_location>m_max_location+500 || m_location<m_min_location-500){m_speed_obj.Target=0;}

    m_speed_obj.Actual = m_speed;
    m_i_q_target = PID_Calculate(m_speed_pid, &m_speed_obj);
}

void FOC_3::location_target_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    int16_t pos_err = m_location_target - m_location;
    if (pos_err > 8192)       pos_err -= 16384;
    else if (pos_err < -8192) pos_err += 16384;

    m_location_obj.Target = (float)(m_location + pos_err);
    m_location_obj.Actual = (float)m_location;

    m_speed_target = PID_Calculate(m_location_pid, &m_location_obj);
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
    int16_t m_location_temp = m_encoder->refresh_raw()- m_rela_location_zero;
    if (m_location_temp > 8191) {
        m_location = m_location_temp - 16384;
    } else if (m_location_temp < -8191) {
        m_location = m_location_temp + 16384;
    } else {
        m_location = m_location_temp;
    }


    elect_angle_from_encoder();
    current_handle();
    int16_t m_i_a, m_i_b;
    m_per_count++;

    if (m_per_count % 30 == 0) {
         if (m_state == 4) {
             location_target_pid();
             move(m_elect_angle);
         }
    }
    if (m_per_count % 15 == 0) {
        m_speed = m_encoder->calc_speed_rpm() / 60.0f;
        if (m_state == 3 || m_state == 4) {
            speed_pid_loop();
            move(m_elect_angle);
        }
    }

    if (m_state == 0) {
        m_uq = 0;
        m_ud = 0;
        move(m_elect_angle);
    }
    if (m_state == 1) {
        elect_angle_from_encoder();

        elect_angle_analog(m_speed_analog);
        // m_ud = 6000;
        // m_uq = 0;
        move(m_elect_angle_A);
    }
    else if (m_state == 2) {
        // m_ud = 0;
        elect_angle_from_encoder();
        // elect_angle_analog(m_speed_analog);
        move(m_elect_angle);
    }
    else if (m_state == 6 || m_state == 3|| m_state == 4) {
        if (driver_en) {
            current_pi_loop();
        }
        elect_angle_from_encoder();
        move(m_elect_angle);
    }


    /* ===== 齿槽补偿学习 (开环拖拽, 模拟角度) ===== */
    else if (m_state == 121) {
        static int16_t last_angle = -1;
        elect_angle_analog(0.2);
        current_handle();
        m_ud = 5000; m_uq = 0;
        move(m_elect_angle_A);

        /* 遍历中间所有角度，防止跳步漏采 */
        int16_t cur = m_elect_angle_A;
        if (last_angle >= 0) {
            if (cur > last_angle) {
                for (int16_t a = last_angle + 1; a <= cur; a++) {
                    m_cogging_table[a] += m_i_q_q15;
                    m_cogging_count[a]++;
                }
            } else if (cur < last_angle) {
                for (int16_t a = last_angle + 1; a < 360; a++) {
                    m_cogging_table[a] += m_i_q_q15;
                    m_cogging_count[a]++;
                }
                for (int16_t a = 0; a <= cur; a++) {
                    m_cogging_table[a] += m_i_q_q15;
                    m_cogging_count[a]++;
                }
                m_cogging_rounds++;
            }
        }
        last_angle = cur;

        if (m_cogging_rounds >= 5) {
            cogging_learn_stop();
        }
    }


    else if (m_state == 111) {
        m_check_wire_count++;
        m_uq = 4000; m_ud = 0;
        elect_angle_from_encoder();
        move(m_elect_angle);
        if (m_location - m_start_angle > 500 && m_location - m_start_angle < 3000) m_state = 0;
        if (m_check_wire_count >= 5000) { m_wire_swapped = true; m_state = 112; m_check_wire_count = 0; }
    }
    else if (m_state == 113) {
        m_uq = 0; m_ud = 0;
    }
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
/*                  齿槽补偿                                                   */
/* ========================================================================== */

void FOC_3::cogging_learn_start() {
    /* 清空补偿表和计数 */
    for (uint16_t i = 0; i < 360; i++) {
        m_cogging_table[i] = 0;
        m_cogging_count[i] = 0;
    }
    m_cogging_rounds = 0;
    m_cogging_enable = false;
    m_elect_angle_analog = 0.0f;
    m_state = 121;
}

void FOC_3::cogging_learn_stop() {
    /* 均化补偿表 */
    for (uint16_t i = 0; i < 360; i++) {
        if (m_cogging_count[i] > 0)
            m_cogging_table[i] /= (int16_t)m_cogging_count[i];
    }
    m_state = 0;
    m_cogging_rounds = 0;
}

/* ========================================================================== */
/*                  串口命令解析 (VOFA+ 键值对)                                */
/* ========================================================================== */

void FOC_3::parse_keyValue(char * cmd)
{
    char *key = strtok(cmd, ":");  // 支持空格、逗号、等号分隔
    char *val_str = strtok(NULL, ":");
    float val = atof(val_str);
    if (strcmp(key, "M_debug") == 0) {
        motorID_debug = (uint8_t)val;
    }else if (strcmp(key, "stop") == 0) {
        driver_en = false;
        m_disable();
    }else if (strcmp(key, "start") == 0) {
        driver_en = true;
        for (uint8_t i = 0; i < 4; i++) {
            if (instances[i]) {
                PID_Reset(instances[i]->m_speed_pid);
                PID_Reset(instances[i]->m_location_pid);
                PID_Reset(instances[i]->m_current_q_pid);
            }
        }
        m_enable();
    }
    FOC_3 *m = instances[motorID_debug];
    if (m){
        /* 选择 PID 环: M_pid:0~4 */
        if (strcmp(key, "M_pid") == 0) {
            pid_debug = (uint8_t)val;
        }
        if (strcmp(key, "wait") == 0) {
            m->m_state = 0;
        }
        else if (strcmp(key, "Uq") == 0) {
            m->m_state = 2;
            m->m_uq = (int16_t)val;          /* 电流 Q15 */
        } else if (strcmp(key, "Ud") == 0) {
            m->m_ud = (int16_t)val;          /* 电流 Q15 */
        }
        else if (strcmp(key, "Iq") == 0) {
            m->m_i_q_target = (int16_t)val;          /* 电流 Q15 */
            m->m_state = 6;
        }
        else if (strcmp(key, "speed") == 0) {
            m->m_speed_target = val;          /* 速度闭环目标 (RPM) */
            m->m_state = 3;
        }
        else if (strcmp(key, "drag") == 0) {
            m->m_speed_analog = val;          /* 开环拖拽速度 */
            m->m_state = 1;
        }
        else if (strcmp(key, "pos") == 0) {
            m->m_location_target = (int16_t)val;  /* 位置目标 (14-bit) */
            m->m_state = 4;
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
            // Serial_SendVar("kp", pid->kp);
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
            // Serial_SendVar("ki", pid->ki);
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
            // Serial_SendVar("kd", pid->kd);
        }
        /* ===== 设置命令 ===== */
        else if (strcmp(key, "location_zero") == 0) {
            m->set_location_zero(m->m_encoder->get_angle_14bit());
            m->set_location_target(0);
            PID_Reset(m->m_location_pid);
            PID_Reset(m->m_speed_pid);
        }
        else if (strcmp(key, "xypos") == 0) {
            /* 格式: xylocation:<x>,<y>  → x→motor0位置, y→motor1位置 */
            char *x_str = strtok(val_str, ",");
            char *y_str = strtok(NULL, ",");
            if (x_str && y_str) {
                FOC_3 *mx = instances[1];
                FOC_3 *my = instances[0];
                if (mx) { mx->m_location_target = -(int16_t)atoi(x_str); mx->m_state = 4; }
                if (my) { my->m_location_target = -(int16_t)atoi(y_str); my->m_state = 4; }
            }
        }

        /* ===== 齿槽补偿命令 ===== */
        else if (strcmp(key, "cogging") == 0) {
            if ((int)val == 1) m->cogging_learn_start();
            else m->cogging_learn_stop();
        }
        else if (strcmp(key, "cogging_en") == 0) {
            m->m_cogging_enable = ((int)val == 1);
        }
        else if (strcmp(key, "cogging_dump") == 0) {
            char buf[128];
            for (uint16_t i = 0; i < 360; i++) {
                snprintf(buf, sizeof(buf), "%d%c", m->m_cogging_table[i],
                         (i == 359) ? '\n' : ',');
                Serial_SendString(buf);
            }
        }
        else if (strcmp(key, "cogging_load") == 0) {
            char *p = val_str;
            for (uint16_t i = 0; i < 360 && p; i++) {
                m->m_cogging_table[i] = (int16_t)atoi(p);
                p = strtok(NULL, ",");
            }
        }
    }
}
/* ===================================调试信息==================================== */

void FOC_3::showPID_cur(float *data) {
    FOC_3 *m = instances[motorID_debug];
    data[0] = m->m_current_q_obj.Target;
    data[1] = m->m_current_q_obj.Actual;
    data[2] = m->m_current_q_obj.Out;
}
void FOC_3::showPID_speed(float *data) {
    FOC_3 *m = instances[motorID_debug];
    data[0] = m->m_speed_obj.Target;
    data[1] = m->m_speed_obj.Actual;
    data[2] = m->m_speed_obj.Out;
}
void FOC_3::showPID_location(float *data) {
    FOC_3 *m = instances[motorID_debug];
    data[0] = m->m_location_obj.Target;
    data[1] = m->m_location_obj.Actual;
    data[2] = m->m_location_obj.Out;
}

void FOC_3::Motors_period() {
    for (uint8_t i = 0; i < 4; i++) {
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

// void FOC_3::show