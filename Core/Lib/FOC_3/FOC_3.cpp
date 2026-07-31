//
// Created on 2026/7/4.
//
// FOC_3 — 纯净 C++ FOC，无 C 兼容层、无全局变量、无 FOC_1 包袱.
//

#include "FOC_3.h"
#include "app_task.h"
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
            float rad = (float)i * 360.0f / (float)TABLE_SIZE * 3.1415926f / 180.0f;
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

    //齿槽补偿
    cogging_load_from_flash();
    m_cogging_enable = true;

    HAL_TIM_Base_Start(m_htim_pwm);
    HAL_TIM_Base_Start(m_htim_ctrl);        /* TIM5 计数但不进中断 */
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
    HAL_TIM_Base_Start(m_htim_ctrl);        /* TIM5 计数但不进中断 */
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

    m_elect_angle = (int16_t)(((int32_t)temp * m_pole_pairs * TABLE_SIZE / (MT6701::RESOLUTION - 1)) % TABLE_SIZE);
}

int16_t FOC_3::elect_angle_from_location(int16_t  location) {

    int16_t raw = location;
    uint16_t temp = 0;
    if (raw >= m_elect_angle_zero)
        temp = raw - m_elect_angle_zero;
    else
        temp = raw + MT6701::RESOLUTION - m_elect_angle_zero;
    int16_t temp_angle = (int16_t)(((int32_t)temp * m_pole_pairs * TABLE_SIZE / (MT6701::RESOLUTION - 1)) % TABLE_SIZE);
    return  temp_angle;
}


void FOC_3::elect_angle_analog(float rad_s) {
    m_elect_angle_analog += rad_s * (float)m_pole_pairs * (float)TABLE_SIZE / 15000.0f;
    if (m_elect_angle_analog >= (float)TABLE_SIZE) m_elect_angle_analog -= (float)TABLE_SIZE;
    if (m_elect_angle_analog < 0.0f)               m_elect_angle_analog += (float)TABLE_SIZE;
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
        target += (float)m_cogging_table[m_elect_angle] ;
    m_current_q_obj.Target = target;
    m_current_q_obj.Actual = (float)m_i_q_q15;
    m_uq = (int16_t)PID_Calculate_Fast(m_current_q_pid, &m_current_q_obj) ;

    m_current_d_obj.Target = 0.0f;
    m_current_d_obj.Actual = (float)m_i_d_q15;
    m_ud = (int16_t)PID_Calculate_Fast(m_current_d_pid, &m_current_d_obj);

    test_curdata[0] = m_current_d_obj.Actual -= (float)m_cogging_table[m_elect_angle];
}

/* ========================================================================== */
/*                      PID 控制环                                              */
/* ========================================================================== */


void FOC_3::speed_pid_loop() {

    m_speed_obj.Actual = m_speed;
    m_i_q_target = PID_Calculate_Fast(m_speed_pid, &m_speed_obj);
}

void FOC_3::location_target_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    int16_t pos_err = m_location_target - m_location;
    if (pos_err > 8192)       pos_err -= 16384;
    else if (pos_err < -8192) pos_err += 16384;

    m_location_obj.Target = (float)(m_location + pos_err);
    m_location_obj.Actual = (float)m_location;

    m_speed_target = PID_Calculate_DerivativeOnMeasure(m_location_pid, &m_location_obj);

    if (m_location == m_location_target) {
        m_location_target_state = 1;
    }
    else {
        m_location_target_state = 0;
    }
}

void FOC_3::direct_location_pid() {
    if (m_location_target > m_max_location) m_location_target = m_max_location;
    if (m_location_target < m_min_location) m_location_target = m_min_location;

    m_location_obj.Target = (float)m_location_target;
    m_location_obj.Actual = (float)m_location;

    m_i_q_ref = PID_Calculate_Fast(m_direct_pid, &m_location_obj);
}

/* ========================================================================== */
/*                      主周期 (15kHz ISR)                                     */
/* ========================================================================== */

void FOC_3::period() {
    tim_dif_start(5);
    tim_dif_start(4);
    m_location_origin = m_encoder->refresh_raw();
    tim_dif_stop(4);
    int16_t m_location_temp = m_location_origin- m_rela_location_zero;
    if (m_location_temp > 8191) {
        m_location = m_location_temp - 16384;
    } else if (m_location_temp < -8191) {
        m_location = m_location_temp + 16384;
    } else {
        m_location = m_location_temp;
    }

    m_speed_obj.Target = m_speed_target;
    if (m_location>m_max_location+500 || m_location<m_min_location-500){
        m_disable();
    }

    tim_dif_start(3);
    elect_angle_from_encoder();
    current_handle();

    m_per_count++;

    if (m_per_count % 30 == 0) {
         if (m_state == 4 || m_state == 121) {
             location_target_pid();
         }
    }
    if (m_per_count % 60 == 1) {
        m_speed = m_encoder->calc_speed_rpm() / 60.0f;
        if (m_state == 3 || m_state == 4 | m_state == 121) {
            speed_pid_loop();
        }
    }

    if (m_state == 0) {
        m_uq = 0;
        m_ud = 0;
        move(m_elect_angle);
    }

    tim_dif_stop(3);
    if (m_state == 1) {
        elect_angle_from_encoder();
        elect_angle_analog(m_speed_analog);
        move(m_elect_angle_A);
    }else if (m_state == 2) {
        // m_ud = 0;
        elect_angle_from_encoder();
        // elect_angle_analog(m_speed_analog);
        move(m_elect_angle);

    }else if (m_state == 6 || m_state == 3|| m_state == 4 || m_state == 121) {
    tim_dif_start(2);
        if (driver_en) {
            current_pi_loop();
        }
        elect_angle_from_encoder();
        move(m_elect_angle);
    tim_dif_stop(2);
    }


    tim_dif_stop(5);
    /* ===== 齿槽补偿学习 (正反向遍历 1440 个电角度，抵消反电动势) ===== */
    if (m_state == 121) {

        if (m_cogging_learnState == 0) {
            m_learn_angle = m_elect_angle;
            m_learn_location = m_encoder->refresh_raw();
            m_learn_count = 0;
            m_cogging_learnState = 1; /*0=开始 1=正向, 2=反向 4=未开始*/
        }

        /* learn_location 是原始编码器值 (0~16383), 转成符号位置给 PID */
        int16_t raw2pos = m_learn_location - m_rela_location_zero;
        if (raw2pos > 8191)       raw2pos -= 16384;
        else if (raw2pos < -8192) raw2pos += 16384;
        m_location_target = raw2pos;

        /* 位置到达 → 采样 */
        if (m_location == raw2pos) {
            m_cogging_table[m_learn_angle] += m_i_q_q15;
            m_cogging_count[m_learn_angle]++;
            if (m_cogging_learnState == 1) {
                if (m_cogging_count[m_learn_angle] >= 5) {
                    m_learn_angle = (m_learn_angle + 1) % TABLE_SIZE;
                    m_learn_count++;
                }
            }
            else if (m_cogging_learnState == 2) {
                if (m_cogging_count[m_learn_angle] >= 10) {
                    if (m_learn_angle == 0) {
                        m_learn_angle = TABLE_SIZE;
                    }
                    m_learn_angle = (m_learn_angle - 1) % TABLE_SIZE;
                    m_learn_count++;
                }
            }
        }

        /* 搜索对应位置的编码器值 (相位决定方向) */
        if (m_cogging_learnState == 1) {
            /* 正向 */
            while (elect_angle_from_location(m_learn_location) != m_learn_angle) {
                m_learn_location++;
                if (m_learn_location >= MT6701::RESOLUTION) m_learn_location = 0;
            }
        } else if (m_cogging_learnState == 2){
            /* 反向 */
            while (elect_angle_from_location(m_learn_location) != m_learn_angle) {
                m_learn_location--;
                if (m_learn_location < 0) m_learn_location = MT6701::RESOLUTION - 1;
            }
        }

        /* 当前相位完成 → 切换相位或停止 */
        if (m_learn_count >= TABLE_SIZE) {
            if (m_cogging_learnState == 1) {
                /* 正向完成 → 切到反向 */
                m_learn_count = 0;
                m_cogging_learnState = 2;
                m_learn_angle = m_elect_angle;
                m_learn_location = m_encoder->refresh_raw();
            } else {
                /* 反向完成 → 停止 */
                m_cogging_learnState = 4;
                cogging_learn_stop();

                m_cogging_learnState = 0;
                m_learn_count = 0;
            }
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
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {
        m_cogging_table[i] = 0;
        m_cogging_count[i] = 0;
    }
    m_cogging_rounds = 0;
    m_cogging_enable = false;
    m_elect_angle_analog = 0.0f;

    /* 初始化学习过程变量 */
    m_learn_angle    = m_elect_angle;         /* 从当前电角度开始 */
    m_learn_location = m_location;
    m_learn_count    = 0;
    m_learn_count    = 0;
    m_cogging_learnState = 0;
    m_state = 121;
}

void FOC_3::cogging_learn_stop() {
    /* 均化补偿表 */
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {
        if (m_cogging_count[i] > 0)
            m_cogging_table[i] /= (int16_t)m_cogging_count[i];
    }
    m_state = 0;
    m_cogging_rounds = 0;
}

/* ========================================================================== */
/*                  齿槽补偿 Flash 存储                                        */
/* ========================================================================== */

#define COGGING_FLASH_ADDR   0x08060000  /* 使用最后一个 Sector (Sector 7, 128KB) */
#define COGGING_FLASH_MAGIC  0xCAFE      /* 校验标记 (16-bit) */
#define COGGING_SLOT_SIZE    0x1000      /* 每个电机槽大小 (4KB, 足够 magic+1440×int16) */

void FOC_3::cogging_save_to_flash() {
    volatile uint32_t timeout;
    uint32_t sr_before;

    /* ===== 诊断：关中断之前就输出 ===== */
    Serial_SendString("cogging: stop tim\r\n");
    HAL_TIM_Base_Stop(&htim1);

    Serial_SendString("cogging: diable irq + clear sr\r\n");
    /* 关中断：Flash 擦写期间总线 stall */
    __disable_irq();

    sr_before = FLASH->SR;
    FLASH->SR = sr_before;               /* 写1清除所有标志 */

    /* 解锁 */
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    if (FLASH->CR & FLASH_CR_LOCK) {
        /* 无法输出了！直接恢复后输出 */
        FLASH->CR |= FLASH_CR_LOCK;
        __enable_irq();
        Serial_SendString("cogging unlock fail\r\n");
        HAL_TIM_Base_Start(&htim1);
        return;
    }

    /* ===== 擦除 Sector 7 ===== */
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= (2UL << 8);              /* PSIZE=2: 32-bit word erase */
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= (7UL << 3);              /* SNB=7 = Sector 7 */
    FLASH->CR |= FLASH_CR_STRT;

    timeout = 5000000;
    while (FLASH->SR & FLASH_SR_BSY) {
        timeout--;
        if (timeout == 0) break;
    }

    FLASH->SR = FLASH->SR;                /* 清标志 */
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);

    /* ===== 暂时恢复中断，输出诊断 ===== */
    __enable_irq();

    if (timeout == 0) {
        Serial_SendString("cogging: erase timeout\r\n");
        goto err_out;
    }
    if (FLASH->SR & FLASH_SR_WRPERR) {
        Serial_SendString("cogging: write protected!\r\n");
        goto err_out;
    }

    // Serial_SendString("cogging: erase ok, program\r\n");
    __disable_irq();

    /* ===== 写入 Magic + 表 ===== */
    for (uint8_t id = 0; id < 2; id++) {
        if (!instances[id]) continue;
        uint32_t addr = COGGING_FLASH_ADDR + id * COGGING_SLOT_SIZE;

        for (uint16_t i = 0; i <= TABLE_SIZE; i++) {
            uint16_t data;
            if (i == 0)
                data = (uint16_t)COGGING_FLASH_MAGIC;
            else
                data = (uint16_t)instances[id]->m_cogging_table[i - 1];

            FLASH->CR &= ~FLASH_CR_PSIZE;
            FLASH->CR |= (1UL << 8);             /* PSIZE=1: half-word */
            FLASH->CR |= FLASH_CR_PG;

            *(volatile uint16_t *)addr = data;

            timeout = 500000;
            while (FLASH->SR & FLASH_SR_BSY) {
                timeout--;
                if (timeout == 0) break;
            }
            FLASH->SR = FLASH->SR;
            FLASH->CR &= ~FLASH_CR_PG;

            if (timeout == 0) {
                __enable_irq();
                Serial_Printf("cogging: pg timeout at i=%d\r\n", i);
                goto err_out;
            }
            addr += 2;
        }
    }

    FLASH->CR |= FLASH_CR_LOCK;
    __enable_irq();
    Serial_SendString("cogging saved\r\n");
    HAL_TIM_Base_Start(&htim1);
    return;

err_out:
    FLASH->CR |= FLASH_CR_LOCK;
    HAL_TIM_Base_Start(&htim1);
}

void FOC_3::cogging_load_from_flash() {
    /* 读之前先使 D-Cache 失效，确保读到真实的 Flash 内容 */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1)
    SCB_InvalidateDCache();
#endif

    uint32_t addr = COGGING_FLASH_ADDR + m_id * COGGING_SLOT_SIZE;
    volatile uint16_t *base = (volatile uint16_t *)addr;

    /* 校验 Magic */
    if (base[0] != COGGING_FLASH_MAGIC) {
        Serial_SendString("cogging flash invalid\r\n");
        return;
    }

    volatile int16_t *src = (volatile int16_t *)(addr + 2);
    for (uint16_t i = 0; i < TABLE_SIZE; i++) {
        m_cogging_table[i] = src[i];
    }

    Serial_SendString("cogging loaded\r\n");

}

/* ========================================================================== */
/*                  串口命令解析 (VOFA+ 键值对)                                */
/* ========================================================================== */

void FOC_3::parse_keyValue(const char *key, char *val_str)
{
    float val = val_str ? atof(val_str) : 0.0f;
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
        else if (strcmp(key, "rale_pos") == 0) {
            m->set_location_rale_target((int16_t)val);  /* 位置目标 (14-bit) */
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
                case 6: pid = &ball_balance_PID;  break;
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
                case 6: pid = &ball_balance_PID;  break;
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
                case 6: pid = &ball_balance_PID;  break;
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
            uint16_t pos = 0;
            for (uint16_t i = 0; i < TABLE_SIZE; i++) {
                int len = snprintf(buf + pos, sizeof(buf) - pos, "%d%c",
                                   m->m_cogging_table[i],
                                   (i == TABLE_SIZE - 1) ? '\n' : ',');
                if (len < 0) break;
                pos += len;
                /* 缓冲区快满时一次性发送 */
                if (pos > sizeof(buf) - 12 || i == TABLE_SIZE - 1) {
                    buf[pos] = '\0';
                    Serial_SendString(buf);
                    pos = 0;
                }
            }
        }
        else if (strcmp(key, "cogging_load") == 0) {
            char *p = strtok(val_str, ",");
            for (uint16_t i = 0; i < TABLE_SIZE && p; i++) {
                m->m_cogging_table[i] = (int16_t)atoi(p);
                p = strtok(NULL, ",");
            }
        }
        else if (strcmp(key, "cogging_save") == 0) {
            m->cogging_save_to_flash();
        }
        else if (strcmp(key, "cogging_fload") == 0) {
            m->cogging_load_from_flash();
        }
    }
}
/* ===================================调试信息==================================== */

void FOC_3::showPID_cur_q(float *data) {
    FOC_3 *m = instances[motorID_debug];
    data[0] = m->m_current_q_obj.Target;
    data[1] = m->m_current_q_obj.Actual;
    data[2] = m->m_current_q_obj.Out;
}
void FOC_3::showPID_cur_d(float *data) {
    FOC_3 *m = instances[motorID_debug];
    data[0] = m->m_current_d_obj.Target;
    data[1] = m->m_current_d_obj.Actual;
    data[2] = m->m_current_d_obj.Out;
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
    tim_dif_start(1);
    FOC_3::Motors_period();
    tim_dif_stop(1);
}

void FOC_3_parse_keyValue(char *key, char *val_str) {
    FOC_3::parse_keyValue(key, val_str);
}

// void FOC_3::show