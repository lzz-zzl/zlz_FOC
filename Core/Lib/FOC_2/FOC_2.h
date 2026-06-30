//
// Created by zhang on 2026/6/3.
//

#ifndef FOC_RTOS_FOC_2_H
#define FOC_RTOS_FOC_2_H

#include "stm32f4xx_hal.h"
#include "Control.h"

#ifdef __cplusplus
#include "MT6701.h"

class FOC_2 {
public:
    struct Config {
        MT6701            *encoder;
        TIM_HandleTypeDef *htim_pwm;        // TIM1
        TIM_HandleTypeDef *htim_ctrl;       // TIM3 (15kHz ISR)
        uint8_t            pole_pairs;      // 极对数
        float              u_bus;           // 母线电压
        float              r_wire;          // 相电阻
        float              i_max;           // 最大电流限制
    };

    explicit FOC_2(const Config &cfg);

    /* ---------- 生命周期 ---------- */
    void init();
    void period();
    void error_handle();

    /* ---------- 状态/目标 ---------- */
    void     set_state(uint8_t state);
    uint8_t  get_state() const;
    void     set_speed_target(float rpm);
    float    get_speed_target() const;
    void     set_location_target(int16_t location);
    int16_t  get_location_target() const;
    void     set_rela_location(int16_t location);
    void     set_location_limit(int16_t min, int16_t max);

    /* ---------- 电压直接设定 ---------- */
    void     set_uq(int16_t v);
    void     set_ud(int16_t v);
    int16_t  get_uq() const;
    int16_t  get_ud() const;
    uint16_t get_ua() const { return m_ua; }
    uint16_t get_ub() const { return m_ub; }
    uint16_t get_uc() const { return m_uc; }

    /* ---------- 只读访问 ---------- */
    uint16_t get_elect_angle() const;
    int16_t  get_location()     const;
    float    get_speed()        const;
    int32_t  get_rotation_count() const;
    int16_t  get_elect_angle_zero() const;

    /* ---------- PID 访问 ---------- */
    control_PID &speed_pid();
    control_PID &location_pid();
    control_PID &direct_location0_pid();
    control_PID &direct_location1_pid();

    /* ---------- 编码器访问 ---------- */
    MT6701 *encoder() const { return m_encoder; }

    /* ---------- C 兼容层需要的 public 方法 ---------- */
    void elect_angle_analog(float rad_s);
    void elect_angle_from_encoder();

private:
    /* 内部方法 */
    void pid_param_init();
    void table_init();
    void elect_angle_zero_calibration();
    void check_wire();
    void speed_pid_loop();
    void location_target_pid();
    void direct_location_pid();
    void inv_park_clark_q15(int16_t angle);
    void svpwm_calculate();
    void set_volt(int16_t ua, int16_t ub, int16_t uc);
    void move(int16_t angle);
    void max_accelerate();
    static inline int16_t mul_q15(int16_t a, int16_t b);
    static inline uint16_t min_three(uint16_t a, uint16_t b, uint16_t c);

    /* ---------- 电流环 ---------- */
    void current_clarke();
    void current_park();
    void current_pi_loop();

    /* ---------- 硬件 ---------- */
    MT6701            *m_encoder;
    TIM_HandleTypeDef *m_htim_pwm;
    TIM_HandleTypeDef *m_htim_ctrl;
    uint8_t            m_pole_pairs;
    float              m_u_bus;
    float              m_r_wire;
    float              m_i_max;

    /* ---------- PWM 电压 ---------- */
    int16_t  m_uq, m_ud;
    int16_t  m_u_alpha, m_u_beta;
    uint16_t m_ua, m_ub, m_uc;
    bool     m_wire_swapped;     // true = B/C 相交换

    /* ---------- 角度/位置 ---------- */
    int16_t  m_elect_angle;
    uint8_t  m_state;
    int16_t  m_location;
    int16_t  m_location_target;
    int8_t   m_location_target_state;
    int16_t  m_max_location;
    int16_t  m_min_location;
    int16_t  m_rela_location_zero;
    int16_t  m_elect_angle_zero;

    /* ---------- 速度 ---------- */
    float    m_speed;
    float    m_speed_target;

    /* ---------- PID ---------- */
    control_PID    m_speed_pid;
    control_PID    m_location_pid;
    control_PID    m_direct_location0_pid;
    control_PID    m_direct_location1_pid;
    control_PID    m_current_q_pid;
    control_PID    m_current_d_pid;
    control_object m_speed_obj;
    control_object m_location_obj;
    control_object m_current_q_obj;
    control_object m_current_d_obj;
    lowpass_Filter  m_derivative_filter;

    /* ---------- 电流反馈 (ADC, 浮点) ---------- */
    float    m_i_a_fb;       // A 相电流 (安培等效, ADC 单位)
    float    m_i_b_fb;       // B 相电流
    float    m_i_c_fb;       // C 相电流 = -(A+B)
    float    m_i_alpha_fb;   // α 轴电流
    float    m_i_beta_fb;    // β 轴电流
    float    m_i_q_fb;       // q 轴电流反馈
    float    m_i_d_fb;       // d 轴电流反馈
    float    m_i_q_ref;      // q 轴电流目标 (来自速度环)
    float    m_i_d_ref;      // d 轴电流目标 (=0)
    int16_t  m_adc_zero_a;   // ADC 零点偏移 A
    int16_t  m_adc_zero_b;   // ADC 零点偏移 B

    /* ---------- sin/cos 查表 ---------- */
    int16_t m_sin_table[360];
    int16_t m_cos_table[360];

    /* ---------- 临时 ---------- */
    uint16_t m_start_angle;
    uint8_t  m_per_count;
    uint16_t m_check_wire_count;   // 接线检查计数器
    float    m_elect_angle_analog;  // 模拟电角度累加值
};


/* ========== 全局默认实例管理 ========== */
void        FOC_2_set_default_instance(FOC_2 *inst);
FOC_2      *FOC_2_get_default_instance();

//

#endif /* __cplusplus */

/* ========================================================================== */
/*                      extern "C" 兼容层 (FOC_1 同名接口)                     */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

extern int16_t  FOC_Uq, FOC_Ud;
extern uint16_t FOC_Ua, FOC_Ub, FOC_Uc;
extern uint8_t  FOC_state;
extern int16_t  FOC_location;
extern int16_t  FOC_location_target;
extern int8_t   FOC_location_state;
extern float    FOC_speed;
extern uint16_t elect_angle_zero;
extern int16_t  FOC_rela_location;

void    FOC_Init(void);
void    FOC_period(void);
void    FOC_Set_Uq(int16_t value);
void    FOC_Set_Ud(int16_t value);
void    FOC_SET_state(uint8_t state);
uint8_t FOC_GET_state(void);
uint16_t FOC_GET_elect_angle(void);
void    FOC_SET_speed_target(float value);
float   FOC_GET_speed_target(void);
void    FOC_SET_rela_location(int16_t location);
void    FOC_SET_location_target(int16_t location);
int16_t FOC_GET_location_target(void);
void    FOC_SET_location_limit(int16_t min, int16_t max);
void    FOC_error_handle(void);
void    FOC_elect_angle_MT6701(void);
void    FOC_elect_angle_analog(float rad_s);
void    FOC_Inv_Part_Clark_Q15(int16_t angle);
int16_t foc_test(int16_t a);
int16_t FOC_GET_elect_angle_zero(void);
void    FOC_2_create_default(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_RTOS_FOC_2_H */
