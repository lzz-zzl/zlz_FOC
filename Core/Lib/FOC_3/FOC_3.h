//
// Created on 2026/7/4.
//
// FOC_3 — 纯净 C++ FOC，无 C 兼容层、无全局变量、无 FOC_1 包袱.
//

#ifndef TI_FOC_FOC_3_H
#define TI_FOC_FOC_3_H

#include "stm32f4xx_hal.h"
#include "MT6701.h"
#include "Control.h"

class FOC_3 {
public:
    static FOC_3 *instances[4];
    static uint8_t motorID_debug;
    static uint8_t motorID_list[4];
    static uint8_t pid_debug;        /* 0=cur_q, 1=cur_d, 2=speed, 3=location, 4=direct */
    static bool driver_en;

    uint32_t  m_per_count;

    struct Config {
        MT6701            *encoder;
        void (*get_current)(int16_t *i_a, int16_t *i_b);
        TIM_HandleTypeDef *htim_pwm;        // TIM1
        TIM_HandleTypeDef *htim_ctrl;       // TIM5
        uint8_t            pole_pairs;
        float              u_bus;           // 母线电压 (V)
        float              r_wire;          // 相电阻 (Ω)
        float              i_max;           // 最大电流 (A)
        uint8_t            id = 0;          // 电机 ID
        const char        *name = nullptr;  // 电机名称
    };

    explicit FOC_3(const Config &cfg);

    /* 三环 PID 指针 (传入外部已初始化的 PID) */
    struct PIDCtrl {
        control_PID *current_q = nullptr;
        control_PID *current_d = nullptr;
        control_PID *speed     = nullptr;
        control_PID *location  = nullptr;
        control_PID *direct    = nullptr;   // 直接位置
    };

    /* ---------- 生命周期 ---------- */
    void init(const PIDCtrl &pid);
    void period();              // 15kHz ISR 调用
    void error_handle();        // FOC_task 中调用
    static void Motors_period();

    /* ---------- 状态控制 ---------- */
    void     set_state(uint8_t s)      { m_state = s; }
    uint8_t  get_state() const         { return m_state; }

    void      set_iq_target(int16_t v)       {m_i_q_target = v;}
    int16_t   get_iq_target() const    { return m_i_q_target; }
    void      set_id_target(int16_t v)       {m_i_d_target = v;}
    int16_t   get_id_target() const    { return m_i_d_target; }

    void     set_speed_target(float rpm) { m_speed_target = rpm; }
    float    get_speed_target() const    { return m_speed_target; }

    void     set_location_target(int16_t v) { m_location_target = v; }
    int16_t  get_location_target() const    { return m_location_target; }

    void     set_uq(int16_t v) { m_uq = v; }
    void     set_ud(int16_t v) { m_ud = v; }
    int16_t  get_uq() const    { return m_uq; }
    int16_t  get_ud() const    { return m_ud; }

    /* ---------- 标识 ---------- */
    uint8_t  get_id()    const { return m_id; }
    const char *get_name() const { return m_name; }

    /* ---------- 只读 ---------- */
    uint16_t get_elect_angle() const { return (uint16_t)m_elect_angle; }
    uint16_t get_elect_angle_A() const { return (uint16_t)m_elect_angle_A; }
    int16_t  get_location()    const { return m_location; }
    float    get_speed()       const { return m_speed; }
    int32_t  get_rotation_count() const { return m_encoder->get_rotation_count(); }
    int16_t  get_elect_angle_zero() const { return m_elect_angle_zero; }

    uint16_t get_ua() const { return m_ua; }
    uint16_t get_ub() const { return m_ub; }
    uint16_t get_uc() const { return m_uc; }

    float    get_iq_ref()  const { return m_i_q_ref; }
    int16_t  get_iq_q15()  const { return m_i_q_q15; }

    /* ---------- PID 访问 (指针, 可能为 nullptr) ---------- */
    control_PID *speed_pid()             { return m_speed_pid; }
    control_PID *location_pid()          { return m_location_pid; }
    control_PID *direct_pid()            { return m_direct_pid; }
    control_PID *current_q_pid()         { return m_current_q_pid; }
    control_PID *current_d_pid()         { return m_current_d_pid; }

    /* ---------- 调试 ---------- */
    void elect_angle_analog(float rad_s);
    void elect_angle_from_encoder();
    static void showPID_cur(float *data);
    static void showPID_speed(float *data);
    static void showPID_location(float *data);


    /* 串口命令解析 (VOFA+ 键值对) */
    static void parse_keyValue(char * cmd);
    static void ParseCmd(const char *key, float val);  /* 使用默认实例 */

private:
    /* 硬件 */
    MT6701            *m_encoder;
    void             (*m_get_current)(int16_t *i_a, int16_t *i_b);
    TIM_HandleTypeDef *m_htim_pwm;
    TIM_HandleTypeDef *m_htim_ctrl;
    uint8_t            m_pole_pairs;
    float              m_u_bus;
    float              m_r_wire;
    float              m_i_max;
    uint8_t            m_id;
    const char        *m_name;

    /* 状态 */
    uint8_t  m_state;

    // int16_t m_Iq;
    // int16_t m_Id;
    int16_t  m_elect_angle;
    int16_t  m_elect_angle_A;
    int16_t  m_elect_angle_zero = 0;

    uint16_t m_check_wire_count;
    float    m_elect_angle_analog;
    int16_t  m_start_angle;

    int16_t  m_location;
    int16_t  m_location_target;
    int16_t  m_rela_location_zero;
    int16_t  m_max_location;
    int16_t  m_min_location;
    int8_t   m_location_target_state;

    float    m_speed;
    float    m_speed_target;
    float    m_speed_limit;

    float     m_speed_analog;

    int16_t  m_i_q_q15, m_i_d_q15;
    int16_t  m_i_q_target, m_i_d_target;



    /* 电流反馈 (Q15) */
    int16_t  m_i_a, m_i_b;
    int16_t  m_i_alpha_q15, m_i_beta_q15;
    float    m_i_q_ref, m_i_d_ref;
    int16_t  m_adc_zero_a, m_adc_zero_b;

    /* PWM 电压 (Q15) */
    int16_t  m_uq, m_ud;
    int16_t  m_u_alpha, m_u_beta;
    uint16_t m_ua, m_ub, m_uc;
    bool     m_wire_swapped;

    /* PID (外部指针, 传入后绑定) */
    control_PID    *m_speed_pid     = nullptr;
    control_PID    *m_location_pid  = nullptr;
    control_PID    *m_direct_pid    = nullptr;
    control_PID    *m_current_q_pid = nullptr;
    control_PID    *m_current_d_pid = nullptr;
    control_object m_speed_obj;
    control_object m_location_obj;
    control_object m_current_q_obj;
    control_object m_current_d_obj;

    /* sin/cos 查表 */
    inline static int16_t m_sin_table[360] ={};
    inline static int16_t m_cos_table[360] ={};
    inline static bool if_tableInit = false;

    /* ---------- 齿槽补偿 ---------- */
    int16_t  m_cogging_table[360];  // Q15 补偿值
    uint16_t m_cogging_count[360];  // 学习计数
    uint8_t  m_cogging_rounds;     // 已学习圈数
    bool     m_cogging_enable;     // 启用补偿
    void cogging_learn_start();
    void cogging_learn_stop();

    /* 内部方法 */
    static void m_enable();
    static void m_disable();

    void table_init();
    void elect_angle_zero_calibration();
    void check_wire();

    void current_handle();
    void set_volt(int16_t ua, int16_t ub, int16_t uc);
    void inv_park_clark_q15(int16_t angle);
    void svpwm_calculate();
    void move(int16_t angle);

    void current_clarke(int16_t i_a, int16_t i_b);
    void current_park();
    void current_pi_loop();
    void speed_pid_loop();
    void location_target_pid();
    void direct_location_pid();

    static inline int16_t mul_q15(int16_t a, int16_t b) {
        return (int16_t)(((int32_t)a * b) >> 15);
    }
    static inline uint16_t min_three(uint16_t a, uint16_t b, uint16_t c) {
        uint16_t m = a;
        if (b < m) m = b;
        if (c < m) m = c;
        return m;
    }

    static constexpr float    SQRT_3         = 0.8660254f;
    static constexpr int16_t  INV_SQRT3_Q15  = 18919;   // 1/√3 × 32767
    static constexpr int16_t  VOLTAGE_Q15_MAX = 32767;
    static constexpr int16_t  VOLTAGE_Q15_MIN = -32768;
    static constexpr int32_t  Q16             = 65536;
    static constexpr int32_t  Q16_D_115       = 56987;   // Q16/1.15
    static constexpr int16_t  Q15_SCALE       = 32767;
    static constexpr uint16_t TABLE_SIZE      = 360;
    static constexpr uint16_t PWM_PERIOD      = 699;
    static constexpr uint16_t ENCODER_BIT     = 16383;
    static constexpr int16_t  ERR_LOCATION    = 10;
    static constexpr int16_t  LOCATION_TIPPING = 200;
};

/* 全局默认实例 (与 FOC_2 同一套硬件) */
// FOC_3 *FOC_3_Init(void);

#ifdef __cplusplus
extern "C" {
#endif

    void FOC_3_period();
    void FOC_3_parse_keyValue(char * cmd);

#ifdef __cplusplus
}
#endif






#endif // TI_FOC_FOC_3_H