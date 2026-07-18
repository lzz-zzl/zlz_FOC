#ifndef BALL_LOCATION_CONTROL_H
#define BALL_LOCATION_CONTROL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


/* 正弦信号发生器 */
typedef struct {
    float amplitude;    /**< 振幅 */
    float period;       /**< 周期 (秒) */
    float phase;        /**< 当前相位 (弧度) */
    float offset;       /**< 直流偏置 */
} SineGen;

void SineGen_Init(SineGen *sg, float amplitude, float period, float offset);
float SineGen_Calc(SineGen *sg);


typedef struct {
    float alpha;
    float last_value;
} lowpass_Filter;
//滤波器函数指针
typedef float (*filter_fun_p)(void *, float);

typedef struct {
    float Target;
    float Actual;
    float Actual_pass;
    float Out;
} control_object;

typedef struct {
    // control_object object;
    float kp;
    float ki;
    float kd;

    float Er_limit;
    float ErMax;
    float ErMin;
    float Er_weaken;

    float Error_now;
    float Error_pass;
    float Error_calculus;
    float Error_output;

    float OutMax;
    float OutMin;

    filter_fun_p Actual_filter;
    void *Actual_filter_para;
    filter_fun_p Derivative_filter;
    void *Derivative_filter_para;
    filter_fun_p Output_filter;
    void *Output_filter_para;

} control_PID;


float lowpass_filter(void *para, float input);


void Control_PID_Init(control_PID *pid, float kp, float ki, float kd,
              float out_min, float out_max, float er_limit, float er_cal_limit);

void PID_EnableActualFilter(control_PID *pid,filter_fun_p filter, void *para);
void PID_EnableDerivativeFilter(control_PID *pid, filter_fun_p filter, void *para);  // 3个参数
void PID_EnableOutputFilter(control_PID *pid, filter_fun_p filter, void *para);



float PID_Calculate(control_PID *pid, control_object *obj);float PID_Calculate_debug(control_PID *pid, control_object *obj, float *debug_data);
float PID_Calculate_DerivativeOnMeasure(control_PID *pid, control_object *obj);
float PID_I_deadband_Calculate(control_PID *pid, control_object *obj);

void PID_Reset(control_PID *pid);

    void control_tim_start();
uint32_t get_c_tim_count();
uint32_t get_tim_dif(uint8_t i);
void tim_dif_start(uint8_t i);
uint32_t tim_dif_stop(uint8_t i);



#ifdef __cplusplus
}
#endif  


#endif //BALL_LOCATION_CONTROL_H


