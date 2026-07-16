// ... existing code ...
#include "Control.h"
#include "tim.h"

float none_filter(void *para, float input) {
    return input;
}

float lowpass_filter(void *para, float input)
{
    lowpass_Filter *filter = (lowpass_Filter *)para;
    filter->last_value = filter->alpha * input + (1.0f - filter->alpha) * filter->last_value;
    return filter->last_value;
}

//PID控制部分
void Control_PID_Init(control_PID *pid, float kp, float ki, float kd,
              float out_min, float out_max, float er_limit,float er_cal_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->OutMin = out_min;
    pid->OutMax = out_max;

    pid->Er_limit = er_limit;
    pid->ErMax = er_cal_limit;
    pid->ErMin = -er_cal_limit;
    pid->Er_weaken = 0.0f;

    pid->Error_now = 0.0f;
    pid->Error_pass = 0.0f;
    pid->Error_calculus = 0.0f;

    pid->Error_now = 0.0f;
    pid->Error_pass = 0.0f;
    pid->Error_calculus = 0.0f;
    pid->Error_output = 0.0f;

    pid->Actual_filter = NULL;
    pid->Actual_filter_para = NULL;
    pid->Derivative_filter = NULL;
    pid->Derivative_filter_para = NULL;
    pid->Output_filter = NULL;
    pid->Output_filter_para = NULL;
}

void PID_EnableActualFilter(control_PID *pid, filter_fun_p filter, void *para){
    pid->Actual_filter = filter;
    pid->Actual_filter_para = para;
}

void PID_EnableDerivativeFilter(control_PID *pid, filter_fun_p filter, void *para){
    pid->Derivative_filter = filter;
    pid->Derivative_filter_para = para;
}

void PID_EnableOutputFilter(control_PID *pid, filter_fun_p filter, void *para){
    pid->Output_filter = filter;
    pid->Output_filter_para = para;
}

float PID_Calculate(control_PID *pid, control_object *obj){
    obj->Target = obj->Target; // Target 通常由外部赋值

    if (pid->Actual_filter != NULL && pid->Actual_filter_para != NULL) {
        obj->Actual = pid->Actual_filter(pid->Actual_filter_para, obj->Actual);
    }

    pid->Error_now = obj->Target - obj->Actual;

    /* 误差输入限幅，防止 15kHz 下积分快速饱和 */
    float err_i = pid->Error_now;
    if (pid->Er_limit > 0.0f) {
        if (err_i > pid->Er_limit)  err_i = pid->Er_limit;
        if (err_i < -pid->Er_limit) err_i = -pid->Er_limit;
    }
    pid->Error_calculus += err_i;
    if (pid->Er_weaken > 0 && pid->Er_weaken <= 1) {
        pid->Error_calculus *=  pid->Er_weaken;
    }
    pid->Error_output = pid->ki * pid->Error_calculus ;

    if (pid->Error_output > pid->ErMax &&  pid->ErMax != 0) {
        pid->Error_output = pid->ErMax;
    } else if (pid->Error_output < pid->ErMin &&  pid->ErMax != 0) {
        pid->Error_output = pid->ErMin;
    }


    float derivative = pid->Error_now - pid->Error_pass;

    if (pid->Derivative_filter != NULL && pid->Derivative_filter_para != NULL) {
        derivative = pid->Derivative_filter(pid->Derivative_filter_para, derivative);
    }

    obj->Out = pid->kp * pid->Error_now +
               pid->Error_output +
               pid->kd * derivative;

    if (obj->Out > pid->OutMax) {
        obj->Out = pid->OutMax;
    } else if (obj->Out < pid->OutMin) {
        obj->Out = pid->OutMin;
    }

    if (pid->Output_filter != NULL && pid->Output_filter_para != NULL) {
        obj->Out = pid->Output_filter(pid->Output_filter_para, obj->Out);
    }

    pid->Error_pass = pid->Error_now;
    return obj->Out;
}
float PID_Calculate_debug(control_PID *pid, control_object *obj, float *debug_data){
    obj->Target = obj->Target; // Target 通常由外部赋值

    if (pid->Actual_filter != NULL && pid->Actual_filter_para != NULL) {
        obj->Actual = pid->Actual_filter(pid->Actual_filter_para, obj->Actual);
    }

    pid->Error_now = obj->Target - obj->Actual;

    float err_i = pid->Error_now;
    if (pid->Er_limit > 0.0f) {
        if (err_i > pid->Er_limit)  err_i = pid->Er_limit;
        if (err_i < -pid->Er_limit) err_i = -pid->Er_limit;
    }
    pid->Error_calculus += err_i;
    if (pid->Er_weaken > 0 && pid->Er_weaken <= 1) {
        pid->Error_calculus *=  pid->Er_weaken;
    }
    pid->Error_output = pid->ki * pid->Error_calculus ;

    if (pid->Error_output > pid->ErMax &&  pid->ErMax != 0) {
        pid->Error_output = pid->ErMax;
    } else if (pid->Error_output < pid->ErMin &&  pid->ErMax != 0) {
        pid->Error_output = pid->ErMin;
    }


    float derivative = pid->Error_now - pid->Error_pass;

    if (pid->Derivative_filter != NULL && pid->Derivative_filter_para != NULL) {
        derivative = pid->Derivative_filter(pid->Derivative_filter_para, derivative);
    }
    debug_data[0] = pid->kp * pid->Error_now;
    debug_data[1] = pid->Error_output;
    debug_data[2] = pid->kd * derivative;

    obj->Out =debug_data[0] +
              debug_data[1] +
                  debug_data[2];

    if (obj->Out > pid->OutMax) {
        obj->Out = pid->OutMax;
    } else if (obj->Out < pid->OutMin) {
        obj->Out = pid->OutMin;
    }

    if (pid->Output_filter != NULL && pid->Output_filter_para != NULL) {
        obj->Out = pid->Output_filter(pid->Output_filter_para, obj->Out);
    }

    pid->Error_pass = pid->Error_now;

    return obj->Out;
}



float PID_Calculate_DerivativeOnMeasure(control_PID *pid, control_object *obj)
{
    if (pid->Actual_filter != NULL && pid->Actual_filter_para != NULL) {
        obj->Actual = pid->Actual_filter(pid->Actual_filter_para, obj->Actual);
    }

    pid->Error_now = obj->Target - obj->Actual;

    float err_i = pid->Error_now;
    if (pid->Er_limit > 0.0f) {
        if (err_i > pid->Er_limit)  err_i = pid->Er_limit;
        if (err_i < -pid->Er_limit) err_i = -pid->Er_limit;
    }
    pid->Error_calculus += err_i;

    if (pid->Error_calculus > pid->ErMax) {
        pid->Error_calculus = pid->ErMax;
    } else if (pid->Error_calculus < pid->ErMin) {
        pid->Error_calculus = pid->ErMin;
    }

    if (pid->Er_weaken > 0 && pid->Er_weaken <= 1) {
        pid->Error_calculus *=  pid->Er_weaken;
    }

    float actual_derivative = obj->Actual - obj->Actual_pass;

    if (pid->Derivative_filter != NULL && pid->Derivative_filter_para != NULL) {
        actual_derivative = pid->Derivative_filter(pid->Derivative_filter_para, actual_derivative);
    }

    obj->Out = pid->kp * pid->Error_now +
               pid->ki * pid->Error_calculus -
               pid->kd * actual_derivative;

    if (obj->Out > pid->OutMax) {
        obj->Out = pid->OutMax;
    } else if (obj->Out < pid->OutMin) {
        obj->Out = pid->OutMin;
    }

    if (pid->Output_filter != NULL && pid->Output_filter_para != NULL) {
        obj->Out = pid->Output_filter(pid->Output_filter_para, obj->Out);
    }

    pid->Error_pass = pid->Error_now;
    obj->Actual_pass = obj->Actual;

    return obj->Out;
}

#define  I_deadband      20
float PID_I_deadband_Calculate(control_PID *pid, control_object *obj){
    obj->Target = obj->Target; // Target 通常由外部赋值

    if (pid->Actual_filter != NULL && pid->Actual_filter_para != NULL) {
        obj->Actual = pid->Actual_filter(pid->Actual_filter_para, obj->Actual);
    }

    pid->Error_now = obj->Target - obj->Actual;

    float err_i = pid->Error_now;
    if (pid->Er_limit > 0.0f) {
        if (err_i > pid->Er_limit)  err_i = pid->Er_limit;
        if (err_i < -pid->Er_limit) err_i = -pid->Er_limit;
    }
    if (pid->Error_now > I_deadband || pid->Error_now < -I_deadband) {
        pid->Error_calculus += err_i;
    }

    if (pid->Error_calculus > pid->ErMax) {
        pid->Error_calculus = pid->ErMax;
    } else if (pid->Error_calculus < pid->ErMin) {
        pid->Error_calculus = pid->ErMin;
    }

    if (pid->Er_weaken > 0 && pid->Er_weaken <= 1) {
        pid->Error_calculus *=  pid->Er_weaken;
    }

    float derivative = pid->Error_now - pid->Error_pass;

    if (pid->Derivative_filter != NULL && pid->Derivative_filter_para != NULL) {
        derivative = pid->Derivative_filter(pid->Derivative_filter_para, derivative);
    }

    obj->Out = pid->kp * pid->Error_now +
               pid->ki * pid->Error_calculus +
               pid->kd * derivative;

    if (obj->Out > pid->OutMax) {
        obj->Out = pid->OutMax;
    } else if (obj->Out < pid->OutMin) {
        obj->Out = pid->OutMin;
    }

    if (pid->Output_filter != NULL && pid->Output_filter_para != NULL) {
        obj->Out = pid->Output_filter(pid->Output_filter_para, obj->Out);
    }

    pid->Error_pass = pid->Error_now;

    return obj->Out;
}

void PID_Reset(control_PID *pid) {
    pid->Error_now = 0;
    pid->Error_pass = 0;
    pid->Error_calculus = 0;
    pid->Error_output = 0;
}

void control_tim_start() {
    HAL_TIM_Base_Start(&htim5);
}

static uint32_t c_tim_count = 0;
uint32_t get_c_tim_count() {
    c_tim_count = TIM5->CNT;
    return c_tim_count;
}
uint32_t tim_dif_start_p[5];
uint32_t tim_dif[5];

uint32_t get_tim_dif(uint8_t i) {
    return tim_dif[i];
}
void tim_dif_start(uint8_t i) {
    tim_dif_start_p[i] = get_c_tim_count();
}
uint32_t tim_dif_stop(uint8_t i) {
    tim_dif[i] = get_c_tim_count() - tim_dif_start_p[i];
    return tim_dif[i];
}