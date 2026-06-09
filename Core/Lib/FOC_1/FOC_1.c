//
// Created by zhang on 2026/3/18.
//
#include "MT6816.h"
#include "tim.h"
#include "FOC_1.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#include "cmsis_os2.h"
#include "DRV8313.h"
#include "OLED.h"
#include "Control.h"
#define sqrt_3  0.8660254f
#define voltage_Q15_MAX_limit  32767
#define voltage_Q15_MIN_limit  (-32768)
#define Q16                    65536
#define Q16_D_115              56987                     //Q16/1.15
#define Q16_0d15               9830  // 65536 *0.15

#define PWM_Period          699

#define FOC_U_SET_PWM(CCR) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, CCR)
#define FOC_V_SET_PWM(CCR) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, CCR)
#define FOC_W_SET_PWM(CCR) __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CCR)






//Hardware

#define drive_Init      DRV8313_Init();
#define drive_enable    DRV8313_CHOOSE_ENABLE(1);
#define drive_disable   DRV8313_CHOOSE_ENABLE(0);

#define encoder_bit    16383



//电机特性
uint8_t pole_pairs= 7;  //电机极对视
uint8_t wire_state = 0;
uint16_t elect_angle_limit;
uint16_t elect_angle_zero;

# define  U_input    12.0f
# define  R_wire    7.5f
# define  Drive_limit_I     1.7f

const int16_t accelerate_U_CONST = (int16_t)(R_wire * Drive_limit_I * Q16 / U_input);

void FOC_max_accelerate() {
    Uq = accelerate_U_CONST ;
}


//变量定义
int16_t foc_test1;
uint16_t foc_test2;

int16_t FOC_Uq = 0,FOC_Ud = 0;
int16_t Uq = 0,Ud = 0 ;
int16_t U_Alpha,U_Beta;
uint16_t Ua ,Ub ,Uc;
uint16_t* Ua_p = &Ua;
uint16_t* Ub_p = &Ub;
uint16_t* Uc_p = &Uc;

int16_t elect_angle;                //0 ~ 360* pole_pairs
uint8_t FOC_state = 0;

float FOC_speed;
float speed_target;

int16_t FOC_location;
int16_t FOC_location_target = 4000;         //14位,与编码器匹配
int16_t FOC_rela_location_zero = 11595;
int16_t FOC_rela_location;
int8_t FOC_location_target_state = 0;
int16_t FOC_Max_location = 16383;
int16_t FOC_Min_location = 0;
void FOC_SET_location_limit(int16_t Min,int16_t Max);

//控制对象
control_object FOC_SPEED_O;
control_object FOC_LOCATION_O;
control_object FOC_direct_LOCATION_O;

//PID参数
control_PID FOC_SPEED_PID;
control_PID FOC_LOCATION_PID;
control_PID FOC_direct_LOCATION_0_PID;
control_PID FOC_direct_LOCATION_1_PID;

lowpass_Filter derivative_filter;

void FOC_PID_Param_Init(void) {
    Control_PID_Init(&FOC_SPEED_PID, 1200.0f, 20.0f, 0.0f, -32767.0f, 32767.0f, 10000.0f);
    Control_PID_Init(&FOC_LOCATION_PID, 0.02f, 0.000005f, 0.001, -4.0f, 4.0f, 15.0f);
    // Control_PID_Init(&FOC_LOCATION_PID, 0.00f, 0.00000f, 0.000, -4.0f, 4.0f, 15.0f);


    derivative_filter.alpha = 0.3f;
    derivative_filter.last_value = 0.0f;
    // PID_EnableActualFilter(&FOC_LOCATION_PID, lowpass_filter, &derivative_filter);
    // PID_EnableDerivativeFilter(&FOC_LOCATION_PID, lowpass_filter, &derivative_filter);
    // PID_EnableOutputFilter(&FOC_LOCATION_PID, lowpass_filter, &derivative_filter);


    Control_PID_Init(&FOC_direct_LOCATION_0_PID, 15.0f, 0.15f, 50.0f, -32767.0f, 32767.0f, 8000.0f);
    Control_PID_Init(&FOC_direct_LOCATION_1_PID, 45.0f, 0.4f, 100.0f, -32767.0f, 32767.0f, 8000.0f);
    PID_EnableActualFilter(&FOC_direct_LOCATION_0_PID, lowpass_filter, &derivative_filter);

}



//q15乘法
static inline int16_t mul_q15(int16_t a, int16_t b) {
    return (int16_t)( ( (int32_t)a * b ) >> 15 );
}
//三个数最小值
static inline uint16_t min_three(uint16_t a, uint16_t b, uint16_t c) {
    uint16_t min = a;
    if (b < min){ min = b;}
    if (c < min){ min = c;}
    return min;
}


//产生三角函数表
#define TABLE_SIZE       360     //表存的个数
#define ANGLE_RES        1       // 1°分辨率
#define Q15_SCALE        32767   // Q15定点数缩放因子
int16_t sin_table[TABLE_SIZE];
int16_t cos_table[TABLE_SIZE];
void FOC_Table_Init(void) {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float angle_rad = (float)i * ANGLE_RES * 3.1415926f / 180.0f;
        cos_table[i] = (int16_t)(cosf(angle_rad) * Q15_SCALE);
        sin_table[i] = (int16_t)(sinf(angle_rad) * Q15_SCALE);
    }
}


//限幅
#define constrain(value, min_val, max_val) \
((value) < (min_val) ? (min_val) : ((value) > (max_val) ? (max_val) : (value)))




//工具函数
void FOC_SET_volt(int16_t dc_Ua, int16_t dc_Ub, int16_t dc_Uc) {
    // 限幅处理
    // Ua = constrain(dc_Ua, 0, Q16_D_115);
    // Ub = constrain(dc_Ub, 0, Q16_D_115);
    // Uc = constrain(dc_Uc, 0, Q16_D_115);

    FOC_U_SET_PWM( (*Ua_p ) * PWM_Period/ Q16_D_115);
    FOC_V_SET_PWM( (*Ub_p ) * PWM_Period/ Q16_D_115);
    FOC_W_SET_PWM( (*Uc_p ) * PWM_Period/ Q16_D_115);

}


void FOC_Inv_Part_Clark_Q15(int16_t use_elect_angle) {
    int16_t cos_theta = cos_table[use_elect_angle];
    int16_t sin_theta = sin_table[use_elect_angle];
//逆park
    U_Alpha = (mul_q15(-Uq,cos_theta) - mul_q15(Ud,sin_theta));
    U_Beta =  (mul_q15(-Uq,sin_theta) + mul_q15(Ud,cos_theta));

    // U_Alpha = (mul_q15(Ud,cos_theta) + mul_q15(Uq,sin_theta));
    // U_Beta =  (mul_q15(Ud,sin_theta) - mul_q15(Uq,cos_theta));

//逆clark
    static const int16_t sqrt3_q15 = (int16_t)(0.8660254f * Q15_SCALE);
    static const int16_t half_q15 = Q15_SCALE / 2; // 预定义为16383

    Ua = U_Alpha+ voltage_Q15_MAX_limit;
    Ub = mul_q15((int16_t)(-U_Alpha), half_q15) + mul_q15(U_Beta, sqrt3_q15)+ voltage_Q15_MAX_limit;
    Uc = mul_q15((int16_t)(-U_Alpha), half_q15) - mul_q15(U_Beta, sqrt3_q15)+ voltage_Q15_MAX_limit;
}
void FOC_SVPWM_Calculate() {
    uint16_t U_min = min_three(Ua, Ub, Uc);
    foc_test2 = U_min;
    Ua -= U_min;
    Ub -= U_min;
    Uc -= U_min;
}



void FOC_move(int16_t use_elect_angle) {
    FOC_Inv_Part_Clark_Q15(use_elect_angle);
    FOC_SVPWM_Calculate();
    FOC_SET_volt(Ua,Ub,Uc);
}

void FOC_check_wire() {
    Uq = 4000;



}

void FOC_elect_angle_zero_calibration() {
    HAL_TIM_Base_Stop_IT(&htim3);
    Ud = 3000;
    Uq = 0;
    drive_enable;
    FOC_move(0);
    osDelay (1000);
    // HAL_Delay(1000);
    elect_angle_zero = MT6816_angle_14bit_refresh();
    elect_angle_zero = MT6816_angle_14bit_refresh();
    // HAL_Delay(50);
    drive_disable;
    Uq = 0;
    Ud = 0;
    FOC_SET_volt(0,0,0);
    HAL_TIM_Base_Start_IT(&htim3);
}
//无刷接线
uint16_t start_angle;
void FOC_chack_wire() {
    FOC_state = 111;  //接线确认状态
    start_angle = MT6816_angle_14bit_lowpass_refresh();
    start_angle = MT6816_angle_14bit_lowpass_refresh();

}



//调用控制
void FOC_Set_Uq(int16_t value) {
    Uq = value;
}

void FOC_Set_Ud(int16_t value) {
    Ud = value;
}

//电角度
void FOC_elect_angle_MT6816() {
    uint16_t temp_elect_angle = 0;
    if (FOC_location >= elect_angle_zero) {
        temp_elect_angle = FOC_location - elect_angle_zero;
    }
    else if (FOC_location < elect_angle_zero) {
        temp_elect_angle = FOC_location + 16384 - elect_angle_zero;
    }
    elect_angle = (int16_t)(temp_elect_angle *7*360/MT6816_limit)%360;
}

void FOC_elect_angle_location() {
    uint16_t temp_elect_angle = 0;
    if (FOC_location_target >= elect_angle_zero) {
        temp_elect_angle = FOC_location_target - elect_angle_zero;
    }
    else if (FOC_location_target < elect_angle_zero) {
        temp_elect_angle = FOC_location_target + 16384 - elect_angle_zero;
    }
    elect_angle = (int16_t)(temp_elect_angle *7*360/MT6816_limit)%360;
}


//功能
void FOC_Init(void){
    FOC_PID_Param_Init();   //PID参数初始化
    elect_angle_zero=  11101;

    // 设置位置限制
    // uint16_t location_extend = 1200;
    // FOC_SET_location_limit(FOC_rela_location_zero-location_extend,
    //                     FOC_rela_location_zero+location_extend);

    FOC_Table_Init();  //生成三角函数表
    // drive_Init;
    MT6816_Init();

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);

    FOC_elect_angle_zero_calibration();
    FOC_SET_volt(0,0,0);

    FOC_chack_wire();// 检测接线
    //计得测定
//开启周期函数
    HAL_TIM_Base_Start_IT(&htim3);
}


//模拟电角度旋转
#define  interrupt_period        3000     //us
#define  pll            (interrupt_period/1000)
uint8_t analog_state = 0;

void FOC_elect_angle_analog(float rad_s) {   //rad/s
    static float elect_angle_analog;
    // if (analog_state == 0) {
    //     elect_angle_analog = elect_angle;
    //     analog_state = 1;
    // }
    elect_angle_analog += rad_s*(float)pole_pairs*360/15000;
    if (elect_angle_analog >= 360) {
        elect_angle_analog = 0;
    }
    if (elect_angle_analog < 0) {
        elect_angle_analog = 360;
    }
    elect_angle = (int16_t)elect_angle_analog;
}
// void analog_to_location(uint16_t location, float rad_s) {    //确保
//
// }


void FOC_speed_PID() {
    FOC_SPEED_O.Target = speed_target;
    FOC_SPEED_O.Actual = FOC_speed;
    float output = PID_Calculate(&FOC_SPEED_PID, &FOC_SPEED_O);
    Ud = 0;
    Uq = (int16_t)output;
}

#define err_location  10
void FOC_location_target_PID() {
    if (FOC_location_target > FOC_Max_location) {
        FOC_location_target = FOC_Max_location;
    }
    else if (FOC_location_target<FOC_Min_location) {
        FOC_location_target = FOC_Min_location;
    }

    int16_t position_error = FOC_location_target - FOC_location;

    if (position_error > 8192) {
        position_error -= 16384;
    }
    else if (position_error < -8192) {
        position_error += 16384;
    }

    FOC_LOCATION_O.Target = (float)(FOC_location + position_error);
    FOC_LOCATION_O.Actual = (float)FOC_location;

    float location_speed_target = PID_Calculate(&FOC_LOCATION_PID, &FOC_LOCATION_O);

    //test

    FOC_SPEED_O.Target = location_speed_target;
    FOC_SPEED_O.Actual = FOC_speed;
    float output = PID_Calculate(&FOC_SPEED_PID, &FOC_SPEED_O);
    Uq = (int16_t)output;
}

#define location_tipping_point_1     200
void FOC_direct_location_PID() {
    if (FOC_location_target > FOC_Max_location) {
        FOC_location_target = FOC_Max_location;
    }
    else if (FOC_location_target<FOC_Min_location) {
        FOC_location_target = FOC_Min_location;
    }

    FOC_LOCATION_O.Target = (float)FOC_location_target;//
    FOC_LOCATION_O.Actual = (float)FOC_location;

    float output = 0;
    float temp_dif_T_A = FOC_LOCATION_O.Target-FOC_LOCATION_O.Actual;
    if (temp_dif_T_A < location_tipping_point_1 && temp_dif_T_A>-location_tipping_point_1) {
        output = PID_I_deadband_Calculate(&FOC_direct_LOCATION_0_PID, &FOC_LOCATION_O);
    }
    else {
        output = PID_I_deadband_Calculate(&FOC_direct_LOCATION_1_PID, &FOC_LOCATION_O);
    }
    Uq = (int16_t)output;

}

void FOC_period() {
    //状态检测
    if (FOC_state != 0) {
        drive_enable;
    }
    else {
        drive_disable;
        FOC_location_target_state = 0;
        FOC_SET_volt(0,0,0);
    }

    // if (FOC_state != 1) {
    //     analog_state = 0;
    // }
    static uint8_t Per_Count;
    Per_Count++;

    //角度
    // FOC_location = MT6816_angle_14bit_lowpass_refresh();
    FOC_location = MT6816_angle_14bit_refresh();
    MT6816_Get_Angle();
    FOC_elect_angle_MT6816();


    //速度环 位置环
    if (Per_Count >= 30) {
        Per_Count = 0;
        FOC_speed = MT6816_Speed_lowpass();
        //PID速度环
        if (FOC_state == 3) {
            FOC_speed_PID();
        }
        if (FOC_state == 4 ) {
            FOC_location_target_PID();
            if (FOC_location_target - FOC_location <err_location && FOC_location_target - FOC_location >-err_location) {
                FOC_location_target_state = 1;
            }
        }
        if (FOC_state == 5) {
            FOC_direct_location_PID();
            if (FOC_location_target - FOC_location <err_location && FOC_location_target - FOC_location >-err_location) {
                FOC_location_target_state = 1;
            }
        }

    }
    if (FOC_location > FOC_Max_location) {
        Uq = 0;
        Ud = 0;
        FOC_Uq = 0;
        FOC_Ud = 0;
    }
    else if (FOC_location<FOC_Min_location) {
        Uq = 0;
        Ud = 0;
        FOC_Uq = 0;
        FOC_Ud = 0;
    }

    //模拟电角度旋转
    if (FOC_state == 1) {
        FOC_elect_angle_analog(speed_target);
        FOC_move(elect_angle);
        Ud = 3000;
    }
    else if (FOC_state == 2) {
        Uq = FOC_Uq;
        FOC_elect_angle_MT6816();
        FOC_move(elect_angle);
    }
    else if (FOC_state == 3|| FOC_state == 4|| FOC_state == 5) {
        Ud = 0;
        FOC_elect_angle_MT6816();
        FOC_move(elect_angle);
    }
    
    
    else if (FOC_state == 111) {
        static uint16_t count_check =0;
        count_check++;
        Uq = 4000;
        Ud = 0;
        FOC_elect_angle_MT6816();
        FOC_move(elect_angle);
        if (FOC_location - start_angle >500 && FOC_location - start_angle<3000) {
            FOC_state = 0;
        }
        if (count_check >= 5000) {
            Ub_p = &Uc;
            Uc_p = &Ub;
            FOC_state = 112;    //待校验零偏

            count_check = 0;
        }
        else if (FOC_state == 113) {
            Uq = 0;
            Ud = 0;
        }
    }
}
void FOC_SET_location_limit(int16_t Min,int16_t Max) {
    FOC_Max_location =  Max>Min ? Max:Min ;
    FOC_Min_location =  Max<Min ? Max:Min;
}
void FOC_SET_location_target(int16_t location) {
    FOC_location_target = location;
}
int16_t FOC_GET_location_target() {
    return FOC_location_target;
}

void FOC_SET_rela_location(int16_t location) {
    if(location + FOC_rela_location_zero < encoder_bit) {
        FOC_location_target = location + FOC_rela_location_zero;
    }
    else {
        FOC_location_target = location + FOC_rela_location_zero - encoder_bit;
    }
}



void FOC_SET_state(uint8_t state) {
    FOC_state = state;
}
uint8_t FOC_GET_state() {
    return FOC_state;
}
uint16_t FOC_GET_elect_angle() {
    return elect_angle;
}


void FOC_SET_speed_target(float analog_value) {
    speed_target = analog_value;
}
float FOC_GET_speed_target() {
    return speed_target;
}

void FOC_relative_location(uint16_t re_location) {
    if ( FOC_location + re_location>encoder_bit) {
        FOC_location_target = FOC_location + re_location -encoder_bit;
    }
    else if ( FOC_location + re_location<0) {
        FOC_location_target = FOC_location +encoder_bit+ re_location ;
    }
    else {
        FOC_location_target = FOC_location - re_location;
    }
}

void FOC_error_handle() {
    if (FOC_state == 112) {
        FOC_elect_angle_zero_calibration();
        OLED_ShowNum(1,1,elect_angle_zero,5);
        FOC_state = 0;
    }
    if (FOC_state == 113) {
        Uq = 0;
        Ud = 0;
        FOC_rela_location_zero = MT6816_GetDate();
        FOC_state = 0;
    }
}








//测试
int16_t foc_test(int16_t a) {
    return a;
}

