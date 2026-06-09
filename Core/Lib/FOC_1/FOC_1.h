//
// Created by zhang on 2026/3/18.
//

#ifndef BLDC_1_FOC_1_H
#define BLDC_1_FOC_1_H

#include "Control.h"

extern control_object FOC_SPEED_O;
extern control_object FOC_LOCATION_O;
extern control_object FOC_direct_LOCATION_O;

extern control_PID FOC_SPEED_PID;
extern control_PID FOC_LOCATION_PID;
extern control_PID FOC_direct_LOCATION_0_PID;
extern control_PID FOC_direct_LOCATION_1_PID;

extern int16_t foc_test1;
extern uint16_t foc_test2;

extern int16_t FOC_Uq,FOC_Ud ;
extern int16_t Uq,Ud ;
extern int16_t U_Alpha,U_Beta;
extern uint16_t Ua ,Ub ,Uc;
extern int16_t elect_angle;

extern uint8_t FOC_state;
extern int16_t FOC_location;
extern  int16_t FOC_location_target;  //test
extern int16_t FOC_rela_location;
extern int8_t FOC_location_state;

void FOC_elect_angle_MT6816();

void FOC_Init(void);
void FOC_Set_Uq(int16_t value);
void FOC_Set_Ud(int16_t value);

int16_t foc_test(int16_t a);

void FOC_Inv_Part_Clark_Q15(int16_t elect_angle);

void FOC_SET_rela_location(int16_t location);
void FOC_SET_location_target(int16_t location);
int16_t FOC_GET_location_target();

void FOC_SET_speed_target(float analog_value);
float FOC_GET_speed_target();

void FOC_elect_angle_analog(float rad_s);
void FOC_period();
void FOC_SET_state(uint8_t state);
uint8_t FOC_GET_state();
uint16_t FOC_GET_elect_angle();


void FOC_error_handle();

//test
extern uint16_t elect_angle_zero;



#endif //BLDC_1_FOC_1_H