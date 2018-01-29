#ifndef __pwm_count_H__
#define __pwm_count_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "main_config.h"

#define PWM_Input_CH0_ENABLE 1    // if use this channel to capture PWM  PC0
#define PWM_Input_CH1_ENABLE 0    // if use this channel to capture PWM  PC1
#define PWM_Input_CH2_ENABLE 0    // if use this channel to capture PWM  PC2
#define PWM_Input_CH3_ENABLE 0    // if use this channel to capture PWM  PC3
#define PWM_Input_CH4_ENABLE 0    // if use this channel to capture PWM  PC4


void HF_PwmCount_Init(uint8_t pwmin_channel);
float HF_Get_PWMCount_Value(uint8_t channel_x); //current input value of PWM (Unit:us)

#ifdef __cplusplus
}
#endif

#endif //__pwm_in_H__


