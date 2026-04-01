/*
 * vmc.h
 *
 *  Created on: 2024年12月3日
 *      Author: Administrator
 */

#ifndef CODE_VMC_H_
#define CODE_VMC_H_
#include "zf_common_headfile.h"

#define SERVO_MOTOR_PWM1             (ATOM0_CH6_P02_6)                           // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2             (ATOM0_CH5_P02_5)
#define SERVO_MOTOR_PWM3             (ATOM0_CH4_P02_4)
#define SERVO_MOTOR_PWM4             (ATOM0_CH7_P02_7)

#define SERVO_MOTOR_FREQ            (300 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define L1  60
#define L2  100
#define L3  100
#define L4  60
#define L5  37


typedef struct{
    float alphaLeft, betaLeft;
    float alphaRight, betaRight;
    float XLeft,YLeft;
    float XRight, YRight;
}IKparam;

void  SERVO_init();
void inverseKinematics();


extern IKparam IKParam;
extern uint8 jump_mode;
extern uint8 jump_cnt ;
extern float Stab_roll ;
extern float Ex_roll ;


#endif /* CODE_VMC_H_ */
