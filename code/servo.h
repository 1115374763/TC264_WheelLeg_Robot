#ifndef SERVO_H_
#define SERVO_H_

#include "zf_common_headfile.h"


#define SERVO_MID 700 //4620
#define SERVO_LEFT 800 //935
#define SERVO_RIGHT 600 //635

#define LEFT_MAX     (SERVO_LEFT- SERVO_MID)//+
#define RIGHT_MAX    (SERVO_RIGHT-SERVO_MID)//-

#define speed_min  (500 - 150 )
#define speed_max  (500 + 150 )

void Servo_init();
float camera_servo_PID(float expect_val,float err);
void camera_servo_pid_init(void);
float balance_servo_PID(float err,float expect_val);
void balance_servo_pid_init(void);
void change_zero(float err);
extern int Steer_Angle;



#endif
