
#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_
#include "zf_common_headfile.h"

void balance_encoder_PD(float encoder,float exspeed);
void balance_encoder_pd_init(void);
float balance_location_PID(float err);
void balance_location_pid_init(void);
float balance_velocity_PID(float encoder, float exspeed);
void balance_velocity_pid_init(void);
float balance_angle_P(float realangle,float Expectangle);
void balance_angle_p_init(void);
void balance_gyro_PD(float gyro,float angle_banlance_error);
void balance_gyro_pd_init(void);
void balance_gyro_weizhi_PD(float gyro,float angle_banlance_error);
void balance_gyro_weizhi_pd_init(void);
float balance_rollangle_PI(float roll,float expectroll)  ;


extern int16 momentumwheel_pwm;
extern int16 motor_pwm;
extern int16 balance_pwm;
extern float balance_velocity_PID_Encoder;
extern int PWM_OUT;
extern int PWM_OUT_sudu;
extern float Expectroll;
extern float Expectroll_T;


#endif /* CODE_BALANCE_H_ */
