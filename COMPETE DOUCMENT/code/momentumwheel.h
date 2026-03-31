

#ifndef CODE_MOMENTUMWHEEL_H_
#define CODE_MOMENTUMWHEEL_H_

#include "zf_common_headfile.h"

void momentumwheel_init(void);
void momentumwheel_pwm_control_f(int speed);
void momentumwheel_pwm_control_a(int speed);
void momentumwheel_pwm_control(int speed);
extern float momentumwheel_encoder_mid;
extern float momentumwheel_encoder_chasu;
extern int16 momentumwheel_encoder_forward;
extern int16 momentumwheel_encoder_after;

#endif /* CODE_MOMENTUMWHEEL_H_ */
