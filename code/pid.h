

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_integral;
    float error;
    float last_error;
    float next_error;
}closed_loop_struct;

extern closed_loop_struct balance_servo_pid;
extern closed_loop_struct balance_encoder_pd;
extern closed_loop_struct balance_location_pid;
extern closed_loop_struct balance_velocity_pid;
extern closed_loop_struct balance_angle_p;
extern closed_loop_struct balance_gyro_pd;
extern closed_loop_struct balance_gyro_weizhi_pd;
extern closed_loop_struct balance_rollangle_pi;

void pid_all_init(void);

#endif /* CODE_PID_H_ */
