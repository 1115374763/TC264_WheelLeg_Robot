
#include "zf_common_headfile.h"
#include "math.h"

/*-------------------------------------------------------------------------------------------------------------------
  @brief     pid≥ı ºªØ
  @note
-------------------------------------------------------------------------------------------------------------------*/
void pid_all_init(void)
{
    balance_angle_p_init();
    balance_velocity_pid_init();
    balance_gyro_pd_init();
    balance_gyro_weizhi_pd_init();
    balance_location_pid_init();
    camera_servo_pid_init();
    balance_encoder_pd_init();
    balance_servo_pid_init();
    balance_rollangle_pi_init();
}


