
#include "zf_common_headfile.h"
#include "math.h"

float momentumwheel_encoder_mid = 0.0;
float momentumwheel_encoder_chasu = 0.0;
int16 momentumwheel_encoder_forward = 0;
int16 momentumwheel_encoder_after = 0;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     传统电机初始化
  @note
-------------------------------------------------------------------------------------------------------------------*/
void momentumwheel_init(void)
{
    pwm_init(ATOM0_CH3_P21_5, 500, 0);//正
    gpio_init(P21_4, GPO, 0, GPO_PUSH_PULL);
    encoder_quad_new_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7);

    pwm_init(ATOM2_CH2_P33_6, 500, 0);//正
    gpio_init(P33_7, GPO, 1, GPO_PUSH_PULL);
    encoder_quad_new_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     传统电机输出
  @note
-------------------------------------------------------------------------------------------------------------------*/
void momentumwheel_pwm_control_f(int speed)
{
    if(speed>=9999) speed=9999;
    else if(speed<=-9999)   speed=-9999;
    if(speed<=0)
    {
        speed=-speed;
        pwm_set_duty(ATOM0_CH3_P21_5, speed);
        gpio_set_level(P21_4, 1);
    }
    else
     {
        pwm_set_duty(ATOM0_CH3_P21_5, speed);
        gpio_set_level(P21_4, 0);
     }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     传统电机输出
  @note
-------------------------------------------------------------------------------------------------------------------*/
void momentumwheel_pwm_control_a(int speed)
{
    if(speed>=9999) speed=9999;
    else if(speed<=-9999)   speed=-9999;
    if(speed<=0)
    {
        speed=-speed;
        pwm_set_duty(ATOM2_CH2_P33_6, speed);
        gpio_set_level(P33_7, 0);
    }
    else
     {
        pwm_set_duty(ATOM2_CH2_P33_6, speed);
        gpio_set_level(P33_7, 1);
     }
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     无刷控制
  @note
-------------------------------------------------------------------------------------------------------------------*/
void momentumwheel_pwm_control(int speed)
{

    if(speed>=6999) speed=6999;
    else if(speed<=-6999)   speed=-6999;
//    small_driver_set_duty(-speed+lora3a22_uart_transfer.joystick[0]/10+Err*2.0, speed+lora3a22_uart_transfer.joystick[0]/10+Err*2.0);   // 计算占空比输出
// if(zhonxianpiancha>0)//车身向左
// {
//     small_driver_set_duty(-speed-location_banlance_error+SingleBridge_err, speed+SingleBridge_err);
// }
// else if(zhonxianpiancha<=0)   //车身向右
// {
//     small_driver_set_duty(-speed+SingleBridge_err, speed-location_banlance_error+SingleBridge_err);
// }



    small_driver_set_duty(-speed+0+location_banlance_error+0, speed+location_banlance_error+0);//location_banlance_error也是左负右正，speed作为一个维持平衡常量



//    small_driver_set_duty(-speed, speed);
 //  small_driver_set_duty(-1000, 1000);//左轮数字越小就是减速，右轮数字越小就是减速
}
