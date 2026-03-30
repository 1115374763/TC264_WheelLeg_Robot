
#include "zf_common_headfile.h"
#include "math.h"


int16 momentumwheel_pwm = 0;
int16 motor_pwm = 0;
int16 balance_pwm = 0;
float Total_roll = 0.0;
int PWM_OUT = 0;
int PWM_OUT_sudu = 0;
float Expectroll = 0.0;//零点 四元素181.0 互补0.6
float balance_velocity_PID_Encoder = 0.0;



float Expectroll_T = 4;//-5
float BASE_p=25.00;
float Lp_Circle= 28.0;      //圆环转向环p
float Lp_Base=50.5;//基础转向环P




float BASE_p_L=26.00;
float BASE_p_R=26.00;
float BASE_p2=25.00;

//结构体 具体定义在pid.h
closed_loop_struct balance_velocity_pid;
closed_loop_struct balance_angle_p;
closed_loop_struct balance_gyro_pd;
closed_loop_struct balance_gyro_weizhi_pd;
closed_loop_struct balance_location_pid;
closed_loop_struct balance_encoder_pd;
closed_loop_struct balance_rollangle_pi;




extern float Err;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     转向环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
float balance_location_PID(float err)                //转向环 左转负右转正
{
    float location_banlance_error;
    float a = 0.00755; //0.0055
    float b = 30; //18
    float Gkd = 305.0;

    balance_location_pid.error = err;
    balance_location_pid.i_integral += balance_location_pid.error;

    if(SingleBridge_mode==1||hengduan_flag!=0)//
    {
        balance_location_pid.kp =20;
    }
    else
    {
        balance_location_pid.kp = Lp_Base;
    }



    location_banlance_error =  balance_location_pid.kp * balance_location_pid.error + balance_location_pid.kd*(balance_location_pid.error- balance_location_pid.last_error)
            -Gkd * QEKF_INS.Gyro[2];

//    if(circle_flag!=1&&Barrier_Flag!=1&&Zebra_Flag!=1&&Ramp_Flag!=1&&SingleBridge_Flag!=1&&Ramp_End_Flag!=1&&Jumpbarrier_Flag!=1)//&&Cross_Flag!=1
//    {}
        if(location_banlance_error>=600) location_banlance_error = 700;
        else if(location_banlance_error<=-600) location_banlance_error = -700;


    balance_location_pid.last_error = balance_location_pid.error;

    return location_banlance_error;
}

void balance_location_pid_init(void)
{
    balance_location_pid.kp = 0.0f;//20.5
    balance_location_pid.ki = 0.0f;
    balance_location_pid.kd = 0.3f;//0.1
    balance_location_pid.i_integral = 0;
    balance_location_pid.last_error = 0;
    balance_location_pid.next_error = 0.f;
    balance_location_pid.error = 0;
}


//        else
//        {
//            balance_location_pid.kp = Lp_Base;
//        }
         //        else if(Check_right_flag==1)
         //        {balance_location_pid.kp = BASE_p_R ;}
         //        else if(Check_left_flag==1)
         //        {balance_location_pid.kp = BASE_p_L ;}
//        else //if (Err<0)左负
//        {
//            balance_location_pid.kp = BASE_p ;
//        //balance_location_pid.kp = a * pow(balance_location_pid.error,2) + b; //P值与差值成二次函数关系//
//        }
//        else if (Err>=0)
//        {
//            balance_location_pid.kp = BASE_p2 ;
//        //balance_location_pid.kp = a * pow(balance_location_pid.error,2) + b; //P值与差值成二次函数关系//
//        }
         //        if(Cross_Flag==1)
         //        {
         //            balance_location_pid.kp = 10;
         //        }
         //        if(circle_flag==1)
         //        {
         ////          balance_location_pid.kp = Lp_Circle;
         //            balance_location_pid.kp = 38.0;
         //        }
         //        else if(Ramp_Flag==1)
         //        {
         //            balance_location_pid.kp = Lp_Ramp;
         //        }
         //        else if(Ramp_End_Flag==1)
         //        {
         //            balance_location_pid.kp = Lp_Ramp_End;
         //        }
/*-------------------------------------------------------------------------------------------------------------------
  @brief     速度环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
float balance_velocity_PID(float encoder, float exspeed)                              //速度环
{

    balance_velocity_PID_Encoder = exspeed - encoder;

    balance_velocity_pid.error = exspeed - encoder;
    balance_velocity_pid.i_integral += balance_velocity_pid.error;

    if(balance_velocity_pid.i_integral <-5)      balance_velocity_pid.i_integral= -5; //限幅
    else if(balance_velocity_pid.i_integral >5)  balance_velocity_pid.i_integral= 5 ; //限幅

    float vertical_banlance_error =  (balance_velocity_pid.kp * balance_velocity_pid.error + balance_velocity_pid.ki * (balance_velocity_pid.i_integral) + balance_velocity_pid.kd * + (balance_velocity_pid.error - balance_velocity_pid.last_error));
    balance_velocity_pid.last_error = balance_velocity_pid.error;

    return vertical_banlance_error;
}


void balance_velocity_pid_init(void)
{
    balance_velocity_pid.kp = 0.05f;//3.8
    balance_velocity_pid.ki = 0.0f;//0.42
    balance_velocity_pid.kd = 0.0f;//2.0
    balance_velocity_pid.i_integral = 0.f;
    balance_velocity_pid.last_error = 0.f;
    balance_velocity_pid.next_error = 0.f;
    balance_velocity_pid.error = 0.f;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     角度环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
float balance_angle_P(float realangle,float Expectangle)      //角度环
{
    float k = 0.0;
    float angle_banlance_error;
    balance_angle_p.error = realangle - Expectangle ;
    balance_angle_p.i_integral += balance_angle_p.error;

    if(balance_angle_p.i_integral <-100)      balance_angle_p.i_integral= -100; //限幅
    else if(balance_angle_p.i_integral > 100)  balance_angle_p.i_integral= 100 ; //限幅


    angle_banlance_error =  balance_angle_p.kp * balance_angle_p.error + balance_angle_p.ki*balance_angle_p.i_integral+balance_angle_p.kd*(balance_angle_p.error- balance_angle_p.last_error);
    balance_angle_p.last_error = balance_angle_p.error;

//     if(hengduan_flag!=0)
//    {
//         balance_angle_p.kp = 30;//50
//         balance_angle_p.kd = 15;//0.01
//    }
//     else if(SingleBridge_mode==1)
//     {
//         balance_angle_p.kp = 40;//50
//         balance_angle_p.kd = 25;//0.01
//     }
//     else{
//         balance_angle_p.kp = 50;//50
//         balance_angle_p.ki = 0;//
//         balance_angle_p.kd = 30;//0.01
//     }
    return angle_banlance_error;
}

void balance_angle_p_init(void)
{
    balance_angle_p.kp = 45.0f;//50
    balance_angle_p.ki = 0.0f;//
    balance_angle_p.kd = 15.0f;//0.01
    balance_angle_p.i_integral = 0.f;
    balance_angle_p.last_error = 0.f;
    balance_angle_p.next_error = 0.f;
    balance_angle_p.error = 0.f;
}

/*--------------
 * -----------------------------------------------------------------------------------------------------
  @brief     角速度环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
void balance_gyro_weizhi_PD(float gyro,float angle_banlance_error)           //角速度环 位置式
{
    int PWM_compensation = 0;

//*向右偏是负的 向左为正
    balance_gyro_weizhi_pd.error = angle_banlance_error - gyro;

    balance_pwm = balance_gyro_weizhi_pd.kp * (balance_gyro_weizhi_pd.error) + balance_gyro_weizhi_pd.kd * (balance_gyro_weizhi_pd.error - balance_gyro_weizhi_pd.last_error);
    balance_gyro_weizhi_pd.last_error = balance_gyro_weizhi_pd.error;

    momentumwheel_pwm_control(balance_pwm);
//    momentumwheel_pwm_control(2000);

}

void balance_gyro_weizhi_pd_init(void)
{
    balance_gyro_weizhi_pd.kp = 15.0f;//1250
    balance_gyro_weizhi_pd.ki = 0.0f;//
    balance_gyro_weizhi_pd.kd = 100.5f;//100
    balance_gyro_weizhi_pd.i_integral = 0.f;
    balance_gyro_weizhi_pd.last_error = 0.f;
    balance_gyro_weizhi_pd.next_error = 0.f;
    balance_gyro_weizhi_pd.error = 0.f;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     速度环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
void balance_encoder_PD(float encoder,float exspeed)
{
//*向右偏是负的 向左为正
    balance_encoder_pd.error = exspeed - encoder;
//    if(stop_flag == 1)
//    {
// //       balance_encoder_pd.kp = 0.08;
//    }
//    else if(Ramp_Flag == 1)
//    {
//        balance_encoder_pd.kp = 0.02;
//    }
//    else if(Ramp_End_Flag == 1)
//    {
//        balance_encoder_pd.kp = 0.04;
//    }
//    else if(Cross_Flag == 1)
//    {
//        balance_encoder_pd.kp = 0.1;
//    }
//
//    else if(SingleBridge_mode == 1)
//   {
//        balance_encoder_pd.kp = 0.040;//15
//
//   }
//    else if(hengduan_flag == 2)
//    {
//        balance_encoder_pd.kp = 0.045;
//
//    }
//     else  if(Jumpbarrier_mode == 1)
//    {
//        balance_encoder_pd.kp = 0.045;
//
//    }
//
//    if(Check_straight_flag==1)
//    {
//        balance_encoder_pd.kp = 0.043;
//
//    }
//    else
//    {
//        balance_encoder_pd.kp = 0.040;
//
//    }
//    balance_encoder_pd.kp = 0.03;

    motor_pwm = balance_encoder_pd.kp * balance_encoder_pd.error + balance_encoder_pd.kd * (balance_encoder_pd.error - balance_encoder_pd.last_error);//shao


    balance_encoder_pd.last_error = balance_encoder_pd.error;

}


void balance_encoder_pd_init(void)
{
    balance_encoder_pd.kp = 0.12f;//0.1/
    balance_encoder_pd.ki = 0.00f;//20
    balance_encoder_pd.kd = 0.0006f;//
    balance_encoder_pd.i_integral = 0.f;
    balance_encoder_pd.last_error = 0.f;
    balance_encoder_pd.next_error = 0.f;
    balance_encoder_pd.error = 0.f;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief    角速度环
  @note     增量式
-------------------------------------------------------------------------------------------------------------------*/
void balance_gyro_PD(float gyro,float angle_banlance_error)           //角速度环 增量式
{
    int PWM_compensation = 0;
//*向右偏是负的 向左为正
    balance_gyro_pd.error = angle_banlance_error - gyro;

    momentumwheel_pwm = balance_gyro_pd.kp * (balance_gyro_pd.error - balance_gyro_pd.last_error) + balance_gyro_pd.ki*balance_gyro_pd.error + balance_gyro_pd.kd*(balance_gyro_pd.error-2*balance_gyro_pd.last_error+balance_gyro_pd.next_error);
    PWM_OUT+=momentumwheel_pwm;

    balance_gyro_pd.next_error = balance_gyro_pd.last_error;
    balance_gyro_pd.last_error = balance_gyro_pd.error;


        momentumwheel_pwm_control_f(-PWM_OUT + PWM_OUT_sudu);
        momentumwheel_pwm_control_a(+PWM_OUT + PWM_OUT_sudu);
}

void balance_gyro_pd_init(void)
{
    balance_gyro_pd.kp = 2000.0f;//1800
    balance_gyro_pd.ki = 75.8f;//93
    balance_gyro_pd.kd = 1000.0f;//0.15
    balance_gyro_pd.i_integral = 0.f;
    balance_gyro_pd.last_error = 0.f;
    balance_gyro_pd.next_error = 0.f;
    balance_gyro_pd.error = 0.f;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief    roll轴补偿环
  @note     增量式
-------------------------------------------------------------------------------------------------------------------*/
float balance_rollangle_PI(float roll,float expectroll)           //roll轴补偿环 增量式
{
    float compensate_roll;
    balance_rollangle_pi.error = expectroll - roll;



    Total_roll = balance_rollangle_pi.kp * (balance_rollangle_pi.error) + balance_rollangle_pi.kd * (balance_rollangle_pi.error - balance_rollangle_pi.last_error);

    balance_rollangle_pi.next_error = balance_rollangle_pi.last_error;
    balance_rollangle_pi.last_error = balance_rollangle_pi.error;

    return Total_roll;
}


void balance_rollangle_pi_init(void)
{
    balance_rollangle_pi.kp = 2.0f;//
    balance_rollangle_pi.ki = 0.0f;//
    balance_rollangle_pi.kd = 5.0f;//
    balance_rollangle_pi.i_integral = 0.f;
    balance_rollangle_pi.last_error = 0.f;
    balance_rollangle_pi.next_error = 0.f;
    balance_rollangle_pi.error = 0.f;
}




