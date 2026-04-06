
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
    float angle_banlance_error;
    balance_angle_p.error = realangle - Expectangle ;
    balance_angle_p.i_integral += balance_angle_p.error;

    if(balance_angle_p.i_integral <-100)      balance_angle_p.i_integral= -100; //限幅
    else if(balance_angle_p.i_integral > 100)  balance_angle_p.i_integral= 100 ; //限幅

    // =========================================================
    // 【修复稳定性】根据重心高度动态调度 PID 参数
    // =========================================================
    float run_kp = balance_angle_p.kp;
    float run_kd = balance_angle_p.kd;

    if (hengduan_flag != 0) 
    {
        // 跳跃和腾空时的硬度
        run_kp = 30.0f;
        run_kd = 15.0f;
    }
    else if (SingleBridge_mode == 1) 
    {
        // 单边桥模式：重心高达 80+，需要更小/更柔和的 P，以及更大的 D 来抑制震荡
        // (注：这里的参数你可能需要微调，通常重心变高，P要稍微减小，D要增加)
        run_kp = balance_angle_p.kp * 0.8f; // 在你调好的基础 P 上打 8 折
        run_kd = balance_angle_p.kd * 1.2f; // 在你调好的基础 D 上增加 1.2 倍
    }

    // 使用动态计算出来的 run_kp 和 run_kd 进行运算
    angle_banlance_error =  run_kp * balance_angle_p.error 
                          + balance_angle_p.ki * balance_angle_p.i_integral
                          + run_kd * (balance_angle_p.error - balance_angle_p.last_error);
                          
    balance_angle_p.last_error = balance_angle_p.error;

    return angle_banlance_error;
}

void balance_angle_p_init(void)
{
    balance_angle_p.kp = 45.0f;//50
    balance_angle_p.ki = 0.0f;//
    balance_angle_p.kd = 25.0f;//0.01
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
/*-------------------------------------------------------------------------------------------------------------------
  @brief     速度环/位置环
  @note      位置式
-------------------------------------------------------------------------------------------------------------------*/
void balance_encoder_PD(float encoder, float exspeed)
{
    // 计算当前速度偏差
    balance_encoder_pd.error = exspeed - encoder;

    // 积分项累加（记录被推走的位移）
    balance_encoder_pd.i_integral += balance_encoder_pd.error;

    // 积分限幅
    if(balance_encoder_pd.i_integral > 5000)       balance_encoder_pd.i_integral = 5000;
    else if(balance_encoder_pd.i_integral < -5000) balance_encoder_pd.i_integral = -5000;

    // --- 算出原始的 motor_pwm 输出 ---
    motor_pwm = balance_encoder_pd.kp * balance_encoder_pd.error 
              + balance_encoder_pd.ki * balance_encoder_pd.i_integral   
              + balance_encoder_pd.kd * (balance_encoder_pd.error - balance_encoder_pd.last_error);

    // =========================================================
    // 【核心位置】在这里加入落地刹车/削峰逻辑
    // 拦截速度环的异常爆发，防止狂奔
    // =========================================================
    if (hengduan_flag == 3 || hengduan_flag == 4) 
    {
        // 落地瞬间（阶段3和阶段4），将电机的最大输出 PWM 强行削减
        // (注: 3000 这个值你可以根据实际电机的硬度适当调大或调小)
        if(motor_pwm > 3000)       motor_pwm = 3000; 
        else if(motor_pwm < -3000) motor_pwm = -3000;
    }

    balance_encoder_pd.last_error = balance_encoder_pd.error;
}


void balance_encoder_pd_init(void)
{
    balance_encoder_pd.kp = 0.12f;//0.1/
    balance_encoder_pd.ki = 0.002f;//20
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




