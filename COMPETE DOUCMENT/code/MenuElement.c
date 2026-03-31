
#include "zf_common_headfile.h"
#include "math.h"

//菜单可调参数
float Max_speed = 900.0;     //最大速度
float Base_speed = 820.0;    //基础速度
float Circle_speed = 520.0;  //圆环速度
float Ramp_speed = 0.0;    //坡道速度
float Ramp_End_speed = 0.0;//坡道结束速度

float Qianzhan = 0.0;      //前瞻
float Base_speed_L = 480.0;    //基础速度
float Base_speed_R = 480.0;    //基础速度
float Lp_Cross = 10.0;      //十字转向环p
extern float Lp_Circle;      //圆环转向环p
float Lp_Ramp  = 0.0;      //坡道转向环p
float Lp_Ramp_End  = 0.0;  //坡道结束转向环p
extern float Lp_Base;

float Ap_Cross = 0.0;      //十字角度环p
float Ap_Circle_Mode1= 0.0;      //圆环一阶段角度环p
float Ap_Circle = 0.0;      //圆环角度环环p
float Ap_Ramp  = 0.0;      //坡道角度环p
float Ap_Base  = 0.0;      //基础角度环P

int Cross_enable = 1;
int Circle_enable = 1;
int Ramp_enable = 1;
int Zebra_enable = 1;
int Barrier_enable = 1;
int SingleBridge_enable = 1;
int Stopcar_enable = 1;
int Jumpbarrier_enable = 1;
int Ramp_sudu_en = 1;

int Time_1 = 0;
int Time_2 = 0;
int Time_Other = 0;

int LeftCircleLine_1 = 0;
int LeftCircleLine_4 = 0;
int RightCircleLine_1 = 0;
int RightCircleLine_4 = 0;

int tof_distance = 0;
int Mode_sudu = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     菜单参数初始化
  @note
-------------------------------------------------------------------------------------------------------------------*/
void MenuElementInit()
{
    if(Mode_sudu==0)
    {
        //速度参数
        Max_speed = 35.0;
        Base_speed = 26.0;
        Circle_speed = 26.0;
        Ramp_speed = 26.0;
        Ramp_End_speed = 24.0;

        //前瞻
        Qianzhan = 35.0;

        //转向环
        Lp_Cross = 0.23;
        Lp_Circle= 0.23;
        Lp_Ramp  = 0.15;
        Lp_Ramp_End = 0.23;
        Lp_Base  = 0.22;

        //角度环
        Ap_Cross = 0.065;
        Ap_Circle_Mode1= 0.07;
        Ap_Circle = 0.05;
        Ap_Ramp  = 0.08;
        Ap_Base  = 0.065;

        //元素使能标志位
        Cross_enable = 1;
        Circle_enable = 1;
        Ramp_enable = 1;
        Zebra_enable = 1;
        Barrier_enable = 1;
        Ramp_sudu_en = 0;

        //圆环计时
        Time_1 = 600;
        Time_2 = 600;
        Time_Other = 600;

        //圆环补线 负的靠左 正的靠右
        LeftCircleLine_1 = -5;
        LeftCircleLine_4 = -25;
        RightCircleLine_1 = 0;
        RightCircleLine_4 = 20;

        //tof识别距离
        tof_distance = 350;
    }

    if(Mode_sudu==1)
    {
        //速度参数
        Max_speed = 35.0;
        Base_speed = 25.0;
        Circle_speed = 26.0;
        Ramp_speed = 26.0;
        Ramp_End_speed = 24.0;

        //前瞻
        Qianzhan = 40.0;

        //转向环
        Lp_Cross = 0.23;
        Lp_Circle= 0.23;
        Lp_Ramp  = 0.15;
        Lp_Ramp_End = 0.23;
        Lp_Base  = 0.22;

        //角度环
        Ap_Cross = 0.065;
        Ap_Circle_Mode1= 0.07;
        Ap_Circle = 0.05;
        Ap_Ramp  = 0.08;
        Ap_Base  = 0.065;

        //元素使能标志位
        Cross_enable = 1;
        Circle_enable = 1;
        Ramp_enable = 1;
        Zebra_enable = 1;
        Barrier_enable = 1;
        Ramp_sudu_en = 1;

        //圆环计时
        Time_1 = 600;
        Time_2 = 600;
        Time_Other = 600;

        //圆环补线 负的靠左 正的靠右
        LeftCircleLine_1 = -5;
        LeftCircleLine_4 = -25;
        RightCircleLine_1 = 0;
        RightCircleLine_4 = 20;

        //tof识别距离
        tof_distance = 350;
    }

    if(Mode_sudu==2)
    {
        //速度参数
        Max_speed = 40.0;
        Base_speed = 27.0;
        Circle_speed = 26.0;
        Ramp_speed = 26.0;
        Ramp_End_speed = 24.0;

        //前瞻
        Qianzhan = 30.0;

        //转向环
        Lp_Cross = 0.23;
        Lp_Circle= 0.23;
        Lp_Ramp  = 0.15;
        Lp_Ramp_End = 0.23;
        Lp_Base  = 0.22;

        //角度环
        Ap_Cross = 0.065;
        Ap_Circle_Mode1= 0.07;
        Ap_Circle = 0.05;
        Ap_Ramp  = 0.08;
        Ap_Base  = 0.065;

        //元素使能标志位
        Cross_enable = 1;
        Circle_enable = 1;
        Ramp_enable = 1;
        Zebra_enable = 1;
        Barrier_enable = 1;
        Ramp_sudu_en = 0;

        //圆环计时
        Time_1 = 600;
        Time_2 = 600;
        Time_Other = 600;

        //圆环补线 负的靠左 正的靠右
        LeftCircleLine_1 = -5;
        LeftCircleLine_4 = -25;
        RightCircleLine_1 = 0;
        RightCircleLine_4 = 20;

        //tof识别距离
        tof_distance = 350;
    }

    if(Mode_sudu==3)
    {
        //速度参数
        Max_speed = 45.0;
        Base_speed = 27.0;
        Circle_speed = 26.0;
        Ramp_speed = 26.0;
        Ramp_End_speed = 24.0;

        //前瞻
        Qianzhan = 30.0;

        //转向环
        Lp_Cross = 0.23;
        Lp_Circle= 0.23;
        Lp_Ramp  = 0.15;
        Lp_Ramp_End = 0.23;
        Lp_Base  = 0.22;

        //角度环
        Ap_Cross = 0.065;
        Ap_Circle_Mode1= 0.07;
        Ap_Circle = 0.05;
        Ap_Ramp  = 0.08;
        Ap_Base  = 0.065;

        //元素使能标志位
        Cross_enable = 1;
        Circle_enable = 1;
        Ramp_enable = 1;
        Zebra_enable = 1;
        Barrier_enable = 1;
        Ramp_sudu_en = 0;

        //圆环计时
        Time_1 = 600;
        Time_2 = 600;
        Time_Other = 600;

        //圆环补线 负的靠左 正的靠右
        LeftCircleLine_1 = -5;
        LeftCircleLine_4 = -25;
        RightCircleLine_1 = 0;
        RightCircleLine_4 = 20;

        //tof识别距离
        tof_distance = 350;
    }
}
