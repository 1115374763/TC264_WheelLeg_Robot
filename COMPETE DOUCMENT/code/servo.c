
#include "zf_common_headfile.h"
#include "math.h"

float zero_change = 0.0;
int Steer_Angle=0;

closed_loop_struct camera_servo_pid;
closed_loop_struct balance_servo_pid;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     舵机初始化
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Servo_init()
{
    pwm_init(ATOM1_CH1_P33_9, 50, 700);//   620   710   800
}
//300hz       3684   4620   5604





/*-------------------------------------------------------------------------------------------------------------------
  @brief     舵机转向环
  @note
-------------------------------------------------------------------------------------------------------------------*/
float camera_servo_PID(float expect_val,float err)//舵机PD调节
{
   float  x;

   float max_zero = 80.0;

   camera_servo_pid.error=err-expect_val;


   x = camera_servo_pid.kp*camera_servo_pid.error;

   camera_servo_pid.last_error=camera_servo_pid.error;

//   if(x>=max_zero)//限幅处理
//       x=max_zero;
//  else if(x<=-max_zero)
//      x=-max_zero;

   return -x;
}


void camera_servo_pid_init(void)
{
    camera_servo_pid.kp = 0.4f;//0.8
    camera_servo_pid.ki = 0.0f;
    camera_servo_pid.kd = 0.0f;
    camera_servo_pid.i_integral = 0.f;
    camera_servo_pid.last_error = 0.f;
    camera_servo_pid.next_error = 0.f;
    camera_servo_pid.error = 0.f;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     舵机平衡环
  @note
-------------------------------------------------------------------------------------------------------------------*/
float balance_servo_PID(float err,float expect_val)//舵机PD调节
{
   float  u;
   float max_angle = 80.0;

   balance_servo_pid.error=err-expect_val;

   balance_servo_pid.i_integral += balance_servo_pid.error;

   if(circle_flag==1)
   {
       balance_servo_pid.kp = 2.0;//12
   }
   else
   {
       balance_servo_pid.kp = 2.5;//12
   }

   u = balance_servo_pid.kp*balance_servo_pid.error + balance_servo_pid.ki * balance_servo_pid.i_integral + balance_servo_pid.kd*QEKF_INS.Gyro[0];

   if(u>=max_angle) u=max_angle;
   if(u<=-max_angle) u=-max_angle;

   return -u;
}

void balance_servo_pid_init(void)
{
    balance_servo_pid.kp = 1.5f;//22 180
    balance_servo_pid.ki = 0.0f;//0.1
    balance_servo_pid.kd = 5.0f;//-0.1 -10
    balance_servo_pid.i_integral = 0.f;
    balance_servo_pid.last_error = 0.f;
    balance_servo_pid.next_error = 0.f;
    balance_servo_pid.error = 0.f;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     舵机输出
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Steer(int angle)
{
//    if(angle>0) angle = angle + 2;
//    if(angle<0) angle = angle - 2;

    if(angle>=LEFT_MAX)//限幅处理
       angle=LEFT_MAX;
   else if(angle<=RIGHT_MAX)
        angle=RIGHT_MAX;
    pwm_set_duty(ATOM1_CH1_P33_9, SERVO_MID+angle);//舵机调节
    if( stop_flag==1 || Ramp_Flag==1) pwm_set_duty(ATOM1_CH1_P33_9, SERVO_MID);// || Ramp_End_Flag==1
}




/*-------------------------------------------------------------------------------------------------------------------
  @brief    动态零点
  @note
-------------------------------------------------------------------------------------------------------------------*/
void change_zero(float err)//动态零点
{
   float max_zero = 0.5;
   float  k = 0.01;

//   k = a * pow(err,2) + b; //P值与差值成二次函数关系//

   zero_change = err*k*1.0;

   if(zero_change>=max_zero)//限幅处理
       zero_change=max_zero;
  else if(zero_change<=-max_zero)
      zero_change=-max_zero;

   Expectroll = Expectroll_T - zero_change;
}

////使用的模糊PID程序，有几个特殊步骤，输入的参数E为err，EC为err的微分，即这次的err减去上次的err
//float KP_Fuzzy(float E,float EC)
//{
//
//    int rule_p[7][7]=
//    {
//        { 6 , 5 , 4 , 4 , 3 , 0 , 0},//-36
//        { 6 , 4 , 3 , 3 , 2 , 0 , 0},//-24
//        { 4 , 3 , 2 , 1 , 0 , 1 , 2},//-12
//        { 2 , 1 , 1 , 0 , 1 , 1 , 2},//0
//        { 2 , 1 , 0 , 1 , 2 , 3 , 4},//12
//        { 0 , 0 , 2 , 3 , 3 , 4 , 6},//24
//        { 0 , 1 , 3 , 4 , 4 , 5 , 6},//36
//    };//模糊规则表 P
//
//
//
//    uint8 i2;
//    /*输入量P语言值特征点*/
//    float EFF[7]={-45,-30,-15,0,15,30,45};
//    /*输入量D语言值特征点*/
//    float DFF[7]={-12,-8,-4,0,4,8,12};
//    /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
//    float UFF[7];
//
//    for(i2=0;i2<7;i2++)
//        UFF[i2]=kp_m/6*i2;
//
//
//    float U=0;  /*偏差,偏差微分以及输出值的精确量*/
//    float PF[2]={0},DF[2]={0},UF[4]={0};
//    /*偏差,偏差微分以及输出值的隶属度*/
//    int Pn=0,Dn=0,Un[4]={0};
//    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
//    /*隶属度的确定*/
//    /*根据PD的指定语言值获得有效隶属度*/
//    if(E>EFF[0] && E<EFF[6])
//    {
//        if(E<=EFF[1])
//        {
//            Pn=-2;
//            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
//        }
//        else if(E<=EFF[2])
//        {
//            Pn=-1;
//            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
//        }
//        else if(E<=EFF[3])
//        {
//            Pn=0;
//            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
//        }
//        else if(E<=EFF[4])
//        {
//            Pn=1;
//            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
//        }
//        else if(E<=EFF[5])
//        {
//            Pn=2;
//            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
//        }
//        else if(E<=EFF[6])
//        {
//            Pn=3;
//            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
//        }
//    }
//
//    else if(E<=EFF[0])
//    {
//        Pn=-2;/*  ??? */
//        PF[0]=1;
//    }
//    else if(E>=EFF[6])
//    {
//        Pn=3;
//        PF[0]=0;
//    }
//
//    PF[1]=1-PF[0];
//
//
//    //判断D的隶属度
//    if(EC>DFF[0]&&EC<DFF[6])
//    {
//        if(EC<=DFF[1])
//        {
//            Dn=-2;
//            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
//        }
//        else if(EC<=DFF[2])
//        {
//            Dn=-1;
//            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
//        }
//        else if(EC<=DFF[3])
//        {
//            Dn=0;
//            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
//        }
//        else if(EC<=DFF[4])
//        {
//            Dn=1;
//            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
//        }
//        else if(EC<=DFF[5])
//        {
//            Dn=2;
//            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
//        }
//        else if(EC<=DFF[6])
//        {
//            Dn=3;
//            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
//        }
//    }
//    //不在给定的区间内
//    else if (EC<=DFF[0])
//    {
//        Dn=-2;
//        DF[0]=1;
//    }
//    else if(EC>=DFF[6])
//    {
//        Dn=3;
//        DF[0]=0;
//    }
//
//    DF[1]=1-DF[0];
//
//    /*使用误差范围优化后的规则表rule[7][7]*/
//    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
//    /*一般都是四个规则有效*/
//    Un[0]=rule_p[Pn+2][Dn+2];
//    Un[1]=rule_p[Pn+3][Dn+2];
//    Un[2]=rule_p[Pn+2][Dn+3];
//    Un[3]=rule_p[Pn+3][Dn+3];
//
//    if(PF[0]<=DF[0])    //求小
//        UF[0]=PF[0];
//    else
//        UF[0]=DF[0];
//    if(PF[1]<=DF[0])
//        UF[1]=PF[1];
//    else
//        UF[1]=DF[0];
//    if(PF[0]<=DF[1])
//        UF[2]=PF[0];
//    else
//        UF[2]=DF[1];
//    if(PF[1]<=DF[1])
//        UF[3]=PF[1];
//    else
//        UF[3]=DF[1];
//    /*同隶属函数输出语言值求大*/
//    if(Un[0]==Un[1])
//    {
//        if(UF[0]>UF[1])
//            UF[1]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[2])
//    {
//        if(UF[0]>UF[2])
//            UF[2]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[3])
//    {
//        if(UF[0]>UF[3])
//            UF[3]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[1]==Un[2])
//    {
//        if(UF[1]>UF[2])
//            UF[2]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[1]==Un[3])
//    {
//        if(UF[1]>UF[3])
//            UF[3]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[2]==Un[3])
//    {
//        if(UF[2]>UF[3])
//            UF[3]=0;
//        else
//            UF[2]=0;
//    }
//    t1=UF[0]*UFF[Un[0]];
//    t2=UF[1]*UFF[Un[1]];
//    t3=UF[2]*UFF[Un[2]];
//    t4=UF[3]*UFF[Un[3]];
//    temp1=t1+t2+t3+t4;
//    temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
//    if(temp2!=0)
//        U=temp1/temp2;
//    else {
//        U=0;
//    }
////    temp1=PF[0]*UFF[Un[0]]+PF[1]*UFF[Un[1]]+PF[0]*UFF[Un[2]]+PF[1]*UFF[Un[3]]+DF[0]*UFF[Un[0]]+DF[0]*UFF[Un[1]]+DF[1]*UFF[Un[2]]+DF[0]*UFF[Un[3]];
////    U=temp1;
//    return U;
//}

//int rule_d[7] = { 6 , 5 , 3 , 2 , 3 , 5 , 6};//模糊规则表 D
//float Kd_Fuzzy(float EC)
//{
//    float out=0;
//    uint8 i=0;
//    float degree_left = 0,degree_right = 0;
//    uint8 degree_left_index = 0,degree_right_index = 0;
//    float DFF[7]={-12,-8,-4,0,4,8,12};
//    float UFF[7];
//
//    for(i=0;i<7;i++)
//            UFF[i]=kd_m/6*i;
//
//    if(EC<DFF[0])
//    {
//        degree_left = 1;
//        degree_right = 0;
//        degree_left_index = 0;
//    }
//    else if (EC>DFF[6]) {
//        degree_left = 1;
//        degree_right = 0;
//        degree_left_index = 6;
//    }
//    else {
//        for(i=0;i<6;i++)
//        {
//            if(EC>=DFF[i]&&EC<DFF[i+1])
//            {
//                degree_left = (float)(DFF[i+1] - EC)/(DFF[i+1] - DFF[i]);
//                degree_right = 1 - degree_left;
//                degree_left_index = i;
//                degree_right_index = i+1;
//                break;
//            }
//        }
//    }
//
//    out = UFF[rule_d[degree_left_index]]*degree_left+UFF[rule_d[degree_right_index]]*degree_right;
//
//    return out;
//}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      模糊PID
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
//int rule[7][7]=
//{
//    { 6 , 5 , 4 , 3 , 2 , 1 , 0},//0
//    { 5 , 4 , 3 , 2 , 1 , 0 , 1},//1
//    { 4 , 3 , 2 , 1 , 0 , 1 , 2},//2
//    { 3 , 2 , 1 , 0 , 1 , 2 , 3},//3
//    { 2 , 1 , 0 , 1 , 2 , 3 , 4},//4
//    { 1 , 0 , 1 , 2 , 3 , 4 , 5},//5
//    { 0 , 1 , 2 , 3 , 4 , 5 , 6},//6
//};//模糊规则表
//
//float Fuzzy(float E,float EC)
//{
//
//    /*输入量P语言值特征点*/
//    float EFF[7]={-45,-30,-15,0,15,30,45};
//    /*输入量D语言值特征点*/
//    float DFF[7]={-12,-8,-4,0,4,8,12};
//    /*输出量U语言值特征点(根据赛道类型选择不同的输出值)*/
////    float UFF[7]={0,0.05,0.2,1.6,8.5,14.5,22};
////    float UFF[7]={0,0.05,0.15,0.4,1.3,2.0,3.0};
//    float UFF[7]={0,0.3,0.6,0.9,1.2,1.5,1.8};
//
//    float U=0;  /*偏差,偏差微分以及输出值的精确量*/
//    float PF[2]={0},DF[2]={0},UF[4]={0};
//    /*偏差,偏差微分以及输出值的隶属度*/
//    int Pn=0,Dn=0,Un[4]={0};
//    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
//    /*隶属度的确定*/
//    /*根据PD的指定语言值获得有效隶属度*/
//    if(E>EFF[0] && E<EFF[6])
//    {
//        if(E<=EFF[1])
//        {
//            Pn=-2;
//            PF[0]=(EFF[1]-E)/(EFF[1]-EFF[0]);
//        }
//        else if(E<=EFF[2])
//        {
//            Pn=-1;
//            PF[0]=(EFF[2]-E)/(EFF[2]-EFF[1]);
//        }
//        else if(E<=EFF[3])
//        {
//            Pn=0;
//            PF[0]=(EFF[3]-E)/(EFF[3]-EFF[2]);
//        }
//        else if(E<=EFF[4])
//        {
//            Pn=1;
//            PF[0]=(EFF[4]-E)/(EFF[4]-EFF[3]);
//        }
//        else if(E<=EFF[5])
//        {
//            Pn=2;
//            PF[0]=(EFF[5]-E)/(EFF[5]-EFF[4]);
//        }
//        else if(E<=EFF[6])
//        {
//            Pn=3;
//            PF[0]=(EFF[6]-E)/(EFF[6]-EFF[5]);
//        }
//    }
//
//    else if(E<=EFF[0])
//    {
//        Pn=-2;
//        PF[0]=1;
//    }
//    else if(E>=EFF[6])
//    {
//        Pn=3;
//        PF[0]=0;
//    }
//
//    PF[1]=1-PF[0];
//
//
//    //判断D的隶属度
//    if(EC>DFF[0]&&EC<DFF[6])
//    {
//        if(EC<=DFF[1])
//        {
//            Dn=-2;
//            DF[0]=(DFF[1]-EC)/(DFF[1]-DFF[0]);
//        }
//        else if(EC<=DFF[2])
//        {
//            Dn=-1;
//            DF[0]=(DFF[2]-EC)/(DFF[2]-DFF[1]);
//        }
//        else if(EC<=DFF[3])
//        {
//            Dn=0;
//            DF[0]=(DFF[3]-EC)/(DFF[3]-DFF[2]);
//        }
//        else if(EC<=DFF[4])
//        {
//            Dn=1;
//            DF[0]=(DFF[4]-EC)/(DFF[4]-DFF[3]);
//        }
//        else if(EC<=DFF[5])
//        {
//            Dn=2;
//            DF[0]=(DFF[5]-EC)/(DFF[5]-DFF[4]);
//        }
//        else if(EC<=DFF[6])
//        {
//            Dn=3;
//            DF[0]=(DFF[6]-EC)/(DFF[6]-DFF[5]);
//        }
//    }
//    //不在给定的区间内
//    else if (EC<=DFF[0])
//    {
//        Dn=-2;
//        DF[0]=1;
//    }
//    else if(EC>=DFF[6])
//    {
//        Dn=3;
//        DF[0]=0;
//    }
//
//    DF[1]=1-DF[0];
//
//    /*使用误差范围优化后的规则表rule[7][7]*/
//    /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
//    /*一般都是四个规则有效*/
//    Un[0]=rule[Pn+2][Dn+2];
//    Un[1]=rule[Pn+3][Dn+2];
//    Un[2]=rule[Pn+2][Dn+3];
//    Un[3]=rule[Pn+3][Dn+3];
//
//    if(PF[0]<=DF[0])    //求小
//        UF[0]=PF[0];
//    else
//        UF[0]=DF[0];
//    if(PF[1]<=DF[0])
//        UF[1]=PF[1];
//    else
//        UF[1]=DF[0];
//    if(PF[0]<=DF[1])
//        UF[2]=PF[0];
//    else
//        UF[2]=DF[1];
//    if(PF[1]<=DF[1])
//        UF[3]=PF[1];
//    else
//        UF[3]=DF[1];
//    /*同隶属函数输出语言值求大*/
//    if(Un[0]==Un[1])
//    {
//        if(UF[0]>UF[1])
//            UF[1]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[2])
//    {
//        if(UF[0]>UF[2])
//            UF[2]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[0]==Un[3])
//    {
//        if(UF[0]>UF[3])
//            UF[3]=0;
//        else
//            UF[0]=0;
//    }
//    if(Un[1]==Un[2])
//    {
//        if(UF[1]>UF[2])
//            UF[2]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[1]==Un[3])
//    {
//        if(UF[1]>UF[3])
//            UF[3]=0;
//        else
//            UF[1]=0;
//    }
//    if(Un[2]==Un[3])
//    {
//        if(UF[2]>UF[3])
//            UF[3]=0;
//        else
//            UF[2]=0;
//    }
//    t1=UF[0]*UFF[Un[0]];
//    t2=UF[1]*UFF[Un[1]];
//    t3=UF[2]*UFF[Un[2]];
//    t4=UF[3]*UFF[Un[3]];
//    temp1=t1+t2+t3+t4;
//    temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
//    U=temp1/temp2;
//    return U;
//}
