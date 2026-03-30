#include "zf_common_headfile.h"
#include "math.h"

uint8 Half_zhidao_width[120]=
{
        21,21,21,21,21,21,21,21,21,21,
        21,21,21,21,21,21,21,21,21,21,
        21,21,21,21,21,21,21,21,21,21,
        21,21,21,21,21,21,21,21,22,23,
        23,23,24,25,25,26,26,27,27,28,
        29,30,31,31,32,32,33,34,34,35,
        36,36,37,37,38,39,39,40,40,41,
        42,42,43,44,44,45,46,47,47,48,
        48,49,50,50,51,51,52,53,53,54,
        55,55,56,56,57,57,58,59,59,60,
        61,61,62,63,63,64,64,65,66,66,
        67,68,68,69,70,70,71,72,72,73
};
uint16 t_1s = 0;
uint16 t_2s = 0;
uint16 t_5s = 0;
uint16 t_10s = 0;
uint16 t1_ms_cricle1 = 0;
uint16 t1_ms_cricle1_target;
uint16 t2_ms_cricle1 = 0;
uint16 t2_ms_cricle2 = 0;
uint16 t2_ms_cricle3 = 0;
uint16 t1_ms_cricle4 = 0;
uint16 t1_ms_cricle5 = 0;
uint8 T_1s_flag = 0;
uint8 T_2s_flag = 0;
uint8 T_10s_flag = 0;
uint8 T1_ms_cricle1_flag = 0;
uint8 T1_ms_cricle4_flag = 0;
uint8 T1_ms_cricle5_flag = 0;
uint8 circle_flag = 0;
uint8 circle_mode = 0;
uint8 left_circle_flag = 0;
uint8 right_circle_flag = 0;
uint8 left_circle_cnt = 0;
uint8 right_circle_cnt = 0;
uint8 left_circle_flag_cnt = 0;
uint8 right_circle_flag_cnt = 0;
float now_yaw = 0.0;
extern uint16 t2_ms_cricle1;
extern uint16 t2_ms_cricle2;
extern uint16 t2_ms_cricle3;
uint8 circle_width_left = 0;
uint8 circle_width_right = 0;
uint8 base_width = 0;
// 静态变量，记录是否已检测到第一次跳变（0→非0）
static int jump_flag = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     圆环识别
  @note
-------------------------------------------------------------------------------------------------------------------*/
void circle_find()
{
    //左环一号标志
       circle_width_left = circle_width_right = 0;
       if(L_corner_flag==1&&L_corner_row>5&&L_corner_row<MT9V03X_H-5&&
          abs(zx[L_corner_row]-zx[L_corner_row+2])<=5&&
          abs(zx[L_corner_row+1]-zx[L_corner_row+3])<=5&&
           abs(zx[L_corner_row+2]-zx[L_corner_row+4])<=5&&
              (zx[L_corner_row]-zx[L_corner_row-2])>=3&&
              (zx[L_corner_row]-zx[L_corner_row-3])>=6&&
              (zx[L_corner_row]-zx[L_corner_row-4])>=9)
       {
           circle_width_left = yx[L_corner_row-10]-zx[L_corner_row-10];
           L_circle_corner_row = L_corner_row;
           L_circle_corner_flag = 1;
       }
    //右环一号标志
       if(R_corner_flag==1&&R_corner_row>5&&R_corner_row<MT9V03X_H-5&&
          abs(yx[R_corner_row]-yx[R_corner_row+2])<=5&&//�ǵ����ֵ���Ը���
          abs(yx[R_corner_row+1]-yx[R_corner_row+3])<=5&&
          abs(yx[R_corner_row+2]-yx[R_corner_row+4])<=5&&
             (yx[R_corner_row]-yx[R_corner_row-2])<=-2&&
             (yx[R_corner_row]-yx[R_corner_row-3])<=-5&&
             (yx[R_corner_row]-yx[R_corner_row-4])<=-8)
       {
           circle_width_right = yx[R_corner_row-10]-zx[R_corner_row-10];
           R_circle_corner_row = R_corner_row;
           R_circle_corner_flag = 1;
       }

       base_width = yx[MT9V03X_H-2]-zx[MT9V03X_H-2];
      if(circle_mode==0&&L_circle_corner_flag==1&&
              Continuity_Change_Right(MT9V03X_H-30,30)==0
              &&Right_TotalLost(MT9V03X_H-30,Finnalline)<2&&L_corner_row>MT9V03X_H-80&&Left_Lost_Time>15)//&&Monotonicity_Change_Right()==0
      {
          left_circle_cnt++;
      }
      else if(left_circle_cnt!=0)
      {
          left_circle_cnt--;
      }
      if(left_circle_cnt==2)
      {
          left_circle_cnt=0;
          left_circle_flag_cnt++;
          circle_flag = 1;
          circle_mode = 1;//״̬1
          left_circle_flag = 1;
          buzzer();
          buzzer();
          buzzer();
      }
//左斜入
      if(circle_mode==0&&L_circle_corner_flag==1&&Continuity_Change_Right(MT9V03X_H-50,30)==0
              &&Right_TotalLost(MT9V03X_H-50,Finnalline)<2&&circle_width_left>135)//&&Monotonicity_Change_Right()==0&&T_2s_flag==1
      {
          circle_flag = 1;
          circle_mode = 1;//״̬1
          left_circle_flag = 1;
          buzzer();
          buzzer();
          buzzer();
      }
      if(left_circle_flag==1)//��Բ��
      {
        if(circle_mode==1)
        {
            int currentA = Left_sideIsNoLost(MT9V03X_H-5, MT9V03X_H-50);//试着写

            // 阶段1：检测0→非0跳变（且未标记过跳变）
              if (currentA != 0 && jump_flag == 0)
                  jump_flag = 1;  // 标记第一次跳变完成

              else  if(Boundry_Start_Left==MT9V03X_H-2
                    &&Right_TotalLost(MT9V03X_H-40,Finnalline)<2
                    &&currentA == 0 && jump_flag == 1)//&&&&Left_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-50)==0
            {
                t2_ms_cricle1 = 0;
                T1_ms_cricle1_flag = 0;
                circle_mode = 2;//222222222222!!!!!!!!!!!!
                buzzer();
                now_yaw = QEKF_INS.YawTotalAngle;
                jump_flag = 0;  // 重置标记，为下一周期准备
            }
        }
        if(circle_mode==2)
        {

            if(QEKF_INS.YawTotalAngle-now_yaw>=30)//ƫ���Ǵ���50��//Lin_circleup_row>MT9V03X_H-50&&Right_Lost_Time<=2
            {
                t2_ms_cricle2 = 0;
                circle_mode=3;//״̬3
                now_yaw = QEKF_INS.YawTotalAngle;
            }
        }
        if(circle_mode==3)
        {
            Circle_out_Left();
            if(QEKF_INS.YawTotalAngle-now_yaw>=200&&Lout_circleup_row>MT9V03X_H-80)//ƫ���Ǵ���200��
            {
                t2_ms_cricle3 = 0;
                circle_mode=4;//״̬4
                now_yaw = QEKF_INS.YawTotalAngle;
            }
        }
        if(circle_mode==4)
        {
            if(QEKF_INS.YawTotalAngle-now_yaw>=20)//ƫ���Ǵ���10��
            {
                if(Finnalline<=40&&Boundry_Start_Right==MT9V03X_H-2&&Continuity_Change_Right(MT9V03X_H-5,30)==0)
                {
                    circle_mode = 5;//״̬5
                    now_yaw = QEKF_INS.YawTotalAngle;
                }
            }
        }
        if(circle_mode==5)
        {
//            Circle_in_Left();
            if(T1_ms_cricle5_flag==1&&Left_sideIsNoLost(MT9V03X_H-10,MT9V03X_H-60)==0)//Left_Lost_Time<15&&Lin_circleup_flag!=1&&Lin_circleup_row>MT9V03X_H-40
            {
                circle_cnt++;
                T1_ms_cricle5_flag = 0;
                circle_flag=0;
                circle_mode=0;
                left_circle_flag=0;
                buzzer();
            }
        }


      }
      ////////////////////////////////////////////////右环标志位/////////////////////////////////////////////////////////////////
        //������
        if(circle_mode==0&&R_circle_corner_flag==1&&Continuity_Change_Left(MT9V03X_H-30,30)==0
                &&Left_TotalLost(MT9V03X_H-30,Finnalline)<2&&R_corner_row>MT9V03X_H-80&&Right_Lost_Time>15)//&&Monotonicity_Change_Left()==0
        {
            right_circle_cnt++;
        }
        else if(right_circle_cnt!=0)
        {
            right_circle_cnt--;
        }
        if(right_circle_cnt==2)
        {
            right_circle_cnt=0;
            right_circle_flag_cnt++;
            circle_flag = 1;
            circle_mode = 1;//״̬һ
            right_circle_flag = 1;
            buzzer();
            buzzer();
            buzzer();
        }
        //��б��
        if(T_2s_flag==1&&circle_mode==0&&R_circle_corner_flag==1&&Continuity_Change_Left(MT9V03X_H-50,30)==0
                &&Left_TotalLost(MT9V03X_H-5,Finnalline)<2&&R_corner_row>MT9V03X_H-80&&circle_width_right>135)//&&Monotonicity_Change_Left()==0
        {
            circle_flag = 1;
            circle_mode = 1;//״̬һ
            right_circle_flag = 1;
            buzzer();
            buzzer();
            buzzer();
        }
        if(right_circle_flag==1)//��Բ��
        {
          if(circle_mode==1)
          {
//              Circle_in_Right();
              if(T1_ms_cricle1_flag==1&&Boundry_Start_Right==MT9V03X_H-2&&Right_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-50)==0&&Left_TotalLost(MT9V03X_H-40,Finnalline)<2)//
              {
                  t2_ms_cricle1 = 0;
                  T1_ms_cricle1_flag = 0;
                  circle_mode = 2;//״̬��
                  now_yaw = QEKF_INS.YawTotalAngle;
              }
          }
          if(circle_mode==2)
          {
//              Circle_in_Right();
              if(now_yaw-QEKF_INS.YawTotalAngle>=30)//ƫ���Ǵ���50��//Rin_circleup_row>MT9V03X_H-50&&Left_Lost_Time<=2
              {
                  t2_ms_cricle2 = 0;
                  circle_mode=3;//״̬3
                  now_yaw = QEKF_INS.YawTotalAngle;
              }
          }
          if(circle_mode==3)
          {
              Circle_out_Right();
              if(now_yaw-QEKF_INS.YawTotalAngle>=200&&Rout_circleup_row>MT9V03X_H-80)//ƫ���Ǵ���200��
              {
                  t2_ms_cricle3 = 0;
                  circle_mode=4;//״̬4
                  now_yaw = QEKF_INS.YawTotalAngle;
              }
          }
          if(circle_mode==4)
          {
              if(now_yaw-QEKF_INS.YawTotalAngle>=20)//ƫ���Ǵ���10��
              {
                  if(Finnalline<=40&&Boundry_Start_Left==MT9V03X_H-2&&Continuity_Change_Left(MT9V03X_H-5,30)==0)//
                  {
                      circle_mode = 5;//״̬5
                      now_yaw = QEKF_INS.YawTotalAngle;
                  }
              }
          }
          if(circle_mode==5)
          {
//              Circle_in_Right();
              if(T1_ms_cricle5_flag==1&&Right_sideIsNoLost(MT9V03X_H-10,MT9V03X_H-60)==0)//Right_Lost_Time<15&&Rin_circleup_row>MT9V03X_H-40&&Rin_circleup_flag!=1
              {
                  circle_cnt++;
                  T1_ms_cricle5_flag = 0;
                  circle_flag=0;
                  circle_mode=0;
                  right_circle_flag=0;
                  buzzer();
              }
          }
        }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     圆环各阶段补线
  @note
-------------------------------------------------------------------------------------------------------------------*/
void is_circle()
{
    if(left_circle_flag==1)
    {
        if(circle_mode==1)//寻右边线
        {
            for(int i=MT9V03X_H-1;i>Finnalline;i--)
             {
                if(yx[i]-Half_zhidao_width[i]-5>=2)
                    wx[i]=yx[i]-Half_zhidao_width[i]-5;
                else wx[i]=2;
             }
        }
        if(circle_mode==2)//寻左边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(zx[i]+Half_zhidao_width[i]-5<=MT9V03X_W-3)//-25
                    wx[i]=zx[i]+Half_zhidao_width[i]-5;
                else wx[i]=MT9V03X_W-3;
             }
        }
        if(circle_mode==3)//寻左边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(zx[i]+Half_zhidao_width[i]+10<=MT9V03X_W-3)//-25
                    wx[i]=zx[i]+Half_zhidao_width[i]+10;
                else wx[i]=MT9V03X_W-3;
             }
        }
        if(circle_mode==4)//寻左边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(zx[i]+Half_zhidao_width[i]+5<=MT9V03X_W-3)//-25
                    wx[i]=zx[i]+Half_zhidao_width[i]+5;
                else wx[i]=MT9V03X_W-3;
             }
        }
        if(circle_mode==5)//寻右边线
        {
            for(int i=MT9V03X_H-1;i>Finnalline;i--)
             {
                if(yx[i]-Half_zhidao_width[i]>=2)
                    wx[i]=yx[i]-Half_zhidao_width[i];
                else wx[i]=2;
             }
        }
    }
////////////////////////////////////////////////////////
    if(right_circle_flag==1)
    {
        if(circle_mode==1)//寻左边线
        {
            for(int i=MT9V03X_H-1;i>Finnalline;i--)
             {
                if(zx[i]+Half_zhidao_width[i]+5<=MT9V03X_W-3)
                    wx[i]=zx[i]+Half_zhidao_width[i]+5;
                else wx[i]=MT9V03X_W-3;
             }
        }
        if(circle_mode==2)//寻右边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(yx[i]-Half_zhidao_width[i]+5>=2)//+25
                    wx[i]=yx[i]-Half_zhidao_width[i]+5;
                else wx[i]=2;
             }
        }
        if(circle_mode==3)//寻右边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(yx[i]-Half_zhidao_width[i]-10>=2)//+25
                    wx[i]=yx[i]-Half_zhidao_width[i]-10;
                else wx[i]=2;
             }
        }
        if(circle_mode==4)//寻右边线
        {
            for(int i=MT9V03X_H-1;i>70;i--)
             {
                if(yx[i]-Half_zhidao_width[i]-5>=2)
                    wx[i]=yx[i]-Half_zhidao_width[i]-5;
                else wx[i]=2;
             }
        }
        if(circle_mode==5)//寻左边线
        {
            for(int i=MT9V03X_H-1;i>Finnalline;i--)
             {
                if(zx[i]+Half_zhidao_width[i]<=MT9V03X_W-3)//-5
                    wx[i]=zx[i]+Half_zhidao_width[i];
                else wx[i]=MT9V03X_W-3;
             }
        }
    }
}

uint8 Cross_Flag=0;
uint8 Cross_mode=0;
uint8 leftzhengru_cnt=0;
uint8 rightzhengru_cnt=0;
uint8 leftxieru_cnt=0;
uint8 rightxieru_cnt=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     十字
  @note
-------------------------------------------------------------------------------------------------------------------*/
void cross_find()
{
    if(Cross_mode==0&&L_corner_flag == 1&&Both_Lost_Time>5)//左拐点找到 两边丢线初步判断十字&&Left_Lost_Time>10
    {
        get_upturningleft_point();//找到左上拐点
        if(L_upcorner_flag==1) leftzhengru_cnt++;
        else if(leftzhengru_cnt!=0) leftzhengru_cnt--;
    }
    else if(leftzhengru_cnt!=0) leftzhengru_cnt--;
    if(leftzhengru_cnt==2)
    {
        leftzhengru_cnt = 0;
        Cross_Flag=1;
        Cross_mode=1;
        ////////buzzer();
    }
    if(Cross_mode==0&&R_corner_flag == 1&&Both_Lost_Time>5)//右拐点找到 两边丢线初步判断十字&&Right_Lost_Time>10
    {
        get_upturningright_point();//找右上拐点
        if(R_upcorner_flag==1)  rightzhengru_cnt++;
        else if(rightzhengru_cnt!=0) rightzhengru_cnt--;
    }
    else if(rightzhengru_cnt!=0) rightzhengru_cnt--;
    if(rightzhengru_cnt==2)
    {
        rightzhengru_cnt = 0;
        Cross_Flag=1;
        Cross_mode=1;
       // //////buzzer();
    }
    //左斜入 三拐点
    if(Cross_mode==0&&L_corner_flag == 1&&R_corner_flag == 1&&Continuity_Change_Left(MT9V03X_H-10,50)!=0&&Both_Lost_Time>5)//&&(Left_Lost_Time+Right_Lost_Time)>9
    {
        get_upturningright_point();//找右上拐点
        if(R_upcorner_flag==1) leftxieru_cnt++;
        else if(leftxieru_cnt!=0) leftxieru_cnt--;
    }
    else if(leftxieru_cnt!=0) leftxieru_cnt--;
    if(leftxieru_cnt==2)
    {
        leftxieru_cnt = 0;
        Cross_Flag=1;
        Cross_mode=1;
       // //////buzzer();
    }
    //右斜入 三拐点
    if(Cross_mode==0&&L_corner_flag == 1&&R_corner_flag == 1&&Continuity_Change_Right(MT9V03X_H-10,50)!=0&&Both_Lost_Time>5)//&&(Left_Lost_Time+Right_Lost_Time)>9
    {
        get_upturningleft_point();//找到左上拐点
        if(L_upcorner_flag==1) rightxieru_cnt++;
        else if(rightxieru_cnt!=0) rightxieru_cnt--;
    }
    else if(rightxieru_cnt!=0) rightxieru_cnt--;
    if(rightxieru_cnt==2)
    {
        rightxieru_cnt = 0;
        Cross_Flag=1;
        Cross_mode=1;
       // //////buzzer();
    }

    if(Cross_mode==1)//从下往上
    {
        XUNXIAN_cross(MT9V03X_H-10,MT9V03X_W);
        get_upturningleft_point();//找左上拐点
        get_upturningright_point();//找左上拐点
        //左
        if(L_corner_flag == 1&&L_upcorner_flag==1)
        {
            //补线
            Left_Add_Line(L_upcorner_col,L_upcorner_row,L_corner_col,L_corner_row);
        }
        if(L_corner_flag == 1&&L_upcorner_flag!=1)
        {
            //补线
            Lengthen_Left_downBoundry(L_corner_row,20);
        }
        if(L_corner_flag != 1&&L_upcorner_flag==1)
        {
            //补线
            Lengthen_Left_Boundry(L_upcorner_row-1,MT9V03X_H-1);
        }
        if(L_corner_flag != 1&&L_upcorner_flag!=1)
        {
//            Left_Add_Line(MT9V03X_W/3,20,30,MT9V03X_H-20);
        }
        //右
        if(R_corner_flag == 1&&R_upcorner_flag==1)
        {
            //补线
            Right_Add_Line(R_upcorner_col,R_upcorner_row,R_corner_col,R_corner_row);
        }
        if(R_corner_flag == 1&&R_upcorner_flag!=1)
        {
            //补线
            Lengthen_Right_downBoundry(R_corner_row,20);
        }
        if(R_corner_flag != 1&&R_upcorner_flag==1)
        {
            //补线
            Lengthen_Right_Boundry(R_upcorner_row-1,MT9V03X_H-1);
        }
        if(R_corner_flag != 1&&R_upcorner_flag!=1)
        {
//            Right_Add_Line(MT9V03X_W/3*2,20,MT9V03X_W-30,MT9V03X_H-20);
        }
        //结束判断
        if( (L_corner_flag != 1&&R_corner_flag != 1&&Both_Lost_Time>8) || (Boundry_Start_Left!=MT9V03X_H-2&&Boundry_Start_Right!=MT9V03X_H-2) )
        {
            Cross_mode=2;
        }
    }
    if(Cross_mode==2)//从上往下
    {
        XUNXIAN_cross(MT9V03X_H-10,MT9V03X_W);
        get_upturningleft_point();//找左上拐点
        get_upturningright_point();//找左上拐点
        if(L_upcorner_flag==1)
        {
            //补线
            Lengthen_Left_Boundry(L_upcorner_row-1,MT9V03X_H-1);
        }
        else
        {
//            Left_Add_Line(MT9V03X_W/3,20,30,MT9V03X_H-20);
        }
        if(R_upcorner_flag==1)
        {
            //补线
            Lengthen_Right_Boundry(R_upcorner_row-1,MT9V03X_H-1);
        }
        else
        {
//            Right_Add_Line(MT9V03X_W/3*2,20,MT9V03X_W-30,MT9V03X_H-20);
        }
        if(L_upcorner_flag!=1&&R_upcorner_flag!=1&&Both_Lost_Time==0)//(L_upcorner_row>=71&&Both_Lost_Time<24)||(R_upcorner_row>=71&&Both_Lost_Time<24)||
        {
            Cross_mode = 3;
        }
    }
    if(Cross_mode==3)
    {
        cross_cnt++;
        Cross_Flag=0;
        Cross_mode=0;
        L_upcorner_flag=0;R_upcorner_flag=0;
       // //////buzzer();
    }
}





/*-------------------------------------------------------------------------------------------------------------------
  @brief     左补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        zx[i]=hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右补线
  @param     补线的起点，终点
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      补的直接是边界，点最好是可信度高的,不要乱补
-------------------------------------------------------------------------------------------------------------------*/
void Right_Add_Line(int x1,int y1,int x2,int y2)//左补线,补的是边界
{
    int i,max,a1,a2;
    int hx;
    if(x1>=MT9V03X_W-1)//起始点位置校正，排除数组越界的可能
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//坐标互换
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//根据斜率补线即可
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
        yx[i]=hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Left_Add_Line(zx[start],start,zx[end],end);
    }
    else
    {
        k=(float)(zx[start]-zx[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            zx[i]=(int)(i-start)*k+zx[start];//(x=(y-y1)*k+x1),点斜式变形
            if(zx[i]>=MT9V03X_W-1)
            {
                zx[i]=MT9V03X_W-1;
            }
            else if(zx[i]<=0)
            {
                zx[i]=0;
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界从下往上延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Left_downBoundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Left_Add_Line(zx[end],end,zx[start],start);
    }
    else
    {
        k=(float)(zx[start]-zx[start+4])/5.0;//这里的k是1/斜率
        for(i=end;i<=start;i++)
        {
            zx[i]=(int)(start-i)*k+zx[start];//(x=(y-y1)*k+x1),点斜式变形
            if(zx[i]>=MT9V03X_W-1)
            {
                zx[i]=MT9V03X_W-1;
            }
            else if(zx[i]<=0)
            {
                zx[i]=0;
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-1)//起始点位置校正，排除数组越界的可能
        start=MT9V03X_H-1;
    else if(start<=0)
        start=0;
    if(end>=MT9V03X_H-1)
        end=MT9V03X_H-1;
    else if(end<=0)
        end=0;
    if(end<start)//++访问，坐标互换
    {
        t=end;
        end=start;
        start=t;
    }

    if(start<=5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(yx[start],start,yx[end],end);
    }
    else
    {
        k=(float)(yx[start]-yx[start-4])/5.0;//这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            yx[i]=(int)(i-start)*k+yx[start];//(x=(y-y1)*k+x1),点斜式变形
            if(yx[i]>=MT9V03X_W-1)
            {
                yx[i]=MT9V03X_W-1;
            }
            else if(yx[i]<=0)
            {
                yx[i]=0;
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界从下往上延长
  @param     延长起始行数，延长到某行
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)；
  @note      从起始点向上找3个点，算出斜率，向下延长，直至结束点
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_downBoundry(int start,int end)
{
    int i,t;
    float k=0;
    if(start>=MT9V03X_H-5)//因为需要在开始点向上找3个点，对于起始点过于靠上，不能做延长，只能直接连线
    {
        Right_Add_Line(yx[end],end,yx[start],start);
    }
    else
    {
        k=(float)(yx[start]-yx[start+4])/5.0;//这里的k是1/斜率
        for(i=end;i<=start;i++)
        {
            yx[i]=(int)(start-i)*k+yx[start];//(x=(y-y1)*k+x1),点斜式变形
            if(yx[i]>=MT9V03X_W-1)
            {
                yx[i]=MT9V03X_W-1;
            }
            else if(yx[i]<=0)
            {
                yx[i]=0;
            }
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left(int start,int end)
{
    int i;
    int continuity_change_flag=0;
//    if(circle_mode==4)
//    {
//        for(i=start;i>end;i--)
//        {
//            if(zx[i-1]-zx[i]>=3)//连续性阈值是5，可更改
//           {
//                continuity_change_flag=i;
//                break;
//           }
//        }
//    }
//    else
//    {
        for(i=start;i>end;i--)
        {
            if(abs(zx[i]-zx[i-1])>=3)//连续性阈值是5，可更改
           {
                continuity_change_flag=i;
                break;
           }
        }
//    }
    return continuity_change_flag;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Right(int start,int end)
{
    int i;
    int continuity_change_flag=0;
//    if(circle_mode==4)
//    {
//        for(i=start;i>end;i--)
//        {
//            if(yx[i]-yx[i-1]>=3)//连续性阈值是5，可更改
//           {
//                continuity_change_flag=i;
//                break;
//           }
//        }
//    }
//    else
//    {
        for(i=start;i>end;i--)
        {
            if(abs(yx[i]-yx[i-1])>=3)//连续性阈值是5，可更改
           {
                continuity_change_flag=i;
                break;
           }
        }
//    }
    return continuity_change_flag;
}






/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界无丢线检测
  @param
  @return    区域无丢线返回0 有丢线返回丢线那行
  Sample
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Left_sideIsNoLost(int start,int end)
{
    int i;
    int Left_sideIsNoLost_flag=0;
    for(i=start;i>end;i--)
    {
        if(zx[i]  == 1)
       {
            Left_sideIsNoLost_flag=i;
            break;
       }
    }
    return Left_sideIsNoLost_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界无丢线检测
  @param
  @return    区域无丢线返回0 有丢线返回丢线那行
  Sample
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Right_sideIsNoLost(int start,int end)
{
    int i;
    int Right_sideIsNoLost_flag=0;
    for(i=start;i>end;i--)
    {
        if(yx[i] == MT9V03X_W-2)
       {
            Right_sideIsNoLost_flag=i;
            break;
       }
    }
    return Right_sideIsNoLost_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左边界区域丢线数检测
  @param
  @return    区域无丢线返回0 有丢线返回丢线数
  Sample
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Left_TotalLost(int start,int end)
{
    int i;
    int Left_TotalLost=0;
    for(i=start;i>end;i--)
    {
        if (zx[i]  == 1)//单边丢线数
            Left_TotalLost++;
    }
    return Left_TotalLost;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     右边界区域丢线数检测
  @param
  @return    区域无丢线返回0 有丢线返回丢线数
  Sample
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Right_TotalLost(int start,int end)
{
    int i;
    int Right_TotalLost=0;
    for(i=start;i>end;i--)
    {
        if (yx[i] == MT9V03X_W-2)//单边丢线数
            Right_TotalLost++;
    }
    return Right_TotalLost;
}

int left_change_cnt = 0;
int right_change_cnt = 0;
int left_Bwjump_counting(int start,int end)
{
    int i,j;
    int left_change_count = 0;
    //从下往上
    for(i=start;i>=end;i--)
    {
        //从中线往左边界
        for(j=wx[i];j>=zx[i];j--)
        {
            if(image[i][j]==0)
            {
                left_change_count++;
            }
        }
    }
    return left_change_count;
}

int right_Bwjump_counting(int start,int end)
{
    int i,j;
    int right_change_count = 0;
    //从下往上
    for(i=start;i>=end;i--)
    {
        //从中线往左边界
        for(j=wx[i];j<=yx[i];j++)
        {
            if(image[i][j]==0)
            {
                right_change_count++;
            }
        }
    }
    return right_change_count;
}

uint8 Barrier_Flag=0;
uint8 left_Barrier_Flag = 0;
uint8 right_Barrier_Flag = 0;
uint8 barrier_mode = 0;
uint8 left_Barrier_cnt = 0;
uint8 right_Barrier_cnt = 0;
uint8 barrier_width_left = 0;
uint8 barrier_width_right = 0;
uint8 barrier_width_avg = 0;
uint8 BlackFind_Left_Flag = 0;
uint8 BlackFind_Right_Flag = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     小障碍
  @note
-------------------------------------------------------------------------------------------------------------------*/
void barrier_find()
{

    get_upturningleft_point();//找到左上拐点
    get_upturningright_point();//找右上拐点

    barrier_width_left = barrier_width_right = barrier_width_avg = BlackFind_Left_Flag = BlackFind_Right_Flag = 0;
    if(L_upcorner_flag==1&&L_upcorner_row>5&&L_upcorner_row<MT9V03X_H-5)
    {
        barrier_width_left = yx[L_upcorner_row-10]-zx[L_upcorner_row-10];
    }
    if(R_upcorner_flag==1&&R_upcorner_row>5&&R_upcorner_row<MT9V03X_H-5)
    {
        barrier_width_right = yx[R_upcorner_row-10]-zx[R_upcorner_row-10];
    }
    if(barrier_width_left!=0&&barrier_width_right!=0)
    {
        barrier_width_avg = (barrier_width_left + barrier_width_right)/2;
    }

    //左障碍 跟右边线
    if(barrier_mode==0&&L_upcorner_flag==1&&Continuity_Change_Right(MT9V03X_H-10,40)==0&&barrier_width_left<=65&&barrier_width_left>=40)//&&Right_Lost_Time<10&&Left_Lost_Time<10
    {
        left_Barrier_cnt++;
    }
    else if(left_Barrier_cnt!=0)
    {
        left_Barrier_cnt--;
    }
    if(left_Barrier_cnt==3)
    {
        left_Barrier_cnt = 0;
        left_Barrier_Flag = 1;
        Barrier_Flag = 1;
        barrier_mode = 1;
//        //////buzzer();
    }
    //右障碍 跟左边线
    if(barrier_mode==0&&R_upcorner_flag==1&&Continuity_Change_Left(MT9V03X_H-10,40)==0&&barrier_width_right<=65&&barrier_width_right>=50)//&&Right_Lost_Time<10&&Left_Lost_Time<10
    {
        right_Barrier_cnt++;
    }
    else if(right_Barrier_cnt!=0)
    {
        right_Barrier_cnt--;
    }
    if(right_Barrier_cnt==3)
    {
        right_Barrier_cnt = 0;
        right_Barrier_Flag = 1;
        Barrier_Flag = 1;
        barrier_mode = 1;
//        ////buzzer();
    }

    if(left_Barrier_Flag==1)
    {
        if(barrier_mode==1&&Continuity_Change_Left(MT9V03X_H-10,50)==0)
        {
            left_Barrier_Flag=0;
            Barrier_Flag=0;
            barrier_mode=0;
        }
    }

    if(right_Barrier_Flag==1)
    {
        if(barrier_mode==1&&Continuity_Change_Right(MT9V03X_H-10,50)==0)
        {
            right_Barrier_Flag=0;
            Barrier_Flag=0;
            barrier_mode=0;
        }
    }

//    if(barrier_mode == 2)
//    {
//        for(int i = MT9V03X_H-5;i>MT9V03X_H-15;i--)
//        {
//            if(yx[i]-zx[i]>110)
//            {
//                barrier_mode = 3;
//            }
//            else
//            {
//                barrier_mode = 2;
//                break;
//            }
//        }
//    }
//    if(barrier_mode==3)
//    {
//        Barrier_Flag=0;
//        barrier_mode=0;
//        left_Barrier_Flag=0;
//        right_Barrier_Flag=0;
//    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     障碍补线
  @note
-------------------------------------------------------------------------------------------------------------------*/
void is_barrier()
{
    if(left_Barrier_Flag==1)//右单边
    {
        for(int i=MT9V03X_H-1;i>Finnalline;i--)
         {
            if(yx[i]-Half_zhidao_width[i]+10>=2)
                wx[i]=yx[i]-Half_zhidao_width[i]+10;
            else wx[i]=2;
         }
    }

    if(right_Barrier_Flag==1)//左单边
    {
        for(int i=MT9V03X_H-1;i>Finnalline;i--)
         {
            if(zx[i]+Half_zhidao_width[i]-10<=MT9V03X_W-3)
                wx[i]=zx[i]+Half_zhidao_width[i]-10;
            else wx[i]=MT9V03X_W-3;
         }
    }

    ///////////////////////////////////////////////////////

//    if(Left_SingleBridge_Flag==1&&SingleBridge_mode==1)//右单边
//    {
//        for(int i=MT9V03X_H-1;i>70;i--)
//         {
//            if(yx[i]-Half_zhidao_width[i]+13>=2)
//                wx[i]=yx[i]-Half_zhidao_width[i]+13;
//            else wx[i]=2;
//         }
//    }
//
//    if(Right_SingleBridge_Flag==1&&SingleBridge_mode==1)//左单边
//    {
//        for(int i=MT9V03X_H-1;i>70;i--)
//         {
//            if(zx[i]+Half_zhidao_width[i]-20<=MT9V03X_W-3)
//                wx[i]=zx[i]+Half_zhidao_width[i]-20;
//            else wx[i]=MT9V03X_W-3;
//         }
//    }
}

uint8 Zebra_Flag = 0;
uint8 zebra_mode = 0;
uint8 zebra_stop_flag = 0;
uint16 zebra_mscnt = 0;
int change_count=0;//跳变计数
/*-------------------------------------------------------------------------------------------------------------------
  @brief     斑马线检测
  @param     null
  @return    null
  Sample     Zebra_find(void)
  @note      边界起始靠下，最长白列较长，赛道宽度过窄，且附近大量跳变
-------------------------------------------------------------------------------------------------------------------*/
void Zebra_find(void)
{
    int i=0,j=0;
    change_count=0;//跳变计数

    if(T_10s_flag==1&&zebra_mode==0
            &&Finnalline<=50)//&&Continuity_Change_Left(MT9V03X_H-20,50)==0&&Continuity_Change_Right(MT9V03X_H-20,50)==0
        //&&&&Boundry_Start_Left>=MT9V03X_H-20&&Boundry_Start_Right>=MT9V03X_H-20
    {
        for(i=80;i>=60;i--)//区域内跳变计数 80-60
        {
            for(j=zx[i];j<=yx[i];j++)
            {
                if(image[i][j+1]-image[i][j]!=0)
                {
                    change_count++;
                }
            }
        }
    }
    if(change_count>30)//跳变大于某一阈值，认为找到了斑马线
    {
        zebra_mode=1;
        Zebra_Flag=1;
//        ////buzzer();
    }
    if(zebra_mode==1&&zebra_mscnt<400&&yx[MT9V03X_H-10]-zx[MT9V03X_H-10]<50)
    {
        zebra_mscnt=0;
        zebra_mode=0;
        Zebra_Flag=0;
        ////buzzer();
        stop_flag=1;
//        zebra_stop_flag=1;
    }
    if(zebra_mscnt >= 600)//退出
    {
        zebra_mscnt=0;
        zebra_mode = 0;
        Zebra_Flag=0;
    }
}

uint8 Ramp_Flag=0;
uint8 Ramp_mode=0;
uint8 Ramp_cnt=0;
uint8 Ramp_cnt_1=0;
uint16 Ramp_ms_cnt=0;
uint8 Ramp_End_Flag=0;
uint8 UnRamp_Flag=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     坡道检测
  @param     null
  @return    null
  Sample     Ramp_find();
  @note      赛宽变宽，测距前面有东西，丢线数少，截止行上面不丢线
-------------------------------------------------------------------------------------------------------------------*/
void Ramp_find(void)
{
    if(UnRamp_Flag==0&&Ramp_mode==0&&Extrawide_counting(MT9V03X_H-5,Finnalline)>=30&&Finnalline<=25
            &&Left_TotalLost(MT9V03X_H-30,Finnalline)<=2&&Right_TotalLost(MT9V03X_H-30,Finnalline)<=2
            &&dl1b_distance_mm<=800)//T_2s_flag==1&&
    {
        Ramp_cnt++;
    }
    else if(Ramp_cnt!=0)
    {
        Ramp_cnt--;
    }
    if(Ramp_cnt==3)
    {
        Ramp_cnt = 0;
        Ramp_Flag = 1;
        Ramp_mode = 1;
        ////buzzer();
    }
    if(Ramp_mode==1&&dl1b_distance_mm>8000&&Extrawide_counting(MT9V03X_H-5,Finnalline)<=5)//&&QEKF_INS.Pitch<=-10
    {
        Ramp_cnt_1++;
    }
    else if(Ramp_cnt_1!=0)
    {
        Ramp_cnt_1--;
    }
    if(Ramp_cnt_1==3)
    {
        Ramp_cnt_1 = 0;
        Ramp_mode = 2;
    }
    if(Ramp_mode==2)//&&QEKF_INS.Pitch>=-5
    {
        Ramp_Flag = 0;
        ramp_cnt++;
        Ramp_mode = 0;
        Ramp_End_Flag = 1;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     赛道超宽部分数
  @note
-------------------------------------------------------------------------------------------------------------------*/
int Extrawide_counting(int start,int end)
{
    int i,j;
    int count = 0;
    //从下往上
    for(i=start;i>=end;i--)
    {
        if(yx[i]-zx[i]-2*Half_zhidao_width[i]>25)//图像赛宽比标准赛宽大
        {
            count++;//赛宽过宽行
        }
    }
    return count;
}

uint8 circle_cnt = 1;
uint8 cross_cnt = 1;
uint8 barrier_cnt = 1;
uint8 ramp_cnt = 1;
char temp_string[100];
char temp_character = 'A';
void Element_cnt_init()
{
    circle_cnt = 1;
    cross_cnt = 1;
    barrier_cnt = 1;
    ramp_cnt = 1;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief    灯光秀
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Led_process()
{
//    if(motor_enable==0)//发车前和停车
//    {
//        dot_matrix_screen_show_string("NG ");
//    }
//    else if(Cross_Flag==1)
//    {
//        temp_character = 'A';
//        sprintf(temp_string, "%c%d", temp_character, cross_cnt);
//        dot_matrix_screen_show_string(temp_string);
//    }
//    else if(circle_flag==1)
//    {
//        temp_character = 'B';
//        sprintf(temp_string, "%c%d", temp_character, circle_cnt);
//        dot_matrix_screen_show_string(temp_string);
//    }
//    else if(Barrier_Flag==1)
//    {
//        temp_character = 'C';
//        sprintf(temp_string, "%c%d", temp_character, barrier_cnt);
//        dot_matrix_screen_show_string(temp_string);
//    }
//    else if(Ramp_Flag==1)
//    {
//        temp_character = 'D';
//        sprintf(temp_string, "%c%d", temp_character, ramp_cnt);
//        dot_matrix_screen_show_string(temp_string);
//    }
//    else if(Err>10)
//    {
//        dot_matrix_screen_show_string("<--");
//    }
//    else if(Err<-10)
//    {
//        dot_matrix_screen_show_string("-->");
//    }
//    else
//    {
//        dot_matrix_screen_show_string("OK ");
//    }
}


uint8 Left_SingleBridge_Flag = 0;
uint8 Right_SingleBridge_Flag = 0;
uint8 SingleBridge_Flag = 0;
uint8 SingleBridge_mode = 0;
uint8 SingleBridge_cnt = 0;
float SingleBridge_err = 0.0;
float Total_err_sum = 0.0;
uint16 SingleBridge_ms_cnt=0;
uint8 SingleBridgeEnd_Flag=0;
uint8 T1_SingleBridge_flag=0;
uint8 SingleBridge_width_left = 0;
uint8 SingleBridge_width_right = 0;
int SingleBridge_width_chazhi = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     单边桥
  @note
-------------------------------------------------------------------------------------------------------------------*/
void SingleBridge_find()
{
    SingleBridge_err = 0.0;
    get_SingleBridgeleft_point();//找到左上拐点
    get_SingleBridgeright_point();//找右上拐点

    SingleBridge_width_left = SingleBridge_width_right = SingleBridge_width_chazhi = 0;
    if(L_SingleBridge_flag==1&&L_SingleBridge_row>20&&L_SingleBridge_row<70)//
    {
        SingleBridge_width_left = yx[L_SingleBridge_row-10]-zx[L_SingleBridge_row-10];
    }
    if(R_SingleBridge_flag==1&&R_SingleBridge_row>20&&R_SingleBridge_row<70)//
    {
        SingleBridge_width_right = yx[R_SingleBridge_row-10]-zx[R_SingleBridge_row-10];
    }

    //判断情况1 障碍分布左 右 左
    if(SingleBridge_mode==0&&L_SingleBridge_flag==1&&
            SingleBridge_width_left>=10&&SingleBridge_width_left<=30&&Left_Lost_Time<=5&&Right_Lost_Time<=5)//
    {
        Finnalline = 10;
        clear_find_point();
        search_neighborhood_zeroone();
        get_SingleBridgeright_point();//找右上拐点
        if(R_SingleBridge_flag==1&&abs(R_SingleBridge_col - L_SingleBridge_col)<=5)//&&R_SingleBridge_row>20
        {
            now_yaw = QEKF_INS.YawTotalAngle ;//+ Err*0.15
            SingleBridge_Flag = 1;
            Left_SingleBridge_Flag = 1;
            SingleBridge_mode = 1;
//            Jumpbarrier_Flag = 1;
//            Jumpbarrier_mode = 1;
            ////buzzer();
        }
    }

    //判断情况2 障碍分布右 左 右
    if(SingleBridge_mode==0&&R_SingleBridge_flag==1&&
            SingleBridge_width_right>=10&&SingleBridge_width_right<=30&&Left_Lost_Time<=5&&Right_Lost_Time<=5)//
    {
        Finnalline = 10;
        clear_find_point();
        search_neighborhood_zeroone();
        get_SingleBridgeleft_point();//找到左上拐点
        if(L_SingleBridge_flag==1&&abs(R_SingleBridge_col - L_SingleBridge_col)<=5)//&&L_SingleBridge_row>20
        {
            now_yaw = QEKF_INS.YawTotalAngle ;//+ Err*0.15
            SingleBridge_Flag = 1;
            Right_SingleBridge_Flag = 1;
            SingleBridge_mode = 1;
//            Jumpbarrier_Flag = 1;
//            Jumpbarrier_mode = 1;
            ////buzzer();
        }
    }


    //左单边桥
    if(Left_SingleBridge_Flag==1)//跟右边线
    {
        if(SingleBridge_mode==1)
        {
            if(T1_SingleBridge_flag==1&&Continuity_Change_Right(MT9V03X_H-10,30)!=0&&zx[MT9V03X_H-5]>50&&zx[MT9V03X_H-5]<140)
            {
                SingleBridge_mode=2;
                T1_SingleBridge_flag=0;
//                stop_flag=1;
            }
        }
        if(SingleBridge_mode==2)
        {
            if(SingleBridgeEnd_Flag==1)
            {
                Left_SingleBridge_Flag = 0;
                SingleBridgeEnd_Flag = 0;
                SingleBridge_Flag = 0;
                SingleBridge_mode = 0;
            }
        }
    }

    //右单边桥
    if(Right_SingleBridge_Flag==1)
    {
        if(SingleBridge_mode==1)
        {
            if(T1_SingleBridge_flag==1&&Continuity_Change_Left(MT9V03X_H-10,30)!=0&&yx[MT9V03X_H-5]<140&&yx[MT9V03X_H-5]>50)
            {
                SingleBridge_mode=2;
                T1_SingleBridge_flag=0;
//                stop_flag=1;
            }
        }
        if(SingleBridge_mode==2)
        {
            if(SingleBridgeEnd_Flag==1)
            {
                Right_SingleBridge_Flag = 0;
                SingleBridgeEnd_Flag = 0;
                SingleBridge_Flag = 0;
                SingleBridge_mode = 0;
            }
        }
    }

}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     蜂鸣器
  @note
-------------------------------------------------------------------------------------------------------------------*/
void buzzer()
{
    gpio_set_level(P33_10, GPIO_HIGH); // 启动蜂鸣器
    system_delay_ms(60);               // 保持发声时长（例如50ms）
    gpio_set_level(P33_10, GPIO_LOW);  // 关闭蜂鸣器
}

void buzzer_stop()
{
    gpio_toggle_level(P33_10);
    system_delay_ms(50);
    gpio_set_level(P33_10, GPIO_LOW);
}

uint8 Jumpbarrier_Flag=0;
uint8 Jumpbarrier_mode=0;
uint8 Jumpbarrier_cnt=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     垂直跳跃
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Jumpbarrier_find()
{
    if(Jumpbarrier_mode==0&&Finnalline>=45
            &&Left_TotalLost(MT9V03X_H-30,Finnalline)<=2&&Right_TotalLost(MT9V03X_H-30,Finnalline)<=2
            &&dl1b_distance_mm<=245)//
    {
        Jumpbarrier_cnt++;
    }
    else if(Jumpbarrier_cnt!=0)
    {
        Jumpbarrier_cnt--;
    }
    if(Jumpbarrier_cnt==3)
    {
        now_yaw = QEKF_INS.YawTotalAngle ;//+ Err*0.15
        Jumpbarrier_cnt = 0;
        Jumpbarrier_Flag = 1;
        Jumpbarrier_mode = 1;
        ////buzzer();
    }
}

uint8 stop_flag = 0;
uint8 stop_cnt = 0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     停车识别
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Stop_car()
{
    if(stop_flag!=1&&T_2s_flag==1)
    {
        if( (mt9v03x_image[MT9V03X_H-1][ wx[MT9V03X_H-1] ]<=Threshold&&
                mt9v03x_image[MT9V03X_H-15][ wx[MT9V03X_H-15] ]<=Threshold&&
                mt9v03x_image[MT9V03X_H-30][ wx[MT9V03X_H-30] ]<=Threshold))
        {
            stop_cnt++;
        }
        else if(stop_cnt!=0)
        {
            stop_cnt--;
        }
        if(stop_cnt==3)
        {
            stop_cnt = 0;
            stop_flag = 1;
        }
    }
}

