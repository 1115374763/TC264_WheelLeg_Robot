
#include "zf_common_headfile.h"
#include "math.h"


#pragma section all "cpu0_psram"

#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
int8 page_index = 0;//页数标识
int8 row_index = 1;//行数标识

float delta = 0;
uint32 delta_flag_left = 1000000;
uint32 delta_flag_right = 1000000;

uint8 camera_enable = 0;
uint8 jiasu_enable = 1;
uint32 tim = 0;
uint8 motor_enable = 0;
extern int16 momentumwheel_encoder_forward;
extern int16 momentumwheel_encoder_after;
extern float Exceptspeed;extern float BASE_p;
float angle_banlance_error = 0;
float location_banlance_error = 0;
float vertical_banlance_error = 0;
float servo_xunji_error = 0;
extern uint16 t1_ms_cricle1_target;
extern float BASE_p2;
uint16 light = 212;
extern uint8 left[image_h];
extern uint8 right[image_h];
extern uint8 middle[image_h];
extern double zhonxianpiancha;
void (*current_operation_index)(void);

typedef struct
{
        int8 current;//当前索引号
        int8 up;//向上翻索引号
        int8 down;//向下翻索引号
        int8 max_row_index;//每一页最大行数
        void (*current_operation)();        //对应执行函数
}key_table;

void page1(void);
void page2(void);
void page3(void);
void page4(void);
void page5(void);
void page6(void);
void page7(void);
void page8(void);

key_table table[8]=
{
        //索引，上，下，最大行，打印函数
        {0,7,1,12,(*page1)},
        {1,0,2,7,(*page2)},
        {2,1,3,5,(*page3)},
        {3,2,4,7,(*page4)},
        {4,3,5,7,(*page5)},
        {5,4,6,3,(*page6)},
        {6,5,7,7,(*page7)},
        {7,6,0,8,(*page8)},
};

/*-------------------------------------------------------------------------------------------------------------------
  @brief     菜单调用函数
  @note
-------------------------------------------------------------------------------------------------------------------*/
void Menu(void)
{
    row_control();
    adjust_control();


    current_operation_index=table[page_index].current_operation;        //调取函数

    (*current_operation_index)();//执行当前操作函数

    delta = 0;

}


//行数的增减控制
void row_control(void)
{

    if(key_up == 0 )
    {

        ips200_clear();
        page_index=table[page_index].current;
        row_index--;
        while(!key_up);//松手检测
    }
    if(key_down== 0)
    {

        ips200_clear();
        page_index=table[page_index].current;
        row_index++;

        while(!key_down);//松手检测key_down
    }
    if(key_mid== 0)
    {
        ips200_clear();
        motor_enable = 1;
        while(!key_mid);//松手检测key_down
    }
    if(row_index > table[page_index].max_row_index)
    {
        page_index=table[page_index].down;
        row_index = 1;
    }
    else if(row_index < 1)
    {
        page_index=table[page_index].up;
        row_index = table[page_index].max_row_index;
    }
}

//变量的增减控制
void adjust_control(void)
{
    uint32 delta_flags = 0;
    if(key_left == 0)
    {

        ips200_clear();
        page_index=table[page_index].current;
        delta = 1;
        while(!key_left)
        {
            if(delta_flags >= delta_flag_left)
            {
                delta_flag_left = 50000;
                break;
            }
            else
            {
                delta_flags++;
            }
        }
    }
    else
    {
        delta_flag_left = 1000000;
    }
    if(key_right == 0)
    {

        ips200_clear();
        page_index=table[page_index].current;
        delta = -1;
        while(!key_right)
        {
            if(delta_flags >= delta_flag_right)
            {
                delta_flag_right = 50000;
                break;
            }
            else
            {
                delta_flags++;
            }
        }
    }
    else
    {
        delta_flag_right = 1000000;
    }
}
int i;
//第一页
void page1(void)
{

    //显示图像

    ips200_show_string(0,0,"current page1");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {


//
//        case 1:
//             Mode_sudu += 1*delta;  // 修改为操作模式变量
//             MenuElementInit();     // 初始化当前模式的参数
//             break;




//        case 2:Max_speed += 5*delta;
//            break;
//
//        case 3:Base_speed += 1*delta;
//            break;
        case 1:motor_enable += 1*delta;break;

        case 2:Max_speed    += 10*delta;break;
//        case 3:Base_speed_R += 10*delta;break;
//        case 4:Base_speed_L += 10*delta;break;
        case 3:Base_speed   += 10*delta;break;
//        case 8:BASE_p_L += 1*delta;break;
//        case 9:BASE_p_R += 1*delta;break;
        case 4:Lp_Base += 1*delta;break;
        case 5:balance_location_pid.kd += 0.1*delta;break;
        case 6:light += 25*delta;mt9v03x_set_exposure_time(light);break;
//      case 5:Expectroll_T += 0.5*delta;break;
//      case 6:Qianzhan += 1*delta;break;
//      case 7:t1_ms_cricle1_target += 25*delta;break;
    }

//    ips200_show_string(20,1*16,"Jump_Flag ");
//    ips200_show_uint(100,1*16,Jumpbarrier_Flag,5);
//
//    ips200_show_string(20,2*16,"left");
//    ips200_show_int(100,2*16,lora3a22_uart_transfer.joystick[1]/10,4);
//
//    ips200_show_string(20,3*16,"right ");
//    ips200_show_int(100,3*16,lora3a22_uart_transfer.joystick[3]/10,4);
 //   ips200_show_int(190,0*16,tim,4);
    //  if ((L_corner_angle1 > 100 && L_corner_angle1 < 110 && R_corner_angle1 > 100 && R_corner_angle1 < 110))
    //  ips200_show_string(150,2*16,"Onc"); ips200_show_float(190,2*16, once, 2, 2);
    //  ips200_show_string(150,3*16,"TIM"); ips200_show_float(190,3*16, time_single, 4, 1);

    ips200_show_string(150,0*16,"mode_stop"); ips200_show_float(190,0*16, mode_stop, 3, 1);
    ips200_show_string(150,1*16,"mode_stright"); ips200_show_float(190,1*16, mode_stright, 2, 2);
    ips200_show_string(150,2*16,"mode_back"); ips200_show_float(190,2*16, mode_back, 3, 1);
    ips200_show_string(150,3*16,"R_c"); ips200_show_float(190,3*16, R_corner_angle1, 3, 1);
    ips200_show_string(150,4*16,"st_f"); ips200_show_float(190,4*16, Check_straight_flag,2, 1);
    ips200_show_string(150,5*16,"Cross"); ips200_show_float(190,5*16, Crossroad_memory, 2, 1);
    ips200_show_string(150,6*16,"a_L_m"); ips200_show_float(190,6*16, annulus_L_memory, 2, 1);

    ips200_show_string(150,8*16,"Anual"); ips200_show_float(190,8*16, Anu_time, 4, 1);

    ips200_show_string(150,9*16,"XZAI"); ips200_show_float(190,9*16, Check_obstacle_flag, 2, 1);
    ips200_show_string(150,7*16,"ba_time"); ips200_show_float(190,7*16, barrier_time, 3, 1);
    ips200_show_string(150,10*16,"Jump"); ips200_show_float(190,10*16, hengduan_flag, 2, 1);
    ips200_show_string(150,11*16,"dl1b");ips200_show_float(190,11*16, dl1b_distance_mm, 4, 1);


    ips200_show_string(20,7*16,"Left_st"); ips200_show_float(100,7*16, Left_straight_flag, 4, 1);
    ips200_show_string(20,8*16,"a_R_me"); ips200_show_float(100,8*16, annulus_R_memory, 4, 1);
    ips200_show_string(20,9*16,"Le_D"); ips200_show_float(100,9*16, Length_obstacle, 4, 1);
    ips200_show_string(20,10*16,"Le_al"); ips200_show_float(100,10*16, Length_annual, 4, 1);
    ips200_show_string(20,11*16,"Jump_L"); ips200_show_float(100,11*16, Length_Hengduan, 4, 1);




    ips200_show_string(20,1*16,"m_e ");
    ips200_show_uint(100,1*16,motor_enable,5);

    ips200_show_string(20,2*16,"STTSPEED");
    ips200_show_float(100,2*16,Max_speed,3,2);

//    ips200_show_string(20,3*16,"RightSPEED  ");
//    ips200_show_float(100,3*16, Base_speed_R,3,2);
//
//    ips200_show_string(20,4*16,"LeftSPEED  ");
//    ips200_show_float(100,4*16, Base_speed_L,3,2);

    ips200_show_string(20,3*16,"ElSpeed ");
    ips200_show_float(100,3*16,Base_speed,3,2);



//    ips200_show_string(20,8*16,"Left_P");
//    ips200_show_float(100,8*16,BASE_p_L,3,2);
//
//    ips200_show_string(20,9*16,"Right_P");
//    ips200_show_float(100,9*16, BASE_p_R,3,2);

    ips200_show_string(20,4*16,"Else_P");
    ips200_show_float(100,4*16, Lp_Base,3,2);



    ips200_show_string(20,5*16,"bpid.kd");
    ips200_show_float(100,5*16,balance_location_pid.kd,3,2);


    ips200_show_string(20,6*16,"light");
    ips200_show_float(100,6*16,light,3,2);
//    ips200_show_string(20,5*16,"yaw ");
//    ips200_show_float(100,5*16,QEKF_INS.Yaw,3,2);
//

//
//    ips200_show_string(20,7*16,"t1_ms_target ");
//    ips200_show_float(100,7*16,t1_ms_cricle1_target,3,2);

//    ips200_show_string(20,2*16,"MaxSpeed ");
//    ips200_show_float(100,2*16,Max_speed,3,2);
//
//    ips200_show_string(20,3*16,"BaseSpeed ");
//    ips200_show_float(100,3*16,Base_speed,3,2);


//    ips200_show_string(20,5*16,"zero ");
//    ips200_show_float(100,5*16,Expectroll_T,3,2);

//    ips200_show_string(20,6*16,"Qianzhan ");
//    ips200_show_float(100,6*16,Qianzhan,3,2);

//    ips200_show_string(20,7*16,"Ramp_sudu_en ");
//    ips200_show_int(100,7*16,Ramp_sudu_en,4);
    ips200_show_gray_image(0, 14*16, imag, image_w, image_h, image_w, image_h, Threshold1);
    for (i =Endline ; i < image_h-1; i++)
    {
        if(right[i]>188)right[i]=188;
        if(right[i]<0)right[i]=0;

        if(left[i]>188)left[i]=188;
        if(left[i]<0)left[i]=0;

        if(middle[i]>188)middle[i]=185;
        if(middle[i]<0)middle[i]=0;

        ips200_draw_point(middle[i], i+14*16,RGB565_GREEN);//显示起点 显示中线
        ips200_draw_point(left[i]   , i+14*16,RGB565_RED);//显示起点 显示左边线
        ips200_draw_point(right[i]   , i+14*16,RGB565_PURPLE );//显示起点 显示右边线
    }

//    ips200_show_gray_image(0, 8*16, mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, Threshold);
//
//    for(int i=0;i<MT9V03X_H;i++)
//    {
//        ips200_draw_point(zx[i], i+8*16, RGB565_RED);
//        ips200_draw_point(yx[i], i+8*16, RGB565_GREEN);
//        ips200_draw_point(wx[i], i+8*16, RGB565_BLUE);
////        ips200_draw_point(L_upcorner_col+5, i, RGB565_YELLOW);
////        ips200_draw_point(R_upcorner_col-5, i, RGB565_YELLOW);
//    }
//    for(int i=0;i<MT9V03X_W;i++)
//    {
//        ips200_draw_point(i, Finnalline+8*16, RGB565_YELLOW);
//    }

}
//第七页
void page8(void)
{

    ips200_show_string(0,0,"current page8");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 1:time_single_time += 100*delta;break;
        case 2:ob_err += 1*delta;break;
        case 3:ob_err_time += 10*delta;break;
        case 4:you_err += 1*delta;break;
//        case 1:light += 100*delta;mt9v03x_set_exposure_time(light);break;
//        case 2:tof_distance += 50*delta;break;
    }
//    ips200_show_string(20,1*16,"L_circle_corner_flag ");
//    ips200_show_float(100,1*16,L_circle_corner_flag,3,2);
//
//    ips200_show_string(20,2*16,"R_circle_corner_flag ");
//    ips200_show_float(100,2*16,R_circle_corner_flag,3,2);
//
//    ips200_show_string(20,3*16,"circle_mode ");
//    ips200_show_float(100,3*16,circle_mode,3,2);
//    ips200_show_float(190,0*16, Lower_left_inflection_X, 3, 3);
//    ips200_show_float(190,1*16, zhonxianpiancha, 3, 3);
    ips200_show_string(20,1*16,"DAN_TIME");
    ips200_show_float(120,1*16,time_single_time,3,2);

    ips200_show_string(20,2*16,"ob_err");
    ips200_show_float(120,2*16,ob_err,3,2);

    ips200_show_string(20,3*16,"ob_err_time");
    ips200_show_float(120,3*16,ob_err_time,3,2);

    ips200_show_string(20,4*16,"you_err");
    ips200_show_float(120,4*16,you_err,3,2);

    ips200_show_string(20,5*16,"dl1b");
    ips200_show_float(120,5*16, dl1b_distance_mm, 4, 1);

    ips200_show_string(20,6*16,"PO_wi");
    ips200_show_float(120,6*16, HALF_WIDTH, 3, 1);

    ips200_show_string(20,7*16,"PO_LE");
    ips200_show_float(120,7*16, Length_ramp, 5, 1);

    ips200_show_string(20,8*16,"PO_fLAG");
    ips200_show_float(120,8*16, flag1, 2, 1);
//ips200_show_string(20,2*16,"Lower_left_inflection_Flag=1 ");
//ips200_show_float(120,2*16,Lower_left_inflection_Flag,3,2);
//
//ips200_show_string(20,3*16,"Right_straight_flag=1 ");
//ips200_show_float(120,3*16,Right_straight_flag,3,2);
//
//ips200_show_string(20,4*16,"imag==255 ");
//ips200_show_float(100,4*16,imag[Lower_left_inflection_Y][2],3,2);
//
//ips200_show_string(20,5*16,"length=78 ");
//ips200_show_float(100,5*16,length,3,2);
//
//ips200_show_string(20,6*16,"L_Flag ");
//ips200_show_float(100,6*16,annulus_L_Flag,3,2);
//
//ips200_show_string(20,7*16,"1-7 ");
//ips200_show_float(100,7*16,annulus_L_memory,3,2);
//
//ips200_show_string(20,8*16,"Check_straight_flag");
//ips200_show_float(100,8*16,Check_straight_flag,3,1);
//
//ips200_show_string(20,9*16,"Check_right_flag");
//ips200_show_float(100,9*16,Check_right_flag,3,1);
//
//ips200_show_string(20,10*16,"Check_left_flag");
//ips200_show_float(100,10*16,Check_left_flag,3,1);

//ips200_show_string(20,8*16,"circle_mode ");
//ips200_show_float(100,8*16, circle_mode,3,2);
//
//ips200_show_string(20,9*16,"left_circle_flag ");
//ips200_show_float(100,9*16,left_circle_flag,3,2);
//
//ips200_show_string(20,10*16,"right_circle_flag ");
//ips200_show_float(100,10*16, right_circle_flag,3,2);

ips200_show_gray_image(0, 11*16, imag, image_w, image_h, image_w, image_h, Threshold1);
for (i =Endline ; i < image_h-1; i++)
{
    if(right[i]>188)right[i]=188;
    if(right[i]<0)right[i]=0;

    if(left[i]>188)left[i]=188;
    if(left[i]<0)left[i]=0;

    if(middle[i]>188)middle[i]=185;
    if(middle[i]<0)middle[i]=0;

    ips200_draw_point(middle[i], i+11*16,RGB565_GREEN);//显示起点 显示中线
    ips200_draw_point(left[i]   , i+11*16,RGB565_RED);//显示起点 显示左边线
    ips200_draw_point(right[i]   , i+11*16,RGB565_PURPLE );//显示起点 显示右边线
}
//for(int i=0;i<MT9V03X_W;i++)
//{
//    ips200_draw_point(i, Finnalline+8*16, RGB565_YELLOW);
//}
//    ips200_show_string(190,0*16,"zebra");
//    ips200_show_int(190,1*16,dl1b_distance_mm,4);
//    ips200_show_int(190,2*16,Ramp_mode,3);

//    ips200_show_uint(190,1*16,Boundry_Start_Left,3);
//    ips200_show_uint(190,2*16,Boundry_Start_Right,3);
//      ips200_show_float (190, 9*16, Err, 3, 2);
//      ips200_show_float (190, 10*16, Cross_mode, 3, 2);
 //   ips200_show_string(190,3*16,"cross");
 //   ips200_show_int(190,4*16,L_corner_flag,3);
 //   ips200_show_int(190,5*16,R_corner_flag,3);
 //   ips200_show_int(190,6*16,L_upcorner_flag,3);
 //   ips200_show_int(190,7*16,R_upcorner_flag,3);

//    ips200_show_int(190,9*16,circle_flag,3);
//    ips200_show_int(190,10*16,Cross_Flag,3);
//    ips200_show_int(190,11*16,Zebra_Flag,3);
//    ips200_show_int(190,12*16,Ramp_Flag,3);
//    ips200_show_int(190,13*16,Barrier_Flag,3);
//    ips200_show_int(190,14*16,SingleBridge_Flag,3);


}
//第二页
void page2(void)
{
    ips200_show_string(0,0,"current page2");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 1:Lp_Cross += 0.01*delta;break;
        case 2:Lp_Circle+= 0.01*delta;break;
        case 3:Lp_Ramp  += 0.01*delta;break;
        case 4:Lp_Ramp_End += 0.01*delta;break;
        case 5:Lp_Base += 0.005*delta;break;
    }

    ips200_show_string(20,1*16,"Lp_Cross ");
    ips200_show_float(100,1*16,Lp_Cross,2,3);
    ips200_show_string(20,2*16,"Lp_Circle ");
    ips200_show_float(100,2*16,Lp_Circle,2,3);
    ips200_show_string(20,3*16,"Lp_Ramp ");
    ips200_show_float(100,3*16,Lp_Ramp,2,3);
    ips200_show_string(20,4*16,"Lp_Ramp_End");
    ips200_show_float(100,4*16,Lp_Ramp_End,2,3);
    ips200_show_string(20,5*16,"Lp_Base ");
    ips200_show_float(100,5*16,Lp_Base,2,3);



//    ips200_show_gray_image(0, 6*16, mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, Threshold);//
//    for(int i=0;i<MT9V03X_H;i++)
//    {
//        ips200_draw_point(zx[i], i+96, RGB565_RED);
//        ips200_draw_point(yx[i], i+96, RGB565_GREEN);
//        ips200_draw_point(wx[i], i+96, RGB565_BLUE);
////        ips200_draw_point(L_upcorner_col+5, i+96, RGB565_YELLOW);
////        ips200_draw_point(R_upcorner_col-5, i+96, RGB565_YELLOW);
//    }
//    for(int i=0;i<MT9V03X_W;i++)
//    {
//        ips200_draw_point(i, Finnalline+96, RGB565_YELLOW);
//    }

}

//第三页
void page3(void)
{

    ips200_show_string(0,0,"current page3");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 1:Ap_Cross += 0.005*delta;break;
        case 2:Ap_Circle_Mode1 += 0.005*delta;break;
        case 3:Ap_Circle += 0.005*delta;break;
        case 4:Ap_Ramp += 0.005*delta;break;
        case 5:Ap_Base += 0.005*delta;break;
    }
    ips200_show_string(20,1*16,"Ap_Cross ");
    ips200_show_float(100,1*16,Ap_Cross,2,4);
    ips200_show_string(20,2*16,"Ap_Circle_M1 ");
    ips200_show_float(100,2*16,Ap_Circle_Mode1,2,4);
    ips200_show_string(20,3*16,"Ap_Circle ");
    ips200_show_float(100,3*16,Ap_Circle,2,4);
    ips200_show_string(20,4*16,"Ap_Ramp ");
    ips200_show_float(100,4*16,Ap_Ramp,2,4);
    ips200_show_string(20,5*16,"Ap_Base ");
    ips200_show_float(100,5*16,Ap_Base,2,4);
//    ips200_show_gray_image(0, 0, mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);//Threshold
}

//第四页
void page4(void)
{

    ips200_show_string(0,0,"current page4");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 3:Circle_enable += 1*delta;break;
        case 4:Cross_enable += 1*delta;break;
        case 5:Ramp_enable += 1*delta;break;
        case 6:Zebra_enable += 1*delta;break;
        case 7:Ramp_sudu_en += 1*delta;break;
    }
    ips200_show_string(20,1*16,"a ");
    ips200_show_int(100,1*16,momentumwheel_encoder_forward,4);
    ips200_show_string(20,2*16,"b ");
    ips200_show_int(100,2*16,momentumwheel_encoder_after,4);

    ips200_show_string(20,3*16,"Circle_enable ");
    ips200_show_int(100,3*16,Circle_enable,4);

    ips200_show_string(20,4*16,"Cross_enable ");
    ips200_show_int(100,4*16,Cross_enable,4);

    ips200_show_string(20,5*16,"Ramp_enable ");
    ips200_show_int(100,5*16,Ramp_enable,4);

    ips200_show_string(20,6*16,"Zebra_enable ");
    ips200_show_int(100,6*16,Zebra_enable,4);

    ips200_show_string(20,7*16,"Ramp_sudu_en ");
    ips200_show_int(100,7*16,Ramp_sudu_en,4);

}

//第五页
void page5(void)
{

    ips200_show_string(0,0,"current page5");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 1:Time_1 += 100*delta;break;
        case 2:Time_2 += 100*delta;break;
        case 3:Time_Other += 100*delta;break;
        case 4:LeftCircleLine_1 += 5*delta;break;
        case 5:LeftCircleLine_4 += 5*delta;break;
        case 6:RightCircleLine_1 += 5*delta;break;
        case 7:RightCircleLine_4 += 5*delta;break;
    }
    ips200_show_string(20,1*16,"Time_1 ");
    ips200_show_int(100,1*16,Time_1,4);

    ips200_show_string(20,2*16,"Time_2 ");
    ips200_show_int(100,2*16,Time_2,4);

    ips200_show_string(20,3*16,"Time_Other ");
    ips200_show_int(100,3*16,Time_Other,4);

    ips200_show_string(20,4*16,"L_1 ");
    ips200_show_int(100,4*16,LeftCircleLine_1,4);

    ips200_show_string(20,5*16,"L_4 ");
    ips200_show_int(100,5*16,LeftCircleLine_4,4);

    ips200_show_string(20,6*16,"R_1 ");
    ips200_show_int(100,6*16,RightCircleLine_1,4);

    ips200_show_string(20,7*16,"R_4 ");
    ips200_show_int(100,7*16,RightCircleLine_4,4);
}

//第六页
void page6(void)
{

    ips200_show_string(0,0,"current page6");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
    }

}

//第七页
void page7(void)
{

    ips200_show_string(0,0,"current page7");
    ips200_show_string(0,row_index*16,"->");
    switch(row_index)
    {
        case 1:Circle_speed += 1*delta;break;
        case 2:Ramp_speed += 1*delta;break;
        case 3:Ramp_End_speed += 1*delta;break;


    }
    ips200_show_string(20,1*16,"Circle_speed ");
    ips200_show_float(100,1*16,Circle_speed,3,2);

    ips200_show_string(20,2*16,"Ramp_speed ");
    ips200_show_float(100,2*16,Ramp_speed,3,2);

    ips200_show_string(20,3*16,"Ramp_E_speed ");
    ips200_show_float(100,3*16,Ramp_End_speed,3,2);


//    ips114_show_binary_image (0, 0, image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H);
}



/*-------------------------------------------------------------------------------------------------------------------
  @brief     显示初始化
  @note
-------------------------------------------------------------------------------------------------------------------*/
void disply_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(0);
    ips200_set_color(RGB565_39C5BB, RGB565_BLACK);
    ips200_init(IPS200_TYPE);
//    gpio_init(P11_10, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(P11_11, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(P11_2, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(P11_3, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(P11_9, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_init(P20_6, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P20_7, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P11_2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P11_3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P20_9, GPI, GPIO_HIGH, GPI_PULL_UP);
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     数据读取
  @note
-------------------------------------------------------------------------------------------------------------------*/
void data_read()
{
    flash_read_page_to_buffer(0, 5);           // 将数据从 flash 读取到缓冲区
    //第一页
    Expectroll_T = flash_union_buffer[0].float_type;
    Max_speed =     flash_union_buffer[1].float_type;
    Base_speed = flash_union_buffer[2].float_type ;
    Qianzhan =     flash_union_buffer[3].float_type;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     数据写入
  @note
-------------------------------------------------------------------------------------------------------------------*/
void data_write()
{
    if(flash_check(0, 5))                      // 判断是否有数据
        flash_erase_page(0, 5);                // 擦除这一页
        flash_buffer_clear();
        //第一页
        flash_union_buffer[0].float_type = Expectroll_T;
        flash_union_buffer[1].float_type = Max_speed;
        flash_union_buffer[2].float_type = Base_speed;
        flash_union_buffer[3].float_type = Qianzhan;
        //第二页

        flash_write_page_from_buffer(0, 5);
}





