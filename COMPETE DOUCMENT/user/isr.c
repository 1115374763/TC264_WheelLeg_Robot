#include "isr_config.h"
#include "zf_device_dot_matrix_screen.h"
#include "zf_device_tld7002.h"
#include "isr.h"
#include "zf_common_headfile.h"
float Balance_Target_Angle = 12.0f;
unsigned char balance_servo_pid_cnt = 0;
unsigned char balance_encoder_pid_cnt = 0;
unsigned char balance_velocity_pid_cnt = 0;
unsigned char balance_angle_p_cnt = 0;
unsigned char balance_gyro_pd_cnt = 0;
unsigned char balance_gyro_weizhi_pd_cnt = 0;
unsigned char balance_location_pid_cnt = 0;
unsigned char Gyro_cnt = 0;
int bridge_count=0;
int bridge_count_2=0;
int bridge_pri=0;
int time_single_1=0;
int time_single_an=0;
float Exceptspeed = 0.0;//-19
extern uint16 t1_ms_cricle1_target;
extern int single_mode;extern int bridge_pri;
extern uint8 SingleBridge_mode;
float Encoder_pre=0;
int Db_count=0;
int Anu_time=0;
int barrier_time=0;
int anu_falg=0;
int16 Length_ramp = 0;
int16 Length_Hengduan = 0;
int16 Length_annual = 0;
int16 Length_obstacle = 0;
// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。


// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
//    Led_process();

    lora3a22_response_time++;
    if (lora3a22_response_time > 500 / 10)   //500ms 没有接受倒数据判断位发送端异常
    {
        lora3a22_state_flag = 0;                     //遥控器状态位清零
        lora3a22_response_time = 0;
    }
    pit_clear_flag(CCU60_CH0);


}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);




}



int xxxx=0;


IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套


    balance_encoder_pid_cnt++;

//****************************************************//
    if(balance_encoder_pid_cnt == 20)
    {
        momentumwheel_encoder_forward = motor_value.receive_left_speed_data;

        momentumwheel_encoder_after   = motor_value.receive_right_speed_data;



        momentumwheel_encoder_mid = (momentumwheel_encoder_forward - momentumwheel_encoder_after)/2.0;

        Encoder_pre *= 0.7;                                                       //===一阶低通滤波器
        Encoder_pre += momentumwheel_encoder_mid * 0.3;                           //===一阶低通滤波器

        momentumwheel_encoder_chasu = (float) (momentumwheel_encoder_after + momentumwheel_encoder_forward);


        if(mode_stop)
        {
            Exceptspeed=0;
        }

        if(mode_stright)
        {
            Exceptspeed=150;
        }

        if(mode_back)
        {
            Exceptspeed=-150;
        }


        balance_encoder_PD(Encoder_pre,Exceptspeed);



        balance_encoder_pid_cnt = 0;
    }
//****************************************************//
//****************************************************//
//****************************************************//
    if(motor_enable != 1 && stop_flag != 1)
     {
         Gyro_cnt++;
         if(Gyro_cnt == 1)
         {
             Attitude_Calculate();
             inverseKinematics();
             Gyro_cnt = 0;
         }
     }

//*****************这个都要motor==1才能触发***********************************//
     if(motor_enable == 1)
     {
         //计时变量//
         t_1s ++;
         t_2s ++;
         t_10s ++;
         //计时变量//
         balance_gyro_pd_cnt++;
         balance_angle_p_cnt++;
         balance_velocity_pid_cnt++;
         balance_location_pid_cnt++;
//         balance_encoder_pid_cnt++;
         balance_servo_pid_cnt++;
         Gyro_cnt++;



         inverseKinematics();




//*******************这个都要motor==1才能触发*********************************//


//******************这个都要motor==1才能触发**********************************//

         if(Check_obstacle_flag!=0)
         {
             barrier_time++;
         }
         if(barrier_time>ob_err_time)
         {
             barrier_time=0;
             Check_obstacle_flag = 0;
         }


//*********************这个都要motor==1才能触发*******************************//
         if(Gyro_cnt == 1)
         {
             Attitude_Calculate();
             Gyro_cnt = 0;
         }


         if(balance_gyro_pd_cnt == 2)
         {
             balance_gyro_weizhi_PD(QEKF_INS.Gyro[1],angle_banlance_error);//调直立环硬度
             balance_gyro_pd_cnt = 0;
         }
         if(balance_angle_p_cnt == 4)
         {
             angle_banlance_error = balance_angle_P(QEKF_INS.Pitch,Balance_Target_Angle);//Expectroll_T
             balance_angle_p_cnt = 0;
         }

         //摔倒保护//
//摔倒保护////摔倒保护////摔倒保护//
         if( (QEKF_INS.Pitch > 30 || QEKF_INS.Pitch < -35 || stop_flag == 1) &&  Jumpbarrier_Flag!=1  )//
         {

         }


     }
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
        // 下降沿到了 准备重新发送占空比数据并更新
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);
//        dot_matrix_screen_scan();

    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
//    tld7002_callback();
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数
}

// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
//    gnss_uart_callback();                           // GNSS串口回调函数
    uart_control_callback();



}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
