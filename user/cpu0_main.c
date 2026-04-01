#include "zf_common_headfile.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"
#include "swj.h"
#pragma section all "cpu0_dsram"
extern int single_mode;
int mode_stop,mode_stright,mode_back;

// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// **************************** 代码区域 ****************************
int holderX = 0;
int core0_main(void)
{
    clock_init();
    debug_init();
    // 此处编写用户代码 例如外设初始化代码等
    disply_init();
    pid_all_init();
    imu660ra_init();
    Attitude_Init();
    SERVO_init();
    small_driver_uart_init();
    pit_ms_init(CCU61_CH1,1);
    uart_receiver_init();
    cpu_wait_event_ready();


    while (TRUE)
    {

        Menu();
        if(1 == uart_receiver.finsh_flag)                            // 帧完成标志判断
        {
            if(0 == uart_receiver.state)                             // 遥控器失控状态判断
            {
                printf("CH1-CH6 data: ");
                for(int i = 0; i < 6; i++)
                {
                    printf("%d ", uart_receiver.channel[i]);         // 串口输出6个通道数据
                }
                printf("\r\n");
            }
            else
            {
                printf("Remote control has been disconnected.\r\n"); // 串口输出失控提示
            }
            uart_receiver.finsh_flag = 0;                            // 帧完成标志复位
        }
        if(1 == uart_receiver.finsh_flag)                            // 帧完成标志判断
          {

           if(uart_receiver.channel[1]>=1000&&uart_receiver.channel[1]<1100)//控制电机速度，电机的值按照你实际的来改
           {
                mode_stop = 1 ;
                mode_stright=0;
                mode_back=0;
           }
            else  if (uart_receiver.channel[1]>=1100&&uart_receiver.channel[1]<=1800)
           {
                mode_stop = 0 ;
                mode_stright=1;
                mode_back=0;
           }
            else  if (uart_receiver.channel[1]>200&&uart_receiver.channel[1]<1000)
           {
                mode_stright = 0;
                mode_stop=0;
                mode_back=1;
           }

              uart_receiver.finsh_flag = 0;                            // 帧完成标志复位
          }

    }
}


#pragma section all restore
// **************************** 代码区域 ****************************


