#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
int Target_Err_time;
extern int time_single_1;
extern int time_single_an;


void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
               location_banlance_error = balance_location_PID(-0);//转向环
    }


}
#pragma section all restore
// **************************** 代码区域 ****************************
