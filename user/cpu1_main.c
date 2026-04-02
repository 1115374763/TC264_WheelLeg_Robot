#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
int Target_Err_time;
extern int time_single_1;
extern int time_single_an;

// 声明我们在 core0 或者 common 中定义的全局变量
extern float remote_dir_err; 

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    
    while (TRUE)
    {
        // 将固定的 -0 替换为遥控器动态下发的方向偏差
        location_banlance_error = balance_location_PID(remote_dir_err);
    }
}
#pragma section all restore