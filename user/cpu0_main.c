#include "zf_common_headfile.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"
#include "swj.h"
#pragma section all "cpu0_dsram"
extern int single_mode;
int mode_stop,mode_stright,mode_back;
#define ABS_DIFF(a, b) ((a) > (b) ? ((a) - (b)) : ((b) - (a)))
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
#define RC_MID_VAL 1000
// 定义全局方向误差变量
float remote_dir_err = 0.0f;
typedef struct {
    uint8_t ch3_state;
    uint8_t ch5_state;
    uint8_t ch6_state;
    uint8_t ch4_mode;
    
    uint16_t last_ch1;
    uint16_t last_ch2;
    uint8_t  error_print_flag; // 用于防止失控提示刷屏
    uint8_t  disconnect_cnt;
} RC_State_t;
RC_State_t rc = {0};
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

while (1)
    {
        if (1 == uart_receiver.finsh_flag)
        {
            if (0 == uart_receiver.state)
            {
                // 连接恢复逻辑
                rc.disconnect_cnt = 0; 
                if (rc.error_print_flag == 1) {
                    printf("[RC_EVENT] Remote Reconnected!\r\n");
                    rc.error_print_flag = 0;
                }

                uint16_t ch1_dir = uart_receiver.channel[0]; 
                uint16_t ch2_thr = uart_receiver.channel[1]; 
                
                // =========================================================
                // 【新增】通道1 (方向) 转向映射控制
                // 引入比例系数 turn_ratio，用于调节打满方向盘时小车的转向急缓
                // =========================================================
                float turn_ratio = 0.05f; // <--- 如果觉得转向太慢就调大，太快就调小

                if (ch1_dir > 1050)      // 摇杆左打 (1050 ~ 1800)
                {
                    // (1050 - 大于1050的数) = 负数，满足左转为负的要求
                    remote_dir_err = (1050 - ch1_dir) * turn_ratio; 
                }
                else if (ch1_dir < 950)  // 摇杆右打 (200 ~ 950)
                {
                    // (950 - 小于950的数) = 正数，满足右转为正的要求
                    remote_dir_err = (950 - ch1_dir) * turn_ratio;  
                }
                else                     // 回中死区
                {
                    remote_dir_err = 0.0f;
                }

                // =========================================================
                // 通道2 (油门) 速度状态控制
                // =========================================================
                if (ch2_thr >= 950 && ch2_thr <= 1050)
                {
                    mode_stop = 1;
                    mode_stright = 0;
                    mode_back = 0;
                }
                else if (ch2_thr > 1050) 
                {
                    mode_stop = 0;
                    mode_stright = 1;
                    mode_back = 0;
                }
                else if (ch2_thr < 950) 
                {
                    mode_stop = 0;
                    mode_stright = 0;
                    mode_back = 1;
                }

                // 其他按键和状态打印逻辑保持不变...
                // ...
            }
            else
            {
                // 断连保护逻辑...
                rc.disconnect_cnt++;
                if (rc.disconnect_cnt > 20) 
                {
                    if (rc.error_print_flag == 0) {
                        printf("[RC_ERROR] Remote Disconnected!\r\n"); 
                        // 失控时同时清除方向和速度
                        mode_stop = 1;
                        mode_stright = 0;
                        mode_back = 0;
                        remote_dir_err = 0.0f; 
                        
                        rc.error_print_flag = 1;
                    }
                    rc.disconnect_cnt = 20; 
                }
            }
            uart_receiver.finsh_flag = 0;
        }
    }
}


#pragma section all restore
// **************************** 代码区域 ****************************


