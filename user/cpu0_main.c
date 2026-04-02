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

/////
while (1)
    {
        if (1 == uart_receiver.finsh_flag)
        {
            // 0 表示正常连接
            if (0 == uart_receiver.state)
            {
                // 【新增】连接恢复逻辑
                rc.disconnect_cnt = 0; // 只要有一帧正常，清零断连计数
                if (rc.error_print_flag == 1) {
                    printf("[RC_EVENT] Remote Reconnected!\r\n");
                    rc.error_print_flag = 0;
                }

                uint16_t ch1_dir = uart_receiver.channel[0]; 
                uint16_t ch2_thr = uart_receiver.channel[1]; 
                
                uint8_t cur_ch3 = (uart_receiver.channel[2] > RC_MID_VAL) ? 1 : 0;
                uint8_t cur_ch5 = (uart_receiver.channel[4] > RC_MID_VAL) ? 1 : 0;
                uint8_t cur_ch6 = (uart_receiver.channel[5] > RC_MID_VAL) ? 1 : 0;

                uint8_t cur_ch4_mode = 2; 
                if (uart_receiver.channel[3] < 600)       
                    cur_ch4_mode = 1; 
                else if (uart_receiver.channel[3] > 1400) 
                    cur_ch4_mode = 3; 
                else                                      
                    cur_ch4_mode = 2; 

                // 差值过滤打印
                if (ABS_DIFF(ch1_dir, rc.last_ch1) > 30 || ABS_DIFF(ch2_thr, rc.last_ch2) > 30) {
                    printf("CH1(DIR): %04d | CH2(THR): %04d\r\n", ch1_dir, ch2_thr);
                    rc.last_ch1 = ch1_dir;
                    rc.last_ch2 = ch2_thr;
                }

                // 状态改变触发打印
                if (cur_ch3 != rc.ch3_state) {
                    rc.ch3_state = cur_ch3;
                    printf("[RC] 通道3 按键改变! 当前: %d\r\n", rc.ch3_state);
                }
                
                if (cur_ch5 != rc.ch5_state) {
                    rc.ch5_state = cur_ch5;
                    printf("[RC] 通道5 按键改变! 当前: %d\r\n", rc.ch5_state);
                }
                
                if (cur_ch6 != rc.ch6_state) {
                    rc.ch6_state = cur_ch6;
                    printf("[RC] 通道6 按键改变! 当前: %d\r\n", rc.ch6_state);
                }

                if (cur_ch4_mode != rc.ch4_mode) {
                    rc.ch4_mode = cur_ch4_mode; 
                    printf("[RC] 通道4 三段开关切换! 当前模式: %d\r\n", rc.ch4_mode);
                }
            }
            else
            {
                // 【新增】容错抗干扰逻辑：连续 20 帧（大约 0.5 秒）异常才判定为断开
                rc.disconnect_cnt++;
                if (rc.disconnect_cnt > 20) 
                {
                    if (rc.error_print_flag == 0) {
                        printf("[RC_ERROR] Remote Disconnected!\r\n"); 
                        rc.error_print_flag = 1;
                    }
                    rc.disconnect_cnt = 20; // 防止数值溢出
                }
            }
            
            uart_receiver.finsh_flag = 0;
        }
    }
}


#pragma section all restore
// **************************** 代码区域 ****************************


