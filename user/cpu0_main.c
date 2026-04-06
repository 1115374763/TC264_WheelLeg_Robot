#include "zf_common_headfile.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"
#include "swj.h"
#pragma section all "cpu0_dsram"
extern int single_mode;
extern int hengduan_flag;
extern uint16 jumptime0, jumptime1, jumptime2, jumptime3, jumptime4;
extern int jump_out;
extern uint8 SingleBridge_mode;
extern void balance_rollangle_pi_init(void);
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
        // =========================================================
        // 独立按键启动逻辑
        // =========================================================
        if (gpio_get_level(P20_9) == 0) // 检测到按键按下 (低电平)
        {
            system_delay_ms(20);        // 软件消抖 (延时20ms，逐飞库自带此函数)
            if (gpio_get_level(P20_9) == 0) // 再次确认按键确实被按下了
            {
                motor_enable = 1;       // 启动电机标志位置 1             
                while(gpio_get_level(P20_9) == 0); // 松手检测：死循环等待直到按键松开，防止一直触发
            }
        }

        // =========================================================
        // 遥控器解析逻辑
        // =========================================================
        if (1 == uart_receiver.finsh_flag)
        {
            // 0 表示正常连接
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
                
                // ---------------------------------------------------------
                // 通道1 (方向) 转向映射控制
                // ---------------------------------------------------------
                float turn_ratio = 0.07f; 
                if (ch1_dir > 1050)      
                {
                    remote_dir_err = (1050 - ch1_dir) * turn_ratio; 
                }
                else if (ch1_dir < 950)  
                {
                    remote_dir_err = (950 - ch1_dir) * turn_ratio;  
                }
                else                     
                {
                    remote_dir_err = 0.0f;
                }

                // ---------------------------------------------------------
                // 通道2 (油门) 速度状态控制
                // ---------------------------------------------------------
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

                // ---------------------------------------------------------
                // 其他通道状态读取
                // ---------------------------------------------------------
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

                // ---------------------------------------------------------
                // 状态打印与通道触发逻辑
                // ---------------------------------------------------------
                if (cur_ch3 != rc.ch3_state) {
                    rc.ch3_state = cur_ch3;
                    printf("[RC] 通道3 按键改变! 当前: %d\r\n", rc.ch3_state);
                }
                
                // 【已删除】原有的纯打印 cur_ch5 检查，防止状态提前被刷新

                if (cur_ch4_mode != rc.ch4_mode) {
                    rc.ch4_mode = cur_ch4_mode; 
                    printf("[RC] 通道4 三段开关切换! 当前模式: %d\r\n", rc.ch4_mode);
                }

                // =========================================================
                // 【重构】通道5 单边桥模式开关控制
                // =========================================================
                if (cur_ch5 != rc.ch5_state) 
                {
                    rc.ch5_state = cur_ch5;
                    
                    if (cur_ch5 == 1) 
                    {
                        // 拨下开关：进入单边桥模式
                        SingleBridge_mode = 1;
                        printf("[RC_BRIDGE] 通道5 开启: 进入单边桥模式!\r\n");
                    } 
                    else 
                    {
                        // 拨回开关：关闭单边桥模式
                        SingleBridge_mode = 0;
                        
                        // 【核心安全逻辑】退出时立刻清空 Roll 环补偿的积分记忆
                        // 防止下次上桥瞬间，车子带着历史误差猛烈抽搐
                        balance_rollangle_pi_init(); 
                        
                        printf("[RC_BRIDGE] 通道5 关闭: 退出单边桥模式，PID已洗白!\r\n");
                    }
                }

                // =========================================================
                // 【核心】通道6 边沿检测跳跃逻辑 (单次触发，防止连跳)
                // =========================================================
                if (cur_ch6 != rc.ch6_state) 
                {
                    rc.ch6_state = cur_ch6; // 立即更新状态，防止重复触发
                    
                    if (cur_ch6 == 1) 
                    {
                        // 动作：0 -> 1 (拨下开关触发跳跃)
                        hengduan_flag = 1;
                        
                        // 清零所有状态机计时器，保证起跳干脆利落
                        jumptime0 = 0;
                        jumptime1 = 0;
                        jumptime2 = 0;
                        jumptime3 = 0;
                        jumptime4 = 0;
                        jump_out = 0;
                        
                        printf("[RC_JUMP] 通道6 开启: 触发单次跳跃!\r\n");
                    } 
                    else 
                    {
                        // 动作：1 -> 0 (拨回开关强制复位)
                        hengduan_flag = 0;
                        
                        // 同样清零计时器，为下一次跳跃做准备
                        jumptime0 = 0;
                        jumptime1 = 0;
                        jumptime2 = 0;
                        jumptime3 = 0;
                        jumptime4 = 0;
                        jump_out = 0;
                        
                        printf("[RC_JUMP] 通道6 关闭: 跳跃系统已复位!\r\n");
                    }
                }
            }
            else
            {
                // ---------------------------------------------------------
                // 断连保护逻辑
                // ---------------------------------------------------------
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