#include "zf_common_headfile.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"
#include "swj.h"
#include "dht11.h"

// 【新增】包含 DL1B TOF 传感器的头文件
#include "zf_device_dl1b.h"

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
uint8 k230_fire_flag = 0; // 0代表无火，1代表有火

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

// 增加全局变量保存温湿度
uint8 env_temperature = 0;
uint8 env_humidity = 0;
uint16 dht11_timer_cnt = 0; // 计时器
extern uint16 real_dht11_timer;

int core0_main(void)
{
    clock_init();
    debug_init();
    // 此处编写用户代码 例如外设初始化代码等
    //disply_init();
    pid_all_init();
    imu660ra_init();
    Attitude_Init();
    SERVO_init();
    small_driver_uart_init();
    pit_ms_init(CCU61_CH1,1);
    
    uart_receiver_init();
    uart_init(UART_0, 115200, UART0_TX_P15_2, UART0_RX_P15_3);
    
    if(DHT11_Init() == 0) {
        printf("DHT11 Init Success!\r\n");
    } else {
        printf("DHT11 Init Failed! Check Wiring.\r\n");
    }
    
    // =========================================================
    // 【新增】初始化 DL1B TOF 传感器
    // =========================================================
    if(dl1b_init() == 0) {
        printf("DL1B TOF Init Success!\r\n");
    } else {
        printf("DL1B TOF Init Failed! Check Wiring.\r\n");
    }

    cpu_wait_event_ready();

    // 用于解析 K230 数据帧的状态机变量
    uint8 recv_dat = 0;
    static uint8 rx_state = 0;
    static uint8 rx_data = 0;
    
    // 【新增】用于 TOF 传感器的非阻塞打印计数器
    static uint32 tof_print_cnt = 0;

    while (1)
    {
        // =========================================================
        // 【新增】TOF 数据读取与打印测试
        // =========================================================
        tof_print_cnt++;
        if(tof_print_cnt >= 3000) // 粗略延时，大约 100~300ms 读一次，防止刷屏卡死
        {
            tof_print_cnt = 0;
            dl1b_get_distance(); // 调用库函数获取最新距离
            
            if(dl1b_finsh_flag == 1) // 确保获取到的数据是有效的
            {
                // 打印距离数据到串口助手，单位：毫米(mm)
                printf("TOF Distance: %d mm\r\n", dl1b_distance_mm);
            }
        }

        // =========================================================
        // 1. 从 UART_0 读取 K230 数据
        // =========================================================
        while (uart_query_byte(UART_0, &recv_dat))
        {
            if (rx_state == 0 && recv_dat == 0xA5) {
                rx_state = 1;
            } else if (rx_state == 1) {
                rx_data = recv_dat;
                rx_state = 2;
            } else if (rx_state == 2) {
                if (recv_dat == 0x5A) {
                    k230_fire_flag = rx_data;

                    // 【调试必看】如果收到了，就在你的 Debug 串口打印出来
                    if(k230_fire_flag == 1) {
                        printf(">>> ALERT: FIRE DETECTED! <<<\r\n");
                    }
                }
                rx_state = 0;
            } else {
                rx_state = 0;
            }
        }
        
        // =========================================================
        // 2. 温湿度真实定时读取逻辑
        // =========================================================
        if (real_dht11_timer >= 2000) 
        {
            real_dht11_timer = 0; 
            
            if(DHT11_Read_Data(&env_temperature, &env_humidity) == 0) 
            {
                printf("Real-Time Temp: %d C, Humi: %d %%\r\n", env_temperature, env_humidity);
            }
            else
            {
                printf("DHT11 Checksum Error or No Response.\r\n");
            }
        }
        
        // =========================================================
        // 3. 独立按键启动逻辑
        // =========================================================
        if (gpio_get_level(P20_9) == 0) 
        {
            system_delay_ms(20);        
            if (gpio_get_level(P20_9) == 0) 
            {
                motor_enable = 1;       
                while(gpio_get_level(P20_9) == 0); 
            }
        }

        // =========================================================
        // 4. 遥控器解析逻辑
        // =========================================================
        if (1 == uart_receiver.finsh_flag)
        {
            if (0 == uart_receiver.state)
            {
                rc.disconnect_cnt = 0; 
                if (rc.error_print_flag == 1) {
                    printf("[RC_EVENT] Remote Reconnected!\r\n");
                    rc.error_print_flag = 0;
                }

                uint16_t ch1_dir = uart_receiver.channel[0]; 
                uint16_t ch2_thr = uart_receiver.channel[1]; 
                
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

                if (cur_ch3 != rc.ch3_state) {
                    rc.ch3_state = cur_ch3;
                    printf("[RC] 通道3 按键改变! 当前: %d\r\n", rc.ch3_state);
                }

                if (cur_ch4_mode != rc.ch4_mode) {
                    rc.ch4_mode = cur_ch4_mode; 
                    printf("[RC] 通道4 三段开关切换! 当前模式: %d\r\n", rc.ch4_mode);
                }

                if (cur_ch5 != rc.ch5_state) 
                {
                    rc.ch5_state = cur_ch5;
                    if (cur_ch5 == 1) 
                    {
                        SingleBridge_mode = 1;
                        printf("[RC_BRIDGE] 通道5 开启: 进入单边桥模式!\r\n");
                    } 
                    else 
                    {
                        SingleBridge_mode = 0;
                        balance_rollangle_pi_init(); 
                        printf("[RC_BRIDGE] 通道5 关闭: 退出单边桥模式，PID已洗白!\r\n");
                    }
                }

                if (cur_ch6 != rc.ch6_state) 
                {
                    rc.ch6_state = cur_ch6; 
                    if (cur_ch6 == 1) 
                    {
                        hengduan_flag = 1;
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
                        hengduan_flag = 0;
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
                rc.disconnect_cnt++;
                if (rc.disconnect_cnt > 20) 
                {
                    if (rc.error_print_flag == 0) {
                        printf("[RC_ERROR] Remote Disconnected!\r\n"); 
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