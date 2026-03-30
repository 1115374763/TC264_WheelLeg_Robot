/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_common_headfile
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

//===================================================C语言 函数库===================================================
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
//===================================================C语言 函数库===================================================

//===================================================芯片 SDK 底层===================================================
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//===================================================芯片 SDK 底层===================================================

//====================================================开源库公共层====================================================
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
#include "isr_config.h"
//====================================================开源库公共层====================================================

//===================================================芯片外设驱动层===================================================
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
//===================================================芯片外设驱动层===================================================

//===================================================外接设备驱动层===================================================
#include "zf_device_absolute_encoder.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_gnss.h"
#include "zf_device_camera.h"
#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"
#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu963ra.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include "zf_device_mpu6050.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_oled.h"
#include "zf_device_ov7725.h"
#include "zf_device_scc8660.h"
#include "zf_device_tft180.h"
#include "zf_device_tsl1401.h"
#include "zf_device_type.h"
#include "zf_device_uart_receiver.h"
#include "zf_device_virtual_oscilloscope.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_wireless_uart.h"
//===================================================外接设备驱动层===================================================

//====================================================应用组件层====================================================
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
//====================================================应用组件层====================================================

//=====================================================用户层======================================================
#include "balance.h"
#include "pid.h"
#include "momentumwheel.h"
#include "disply.h"
#include "swj.h"
#include "servo.h"
#include "camera.h"
#include "element.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "matrix.h"
#include "EKF_Platform.h"
#include "MenuElement.h"
#include "small_driver_uart_control.h"
#include "vmc.h"
#include "zf_device_lora3a22.h"
#include "image.h"
#include "isr.h"
//=====================================================用户层======================================================
extern uint8 Cross_mode;//十字判断
extern float location_banlance_error;
extern double zhonxianpiancha;
extern float Base_speed_L;    //基础速度
extern float Base_speed_R;    //基础速度
extern int HALF_WIDTH;
extern uint8 Check_left_flag;
extern uint8 Check_right_flag;
extern uint8 Check_straight_flag;
extern uint8 Threshold1;extern float BASE_p_L,BASE_p_R;
extern uint8 Right_straight_flag; //右直线
extern uint8 Left_straight_flag; //左直线
extern uint16 annulusltime;
extern int Anu_time;extern int anu_falg;
extern uint8 annulus_L_Flag;       //左圆环
extern uint8 annulus_R_Flag;       //右圆环
extern uint8 annulus_L_memory;     //左圆环计步
extern uint8 annulus_R_memory;     //右圆环计步
extern uint8 zebra_crossing_flag;//斑马线
extern uint8 zebra_flag_end;
extern uint8 Left_garage_flag; //左车库
extern uint8 Left_garage_memory;//左车库计步
extern int barrier_time;
extern int width;
extern int flag1;
extern int annulus_out;
extern uint16 jumptime ;
extern uint16 jumptime0 ;
extern uint16 jumptime1 ;
extern uint16 jumptime2 ;
extern uint16 jumptime3 ;
extern uint16 jumptime4 ;
//圆环凸起点
extern uint8 roundabout_X;
extern uint8 roundabout_Y;
extern uint8 roundabout_Flag;
extern int time_single_1;
extern int time_single_an;
extern int you_err;
//出环识别点
extern uint8 Exit_loop_X;
extern uint8 Exit_loop_Y;
extern uint8 Exit_loop_Flag;
extern int ob_err_F_small;// 小障碍专用
extern int ob_err_N_small;


extern uint8 Crossroad_Flag;      //十字
extern uint8 Crossroad_memory;     //十字计步
extern uint8 Finish_Flag; //处理完成标识位
//丢线
extern uint8 Lost_left_Flag;
extern uint8 Lost_right_Flag;
extern uint8 Lost_point_L_scan_line;
extern uint8 Lost_point_R_scan_line;
extern int once;
//左下拐点
extern uint8 Lower_left_inflection_X ;
extern uint8 Lower_left_inflection_Y ;
extern uint8 Lower_left_inflection_Flag;
extern float Exceptspeed ;//-19

//右下拐点
extern uint8 Lower_right_inflection_X ;
extern uint8 Lower_right_inflection_Y ;
extern uint8 Lower_right_inflection_Flag;

//左上拐点
extern uint8 Upper_left_inflection_X ;
extern uint8 Upper_left_inflection_Y ;
extern uint8 Upper_left_inflection_Flag;
extern uint8 Check_obstacle_flag;
extern int time_single_time;
extern int ob_err;
extern int ob_err_time;
//右上拐点
extern uint8 Upper_right_inflection_X ;
extern uint8 Upper_right_inflection_Y ;
extern uint8 Upper_right_inflection_Flag;
extern int once;
extern uint8 SingleBridge_Flag;
extern int time_single;
extern float R_corner_angle1;
extern float L_corner_angle1;
extern int hengduan_flag;
extern int annulus_L_3;
extern int podao_time;
extern int turnleft_flag;
extern int turnright_flag;
extern int jump_out;

extern int16 Length_Hengduan;
extern int16 Length_ramp;
extern int16 Length_annual;
extern int16 Length_obstacle;
extern float Encoder_pre;
extern int mode_stop,mode_stright,mode_back;
#endif

