#include "zf_common_headfile.h"
#ifndef __IMAGE_H_
#define __IMAGE_H_

extern int stop_time_flag;
struct ROAD_TYPE
{
     int8 straight;         //直道
     int8 bend;             //弯道
     int8 Ramp;             //坡道
     int8 Cross;            //十字
     int8 L_Cross;          //入左十字
     int8 R_Cross;          //入右十字
     int8 LeftCirque;       //左环岛
     int8 RightCirque;      //右环岛
     int8 Fork;             //三岔口
     int8 Barn_l_out;       //出左库
     int8 Barn_r_out;       //出右库
     int8 Barn_l_in;        //入左库
     int8 Barn_r_in;        //入右库
};
extern struct ROAD_TYPE road_type;
struct SPEED_TYPE
{
        int16 straight;
        int16 bend;
        int16 difficut_bend;
        int16 cross;
        int16 cirque;
        int16 fork;
        int16 lean_cross;
        int16 ramp;
        int16 stop;
        int16 barn;
};
extern struct SPEED_TYPE speed_type;

//宏定义
#define image_h 80//图像高度
#define image_w 188//图像宽度
extern uint8 Center;
extern uint8 Endline;
extern uint8 length;
extern uint8 Crossroad_Flag;
#define white_pixel 255
#define black_pixel 0
#define USE_num image_h*3
#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值
#define UP(row,col)     (imag[row][col]==0)&&(imag[row+1][col]!=0)&&(imag[row-1][col]==0)
#define DOWN(row,col)   (imag[row][col]==0)&&(imag[row-1][col]!=0)&&(imag[row+1][col]==0)

//float Desired_speed_change = 0;          //一直为0，不能更改，消除积分累计
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 Finish_Flag; //处理完成标识位
extern uint32 image_process_time;   //图像处理时间
extern uint8 right_lost_num;
extern uint8 left_lost_num;
extern uint8 zebra_stop_flag;
extern uint8 annulus_L_memory;
extern uint8 annulus_L_Flag;
extern uint8 annulus_R_memory;
extern uint8 annulus_R_Flag;
void Calculate_Offset_1();
void HDPJ_lvbo(uint8 data[], uint8 N, uint8 size);
extern int16 offset;                    //摄像头处理得到的偏差
extern float error_all,error_all_turn ;
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数
extern uint16 dir_r[(uint16)USE_num];//用来存储右边生长方向
extern uint16 dir_l[(uint16)USE_num];//用来存储左边生长方向
extern void image_process(void);
float Err_Sum(uint8 data[]);
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                                         uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2]);
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept);
void Get01change_Dajin(void);
uint8 Threshold1_Deal(uint8* image, uint16 col, uint16 row, uint32 pixel_Threshold1);
void binaryzation(void);
void IPS_show(void);
float absolute(float z);
void right_straight(void);
void Addingline1( uint8 choice, uint8 startX, uint8 startY);
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY);
void Addingline2( uint8 choice, uint8 startX, uint8 startY);
void Element_recognition(void);
void Check_Zhidao(void);
extern uint8 sudu_yingzi;
uint8 OtsuThreshold1(uint8 *image, uint16 col, uint16 row);
void advanced_regression1(int type, int startline1, int endline1, int startline2, int endline2);
int Judgment_symbol(float x, float y);
void Lower_left(void);
void Lower_right(void);
void Upper_left(void);
void Upper_right(void);
void  xieru_shzi();
void Mid_Col(void);
short GetOTSU (unsigned char MT_Img[image_h][image_w]);
void Get_Bin_Image (unsigned char mode);
void Sobel (uint8 imageIn[image_h][image_w], uint8 imageOut[image_h][image_w], uint8 Threshold1);
void Element_Test(void);
void check_bend_dir(void);
void  zebra_crossing(uint8(*bin_image)[image_w]);
extern uint8 Check_left_flag,Check_right_flag,Check_straight_flag;
extern uint8  middle[image_h];
extern uint8 imag[image_h][image_w];
extern uint8 Threshold1_value;
extern uint8 zebra_crossing_flag;//斑马线
int ImageA_MidLine_Get_F(void);
int ImageA_MidLine_Get_N(void);
void Check_obstacle1(void);
void CheckAllWhite(uint8 inflectionY);
#endif /* IMAGE_H_ */
