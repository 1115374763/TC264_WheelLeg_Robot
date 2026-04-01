#ifndef CAMERA_H_
#define CAMERA_H_

#include "zf_common_headfile.h"
#define countnum (250)
void ZeroOne(uint8 *ima,uint8 h_start,uint8 height,uint8 w_start,uint8 width,uint8 yuzhi);
void fenkuai_ZeroOne(uint8 height,uint8 width,uint8 leftyuzhi,uint8 midyuzhi,uint8 rightyuzhi);
void XUNXIAN(uint8 height,uint8 width);
void XUNXIAN_MODE1(uint8 height,uint8 width);
void XUNXIAN_cross(uint8 height,uint8 width);
void Track_Analysis();
uint8 otsuThreshold(uint8 *ima,uint8 h_start,uint8 h_end,uint8 w_start,uint8 w_end);
extern uint8 zx[MT9V03X_H],yx[MT9V03X_H],wx[MT9V03X_H],Left_Lost_Flag[MT9V03X_H],Right_Lost_Flag[MT9V03X_H];
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);
float FMy_Abs(float a, float b);
void Midline_get();
void is_wangdao();
void is_straight();
void image_draw_rectan();
void image_draw_rectan_zeroone();
void search_neighborhood(void);
void search_neighborhood_zeroone(void);
void clear_find_point(void);
void get_turningleft_point(void);
void get_turningright_point(void);
void get_upturningleft_point(void);
void get_upturningright_point(void);

void Circle_in_Left();
void Circle_out_Left();
void Circle_in_Right();
void Circle_out_Right();

int Monotonicity_Change_Left();
int Monotonicity_Change_Right();

int16 calc_diff(int16 x, int16 y);
void fineonepoint();
void fineonepoint_zeroone();
void searchmode(int i);
float Err_Sum1(void);
void get_SingleBridgeleft_point(void);
void get_SingleBridgeright_point(void);

extern uint8 image[MT9V03X_H][MT9V03X_W];
extern uint8 Threshold;
extern uint8 Threshold_Lup;
extern uint8 Threshold_Rup;
extern uint8 Finnalline;
extern uint8 left_threshold;
extern uint8 mid_threshold;
extern uint8 right_threshold;
extern uint8 threshold ;
extern uint8 Midline_white;
extern uint8 Leftline_white;
extern uint8 Rightline_white;
extern uint8 leftlost_mode;
extern uint8 rightlost_mode;
extern uint8 left_control_flag;
extern uint8 right_control_flag;
extern uint8 trueshortflag;
extern float parameterB;
extern float parameterA;
extern float Err;
extern int forwardline;
extern uint8 Left_Lost_Time;
extern uint8 Right_Lost_Time;
extern uint8 Both_Lost_Time;
extern uint8 Boundry_Start_Left;
extern uint8 Boundry_Start_Right;
extern uint8 Boundry_Startlost_Left;
extern uint8 Boundry_Startlost_right;
extern uint8 cross_xunxian_mode;
extern uint8 L_edge_row[countnum],L_edge_col[countnum];
extern uint8 R_edge_row[countnum],R_edge_col[countnum];
//extern uint8 H_edge_row[Hcount],H_edge_col[Hcount];
extern uint8 Longest_White_Column_Left[2];
extern int16 L_corner_flag;
extern int16 L_corner_row;
extern int16 L_corner_col;
extern float L_corner_angle;
extern int16 R_corner_flag;
extern int16 R_corner_row;
extern int16 R_corner_col;
extern float R_corner_angle;
extern uint8 L_edge_count;
extern uint8 R_edge_count;                     //左右边点的个数
extern uint8 L_start_y;
extern uint8 L_start_x;
extern uint8 R_start_y;
extern uint8 R_start_x;
extern int16 L_upcorner_flag;//左拐点存在标志
extern int16 L_upcorner_row;//左拐点所在行
extern int16 L_upcorner_col;//左拐点所在列
extern int16 R_upcorner_flag;//左拐点存在标志
extern int16 R_upcorner_row;//左拐点所在行
extern int16 R_upcorner_col;//左拐点所在列
extern int16 L_circle_corner_flag;//左拐点存在标志
extern int16 L_circle_corner_row;//左拐点所在行
extern int16 L_circle_corner_col;//左拐点所在列
extern int16 R_circle_corner_flag;//右拐点存在标志
extern int16 R_circle_corner_row;//右拐点所在行
extern int16 R_circle_corner_col;//右拐点所在列

extern int16 Lin_circleup_flag;//左拐点存在标志
extern int16 Lin_circleup_row;//左拐点所在行
extern int16 Lin_circleup_col;//左拐点所在列

extern int16 Lout_circleup_flag;//左拐点存在标志
extern int16 Lout_circleup_row;//左拐点所在行
extern int16 Lout_circleup_col;//左拐点所在列

extern int16 Rin_circleup_flag;//左拐点存在标志
extern int16 Rin_circleup_row;//左拐点所在行
extern int16 Rin_circleup_col;//左拐点所在列

extern int16 Rout_circleup_flag;//左拐点存在标志
extern int16 Rout_circleup_row;//左拐点所在行
extern int16 Rout_circleup_col;//左拐点所在列

extern int16 L_SingleBridge_flag ;//左拐点存在标志
extern int16 L_SingleBridge_row ;//左拐点所在行
extern int16 L_SingleBridge_col ;//左拐点所在列
extern int16 R_SingleBridge_flag ;//右拐点存在标志
extern int16 R_SingleBridge_row ;//右拐点所在行
extern int16 R_SingleBridge_col ;//右拐点所在列

//extern uint8 H_edge_count;






#endif
