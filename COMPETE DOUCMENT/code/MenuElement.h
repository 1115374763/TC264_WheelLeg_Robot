
#ifndef CODE_MENUELEMENT_H_
#define CODE_MENUELEMENT_H_

#include "zf_common_headfile.h"

void MenuElementInit();



extern float Max_speed;     //最大速度
extern float Base_speed;    //基础速度
extern float Circle_speed;  //圆环速度
extern float Ramp_speed;    //坡道速度
extern float Ramp_End_speed;//坡道结束速度

extern float Qianzhan;      //前瞻

extern float Lp_Cross;      //十字转向环p
extern float Lp_Circle;      //圆环转向环p
extern float Lp_Ramp;      //坡道转向环p
extern float Lp_Ramp_End;  //坡道结束转向环p
extern float Lp_Base;      //基础转向环P

extern float Ap_Cross;      //十字角度环p
extern float Ap_Circle_Mode1;      //圆环一阶段角度环p
extern float Ap_Circle;      //圆环角度环环p
extern float Ap_Ramp;      //坡道角度环p
extern float Ap_Base;      //基础角度环P

extern int Cross_enable;
extern int Circle_enable;
extern int Ramp_enable;
extern int Zebra_enable;
extern int Barrier_enable;
extern int SingleBridge_enable;
extern int Stopcar_enable;
extern int Jumpbarrier_enable;
extern int Ramp_sudu_en;
extern int Mode_sudu;

extern int Time_1;
extern int Time_2;
extern int Time_Other;

extern int LeftCircleLine_1;
extern int LeftCircleLine_4;
extern int RightCircleLine_1;
extern int RightCircleLine_4;

extern int tof_distance;

#endif /* CODE_MENUELEMENT_H_ */
