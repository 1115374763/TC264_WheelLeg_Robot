#ifndef ELEMENT_H_
#define ELEMENT_H_


#include "zf_common_headfile.h"


extern uint8 Cross_Flag;
extern uint8 Cross_mode;

extern uint8 circle_flag;
extern uint8 circle_mode;
extern uint8 left_circle_flag;
extern uint8 right_circle_flag;
extern uint8 left_circle_cnt;
extern uint8 right_circle_cnt;
extern uint8 left_circle_flag_cnt;
extern uint8 right_circle_flag_cnt;
extern int left_change_cnt;
extern int right_change_cnt;
extern uint8 Barrier_Flag;
extern uint8 left_Barrier_Flag;
extern uint8 right_Barrier_Flag;
extern uint8 barrier_mode;
extern uint16 t_1s ;
extern uint16 t_2s ;
extern uint16 t_5s ;
extern uint16 t_10s ;
extern uint16 t1_ms_cricle1 ;
extern uint16 t2_ms_cricle1 ;
extern uint16 t2_ms_cricle2 ;
extern uint16 t2_ms_cricle3 ;
extern uint16 t1_ms_cricle4 ;
extern uint16 t1_ms_cricle5 ;
extern uint8 T_1s_flag;
extern uint8 T_2s_flag;
extern uint8 T_10s_flag;
extern uint8 T1_ms_cricle1_flag;
extern uint8 T1_ms_cricle4_flag;
extern uint8 T1_ms_cricle5_flag;
extern uint8 Zebra_Flag;
extern uint8 zebra_mode;
extern uint8 zebra_stop_flag;
extern uint16 zebra_mscnt;
extern int change_count;
extern uint8 Ramp_Flag;
extern uint8 Ramp_mode;
extern uint8 circle_cnt;
extern uint8 cross_cnt;
extern uint8 barrier_cnt;
extern uint8 ramp_cnt;
extern uint8 circle_width_left;
extern uint8 circle_width_right;
extern uint8 base_width;
extern uint8 Ramp_End_Flag;
extern uint16 Ramp_ms_cnt;
extern uint8 UnRamp_Flag;
extern uint8 barrier_width_left;
extern uint8 barrier_width_right;
extern uint8 barrier_width_avg;
extern uint8 SingleBridge_Flag;
extern uint8 SingleBridge_mode;
extern uint8 BlackFind_Left_Flag;
extern uint8 BlackFind_Right_Flag;
extern float SingleBridge_err;
extern uint16 SingleBridge_ms_cnt;
extern uint8 Left_SingleBridge_Flag;
extern uint8 Right_SingleBridge_Flag;
extern uint8 SingleBridgeEnd_Flag;
extern float Total_err_sum;
extern uint8 ;
extern uint8 SingleBridge_width_right;
extern int SingleBridge_width_chazhi;
extern uint8 Jumpbarrier_Flag;
extern uint8 Jumpbarrier_mode;
extern uint8 Jumpbarrier_cnt;
extern uint8 Half_zhidao_width[120];
extern uint8 stop_flag;
extern uint8 T1_SingleBridge_flag;
extern float now_yaw;





void cross_find();
void Left_Add_Line(int x1,int y1,int x2,int y2);
void Right_Add_Line(int x1,int y1,int x2,int y2);
void Lengthen_Left_Boundry(int start,int end);
void Lengthen_Left_downBoundry(int start,int end);
void Lengthen_Right_Boundry(int start,int end);
void Lengthen_Right_downBoundry(int start,int end);
void circle_find();
void is_circle();
void barrier_find();
void is_barrier();
void Zebra_find(void);
void Ramp_find(void);
void SingleBridge_find();
void buzzer();
void Jumpbarrier_find();
void buzzer_stop();

int Continuity_Change_Left(int start,int end);
int Continuity_Change_Right(int start,int end);
int Left_sideIsNoLost(int start,int end);
int Right_sideIsNoLost(int start,int end);
int Left_TotalLost(int start,int end);
int Right_TotalLost(int start,int end);
int left_Bwjump_counting(int start,int end);
int right_Bwjump_counting(int start,int end);
int Extrawide_counting(int start,int end);
void Led_process();
void Element_cnt_init();
void Stop_car();

#endif
