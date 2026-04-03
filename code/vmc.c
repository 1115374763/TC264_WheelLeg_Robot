/*
 * vmc.c
 *
 *  Created on: 2024年12月3日
 *      Author: Administrator
 */
#include "vmc.h"
#include "zf_common_headfile.h"

/**
 * @brief根据最下点坐标，运动学逆解出舵机打角
 *
 * @note 右侧视图
 *  ___x
 * |    1  _____  4
 * |y     /     \
 *      2 \     / 3
 *         \   /
 *          \./
 *           5
 *
 */




float servo_motor_duty1 = 90.0;                                                  // 舵机动作角度
float servo_motor_duty2 = 90.0;                                                  // 舵机动作角度
float servo_motor_duty3 = 90.0;                                                  // 舵机动作角度
float servo_motor_duty4 = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态
float E_H = 0.0;
float Stab_roll = 0.0;
float Ex_roll = 0.0;
extern int bridge_pri;
IKparam IKParam;
int16_t alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;
int count_jump=0;
uint16 jumptime = 0;
uint16 jumptime0 = 0;
uint16 jumptime1 = 0;
uint16 jumptime2 = 0;
uint16 jumptime3 = 0;
uint16 jumptime4 = 0;
int jump_out =0;
int jumpaginst = 0;
uint16 annulustime = 0;
uint8 jump_mode = 0;
uint8 jump_cnt =0;
float speed_k = 0.0;

/*-------------------------------------------------------------------------------------------------------------------
  @brief     初始化四个腿
  @note
-------------------------------------------------------------------------------------------------------------------*/
void SERVO_init()
{
    pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 0); //3
    pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 0); //2
    pwm_init(SERVO_MOTOR_PWM3, SERVO_MOTOR_FREQ, 0); //1
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 0); //4

//    pwm_init(ATOM0_CH4_P02_4, 300, a1);  //右上  1800上 a1中 7960下        -500  4180
//    pwm_init(ATOM0_CH5_P02_5, 300, a4);  //左上  1400下 a4中 7580上         +500  5440
//    pwm_init(ATOM0_CH6_P02_6, 300, a3);  //右下  1200下 a3中 7480下       +500  4840
//    pwm_init(ATOM0_CH7_P02_7, 300, a2);  //左下  1260上 a2中 7580下        -500  3650
}

int annulus_L_3=0;
int time_single=0;
int time_single_time=2600;
float o11=0;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     腿部控制
  @note
-------------------------------------------------------------------------------------------------------------------*/
void inverseKinematics()
{
  float alpha1,alpha2,beta1,beta2;
  uint16_t servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;
  uint32 servoLeftFrontPwm,servoLeftRearPwm,servoRightFrontPwm,servoRightRearPwm;


//  if(stop_flag == 1)
//  {
//      speed_k = 20.0;
//  }
//  else if(annulus_L_Flag == 1)
//  {
//      speed_k = 10.0;
//  }
//  else if(Check_straight_flag == 1)
//  {
//      speed_k = 4.0;
//  }
//  else if(Jumpbarrier_mode==1)
//  {
//      speed_k = 20.0;
//  }
//  else if(hengduan_flag==2)
//  {
//      speed_k = 30.0;
//  }
//  else if(SingleBridge_mode==1)
//  {
//      speed_k = 30.0;
//  }
//  else
//  {
//      speed_k = 8.0;
//  }

  if(o11>motor_pwm) o11=o11-0.2;
  if(o11<motor_pwm) o11=o11+0.2;


  IKParam.XLeft  = o11+20;
  IKParam.XRight = o11+20;

  if(IKParam.XLeft >80) IKParam.XLeft=80;
  else if(IKParam.XLeft<-55) IKParam.XLeft=-55;
  if(IKParam.XRight >80) IKParam.XRight=80;
  else if(IKParam.XRight<-55) IKParam.XRight=-55;



  // =========================================================
    // 【高度与跳跃状态机】严格互斥，防止指令打架
    // =========================================================
    if (hengduan_flag != 0) 
    {
        // 跳跃期间的 X 轴姿态微调
        IKParam.XLeft  = o11 + 18;
        IKParam.XRight = o11 + 18;
        if(IKParam.XLeft > 80) IKParam.XLeft = 80;
        else if(IKParam.XLeft < -55) IKParam.XLeft = -55;
        if(IKParam.XRight > 80) IKParam.XRight = 80;
        else if(IKParam.XRight < -55) IKParam.XRight = -55;

        // --- 阶段1：爆发蹬腿 (起跳) ---
        if (hengduan_flag == 1) 
        {
            jumptime0++;
            IKParam.YLeft  = 163.0f; // 起跳需要最大爆发力，直接阶跃
            IKParam.YRight = 163.0f; 
            if (jumptime0 >= 130) {
                jumptime0 = 0;
                hengduan_flag = 2;
            }
        }
        // --- 阶段2：腾空收腿 (越障) ---
        else if (hengduan_flag == 2) 
        {
            jumptime1++;
            // 优化点：使用低通滤波平滑收腿，避免瞬态冲击扫齿。下限抬高到 45 保证机械不死点。
            IKParam.YLeft  = IKParam.YLeft  + (45.0f - IKParam.YLeft)  * 0.23f;
            IKParam.YRight = IKParam.YRight + (45.0f - IKParam.YRight) * 0.23f;
            if (jumptime1 >= 95) {
                jumptime1 = 0;
                hengduan_flag = 3;
            }
        }
        // --- 阶段3：准备着陆 (伸出腿当弹簧) ---
        else if (hengduan_flag == 3) 
        {
            jumptime2++;
            // 优化点：落地前腿要半弯曲（比如80），太直容易震坏，太弯缓冲行程不够
            IKParam.YLeft  = IKParam.YLeft  + (82.0f - IKParam.YLeft)  * 0.15f;
            IKParam.YRight = IKParam.YRight + (82.0f - IKParam.YRight) * 0.15f;
            if (jumptime2 >= 90) {
                jumptime2 = 0;
                hengduan_flag = 4;
            }
        }
        // --- 阶段4：落地缓冲与恢复常态 ---
        else if (hengduan_flag == 4) 
        {
            jumptime3++;
            // 优化点：缓慢恢复到平地常态高度 51，吸收落地动能
            IKParam.YLeft  = IKParam.YLeft  + (51.0f - IKParam.YLeft)  * 0.04f; 
            IKParam.YRight = IKParam.YRight + (51.0f - IKParam.YRight) * 0.04f;
            
            if (jumptime3 >= 100) {
                jumptime3 = 0;
                jump_out = 1;
                hengduan_flag = 5; // 转移到冷却阶段，防止枚举混乱
            }
        }
        // --- 阶段5：跳跃冷却 (防连跳) ---
        else if (hengduan_flag == 5 && jump_out == 1)
        {
            jumptime4++;
            // 保持常态高度
            IKParam.YLeft  = IKParam.YLeft  + (51.0f - IKParam.YLeft)  * 0.1f;
            IKParam.YRight = IKParam.YRight + (51.0f - IKParam.YRight) * 0.1f;

            if (jumptime4 >= 5000) { // 冷却时间到达
                jump_out = 0;
                jumptime4 = 0;
                hengduan_flag = 0;   // 彻底退出跳跃模式
            }
        }

        // 跳跃期间强制进行 Y 轴安全限幅
        if(IKParam.YLeft > 175) IKParam.YLeft = 175;
        else if(IKParam.YLeft < 45) IKParam.YLeft = 45; // 保护下限
        if(IKParam.YRight > 175) IKParam.YRight = 175;
        else if(IKParam.YRight < 45) IKParam.YRight = 45;
    }
    // =========================================================
    // 【常态与其他模式】只有不在跳跃时才执行，绝对互斥！
    // =========================================================
    else if (SingleBridge_mode == 1) // === 单边桥模式 ===
    {
        Exceptspeed = 820;
        Ex_roll = balance_rollangle_PI(QEKF_INS.Roll, -4.0);
        IKParam.YLeft  = 79 - Ex_roll;
        IKParam.YRight = 79 + Ex_roll;

        if(IKParam.YLeft > 175) IKParam.YLeft = 175;
        else if(IKParam.YLeft < 40) IKParam.YLeft = 40;
        if(IKParam.YRight > 175) IKParam.YRight = 175;
        else if(IKParam.YRight < 40) IKParam.YRight = 40;
    }
    else if (flag1 == 1) // === 坡道模式 ===
    {
        Exceptspeed = 800;
        IKParam.YLeft = IKParam.YLeft + (56.0f + lora3a22_uart_transfer.joystick[3] / 20.0f - IKParam.YLeft) / 5.0f;
        if(IKParam.YLeft > 175) IKParam.YLeft = 175;
        else if(IKParam.YLeft < 40) IKParam.YLeft = 40;

        IKParam.YRight = IKParam.YRight + (56.0f + lora3a22_uart_transfer.joystick[3] / 20.0f - IKParam.YRight) / 5.0f;
        if(IKParam.YRight > 175) IKParam.YRight = 175;
        else if(IKParam.YRight < 40) IKParam.YRight = 40;
    }
    else // === 正常平地模式 ===
    {
        IKParam.YLeft = IKParam.YLeft + (51.0f + lora3a22_uart_transfer.joystick[3] / 20.0f - IKParam.YLeft) / 5.0f;
        if(IKParam.YLeft > 175) IKParam.YLeft = 175;
        else if(IKParam.YLeft < 40) IKParam.YLeft = 40;

        IKParam.YRight = IKParam.YRight + (51.0f + lora3a22_uart_transfer.joystick[3] / 20.0f - IKParam.YRight) / 5.0f;
        if(IKParam.YRight > 175) IKParam.YRight = 175;
        else if(IKParam.YRight < 40) IKParam.YRight = 40;
    }

    // 单边桥的防卡死逻辑保持独立
    if (once == 1) {
        Check_obstacle_flag = 0;
        time_single++;
    }
    if (time_single > time_single_time) {
        SingleBridge_mode = 0;
        once = 0;
        time_single = 0;
    }

    // ... 下方继续保留你的运动学解算代码：float aLeft = 2 * IKParam.XLeft * L1; ...






  if(flag1==1)//PODAO
      {
          Exceptspeed=800;
          IKParam.YLeft = IKParam.YLeft + (56.0+lora3a22_uart_transfer.joystick[3]/20.0 - IKParam.YLeft)/5.0;//括号第一个变量为初始值
          if(IKParam.YLeft >175) IKParam.YLeft=175;
          else if(IKParam.YLeft<40) IKParam.YLeft=40;

          IKParam.YRight = IKParam.YRight + (56.0+lora3a22_uart_transfer.joystick[3]/20.0  - IKParam.YRight)/5.0;//
          if(IKParam.YRight >175) IKParam.YRight=175;
          else if(IKParam.YRight<40) IKParam.YRight=40;


      }





  else
  {
      IKParam.YLeft = IKParam.YLeft + (51.0+lora3a22_uart_transfer.joystick[3]/20.0 - IKParam.YLeft)/5.0;//括号第一个变量为初始值
      if(IKParam.YLeft >175) IKParam.YLeft=175;
      else if(IKParam.YLeft<40) IKParam.YLeft=40;

      IKParam.YRight = IKParam.YRight + (51.0+lora3a22_uart_transfer.joystick[3]/20.0  - IKParam.YRight)/5.0;//
      if(IKParam.YRight >175) IKParam.YRight=175;
      else if(IKParam.YRight<40) IKParam.YRight=40;


  }





  float aLeft = 2 * IKParam.XLeft * L1;
  float bLeft = 2 * IKParam.YLeft * L1;
  float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
  float dLeft = 2 * L4 * (IKParam.XLeft - L5);
  float eLeft = 2 * L4 * IKParam.YLeft;
  float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

  alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
  beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
  beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;
  else IKParam.alphaLeft = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaLeft = beta1;
  else IKParam.betaLeft = beta2;

  float aRight = 2 * IKParam.XRight * L1;
  float bRight = 2 * IKParam.YRight * L1;
  float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (IKParam.XRight - L5);
  float eRight = 2 * L4 * IKParam.YRight;
  float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

  IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
  beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
  beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

  alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

  if(alpha1 >= PI/4) IKParam.alphaRight = alpha1;
  else IKParam.alphaRight = alpha2;
  if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaRight = beta1;
  else IKParam.betaRight = beta2;

  alphaLeftToAngle = (int)((IKParam.alphaLeft / 6.28) * 360);//弧度转角度
  betaLeftToAngle = (int)((IKParam.betaLeft / 6.28) * 360);

  alphaRightToAngle = (int)((IKParam.alphaRight / 6.28) * 360);
  betaRightToAngle = (int)((IKParam.betaRight / 6.28) * 360);

  servoLeftFront = 90 + betaLeftToAngle;
  servoLeftRear =  alphaLeftToAngle-90;
  servoRightFront = 90 - betaRightToAngle;
  servoRightRear = 270 - alphaRightToAngle;

  servoLeftFrontPwm=SERVO_MOTOR_DUTY(servoLeftFront);
  if(servoLeftFrontPwm>=9999) servoLeftFrontPwm=9999;

  servoLeftRearPwm =SERVO_MOTOR_DUTY(servoLeftRear);
  if(servoLeftRearPwm>=9999) servoLeftRearPwm=9999;

  servoRightFrontPwm=SERVO_MOTOR_DUTY(servoRightFront);
  if(servoRightFrontPwm>=9999) servoRightFrontPwm=9999;

  servoRightRearPwm=SERVO_MOTOR_DUTY(servoRightRear);
  if(servoRightRearPwm>=9999) servoRightRearPwm=9999;

    pwm_set_duty(SERVO_MOTOR_PWM1, servoRightRearPwm);
    pwm_set_duty(SERVO_MOTOR_PWM2, servoRightFrontPwm);
    pwm_set_duty(SERVO_MOTOR_PWM3, servoLeftRearPwm);
    pwm_set_duty(SERVO_MOTOR_PWM4, servoLeftFrontPwm);

}









