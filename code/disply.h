

#ifndef CODE_DISPLY_H_
#define CODE_DISPLY_H_

#include "zf_common_headfile.h"

//#define key_left  gpio_get_level(P11_2)//P11_9
//#define key_right   gpio_get_level(P11_9)//P11_2
//#define key_mid    gpio_get_level(P11_11)
//#define key_down   gpio_get_level(P11_10)//
//#define key_up  gpio_get_level(P11_3)

#define key_left  gpio_get_level(P11_2)
#define key_right   gpio_get_level(P11_3)
#define key_mid    gpio_get_level(P20_9)
#define key_down   gpio_get_level(P20_7)
#define key_up  gpio_get_level(P20_6)

void row_control(void);
void adjust_control(void);
void Menu(void);
void disply_init(void);
void data_read();
void data_write();

extern uint8 camera_enable;
extern uint8 jiasu_enable;
extern uint8 motor_enable;
extern uint32 tim;
extern float angle_banlance_error;
extern float vertical_banlance_error;
extern float location_banlance_error;
extern float servo_xunji_error;



#endif /* CODE_DISPLY_H_ */
