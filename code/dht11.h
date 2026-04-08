#ifndef _DHT11_H_
#define _DHT11_H_

#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"


#define DHT11_PIN   P22_3

uint8 DHT11_Init(void);
uint8 DHT11_Read_Data(uint8 *temp, uint8 *humi);

#endif
