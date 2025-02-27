#ifndef DHT22_H_
#define DHT22_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
//--------------------------------------------------
#define data_port GPIOA
#define data_pin  GPIO_PIN_2
void dht22_init(void);
uint8_t DHT22_GetTemp_Humidity(float *Temp, float *Humidity);
//--------------------------------------------------
#endif /* DHT22_H_ */
