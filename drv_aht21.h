#ifndef __DRV_AHT21_H
#define __DRV_AHT21_H

#include "main.h"

#define AHT_SCL_PORT    GPIOE
#define AHT_SCL_PIN     GPIO_PIN_0
#define AHT_SDA_PORT    GPIOE
#define AHT_SDA_PIN     GPIO_PIN_1

/* AHT20/21 I2C Address */
#define AHT_ADDR        0x38

/* Function Prototypes */
void AHT21_Init(void);
uint8_t AHT21_Read(float *Temperature, float *Humidity);

#endif
