#ifndef ALL_INITS
#define ALL_INITS

#include "i2c_driver.h"

#pragma once

#define SLAVE_ADDRESS   0X48

extern I2C_Handle I2C1Handle;

//void SPI1_GPIOInits(void);
//void SPI1_Inits(void);
//void I2C1_GPIOInits(void);
//void I2C_Inits(void);
//void RSTPIN_Init();
//void RSPIN_Init();
//void CSPIN_Init();
//void ButtonUp_init(void);
//void ButtonEnter_init(void);
//void ButtonDown_init(void);
//void Redled_init(void);
//void Blueled_init(void);
void hardware_init();

#endif