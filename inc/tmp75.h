#ifndef TMP75_H
#define TMP75_H

#include "stm32f030.h"
#include "i2c_driver.h"

#pragma once

#define TEMP_READINGS_PER_HOUR 120

/************************ variable definitions *********************************/
extern uint8_t temperature[2];
extern float min_temp;
extern float max_temp;
extern volatile uint8_t new_min_event;
extern volatile uint8_t new_max_event;

/************************ function definitions ********************************/
void initTMP75(void);
void receive_temp_measurement(void);
void store_temperature(float temp);
void update_min_max(void);
//void redLed_update(void);
//void blueLed_update(void);

#endif