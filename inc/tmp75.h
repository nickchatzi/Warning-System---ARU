#ifndef TMP75_H
#define TMP75_H

#include "stm32f030.h"

#pragma once

#define TEMP_READINGS_PER_HOUR 120

/************************ variable definitions *********************************/
extern uint8_t temperature[2];
extern float min_temp;
extern float max_temp;
extern volatile uint8_t new_min_event;
extern volatile uint8_t new_max_event;

/************************ function definitions ********************************/
void initTMP75();
void receive_temp_measurement();
void store_temperature(float temp);
void update_min_max(void);

#endif