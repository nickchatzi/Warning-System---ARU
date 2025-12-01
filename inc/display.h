#ifndef DISPLAY_H
#define DISPLAY_H

#include "stm32f030.h"

#pragma once

/************************ variable definitions *********************************/

extern volatile uint8_t flag_up;
extern volatile uint8_t flag_down;
extern volatile uint8_t flag_enter;

extern volatile uint8_t selected_option;       
extern volatile uint8_t confirmed_selection; 

extern volatile int16_t upper_threshold;
extern volatile int16_t lower_threshold;

extern uint32_t last_btn_time;

/************************ function definitions ********************************/

void activate_backlight_counter(void);
void display_temp(void);
float convertTemp(void);
void display_upper_threshold(void);
void display_lower_threshold(void);
void main_threshold_temp_set(void);
void display_max_temp(void);
void display_min_temp(void);

#endif