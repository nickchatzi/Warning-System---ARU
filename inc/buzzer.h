#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f030.h"
#include "gpio_driver.h"
#include "startup.h"

#pragma once

#define FAST_BEEP_PERIOD_MS   200   // High temp: fast beep (200ms)
#define SLOW_BEEP_PERIOD_MS   800   // Low temp: slow beep (800ms)

/******************************* PRIVATE DEFINITIONS ************************************/

static volatile uint8_t beep_once;
static volatile uint8_t single_beep_request;

/******************************* FUNCTION DEFINITIONS ************************************/

void buzzer_on(uint32_t period);
void buzzer_off(void);
void buzzer_single_beep(void);
void buzzer_double_beep_request(void);
void buzzer_triple_beep_request(void);
void buzzer_update(void);


#endif