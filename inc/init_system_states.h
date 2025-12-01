#ifndef INIT_SYSTEM_STATES
#define INIT_SYSTEM_STATES

#include "stm32f030.h"
#include "gpio_driver.h"

#pragma once

typedef enum{
    STATE_NORMAL = 0,
    STATE_HIGH_TEMP,
    STATE_LOW_TEMP
}SystemState_t;

void system_state_process(const SystemState_t s);
SystemState_t set_state(void);

void buzzer_init(void);

#endif