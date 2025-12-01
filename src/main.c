/******************************************************************************
 *  File Name    : main.c
 *  Author       : Nikolaos Chatzilamprou
 *  Description  : Entry point for the Temperature Warning System application.
 *
 *  Copyright (c) 2025 Nikolaos Chatzilamprou
 *  All rights reserved.
 *
 *  This source code is proprietary and confidential. Unauthorized copying
 *  of this file, via any medium, is strictly prohibited without prior
 *  written permission from the author.
 *
 *  Created on   : 2025-11-28
 *  Version      : 1.0.0
 *
 *  Notes:
 *  - Initial release
 *
 ******************************************************************************/

#include "gpio_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "rcc_driver.h"
#include "all_inits.h"
#include "lcd.h"
#include "main.h"
#include "startup.h"
#include "display.h"
#include "timers.h"
#include "tmp75.h"
#include "init_system_states.h"

int main(void)
{
  SysTick_Init();
  hardware_init();
  //initTMP75();
  irq_handlers_init();
  lcd_init();
  lcd_set_black_background();
  SysTick_Config(HSI_VALUE / 1000);
  
  while (1)
  {
    SystemState_t s = set_state();
    buzzer_update();
    system_state_process(s);
    main_threshold_temp_set();
    display_upper_threshold();
    display_lower_threshold();
    display_max_temp();
    display_min_temp();
    display_temp();
  }
}

 /*********************************END FILE***************************************/