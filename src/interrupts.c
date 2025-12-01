#include "stm32f030.h"
#include "main.h"
#include "gpio_driver.h"
#include "display.h"
#include "tmp75.h"
#include "init_system_states.h"
#include "timers.h"

extern volatile uint8_t new_data_ready = 0;
extern volatile float latest_temp = 0;

void EXTI4_15_IRQHandler(void)
{
  // Handle UP button (GPIOA7)
  if ((EXTI->PR & (1 << 7)))
  {
    GPIO_IRQHandling(GPIO_PIN_NO_7);
    flag_up = 1;
  }

  //Handle DOWN button (GPIOA5)
  if ((EXTI->PR & (1 << 5)))
  {
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    flag_down = 1;
  }

  // Handle ENTER button (GPIOA6)
  if (EXTI->PR & (1 << 6))
  {
    GPIO_IRQHandling(GPIO_PIN_NO_6);
    flag_enter = 1; 
  }
}

void TIM14_IRQHandler(void)
{
  receive_temp_measurement();
  latest_temp = convertTemp();
  store_temperature(latest_temp);
  new_data_ready = 1;   // flag for main
}

void TIM3_IRQHandler(void)
{
  if (TIM3_SR & 1)
  {
    TIM3_SR &= ~1;       
    char state = set_state();
    if (state == STATE_NORMAL)
    {
      GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, RESET); 
      TIM3_CR1 = 0; // stop timer
    }      
  }
}

void irq_handlers_init()
{
  GPIO_IRQPriorityConfig(IRQ_NO_EXTI4_15, 15);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI4_15, ENABLE);

  GPIO_IRQPriorityConfig(IRQ_NO_TIM14, 15);
  GPIO_IRQInterruptConfig(IRQ_NO_TIM14, ENABLE);

  GPIO_IRQPriorityConfig(IRQ_NO_TIM3, 15);
  GPIO_IRQInterruptConfig(IRQ_NO_TIM3, ENABLE);
}