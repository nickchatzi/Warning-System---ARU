#include "stm32f030.h"
#include "startup.h"
#include "buzzer.h"

volatile uint32_t ms_ticks = 0;

void SystemInit(void)
{
#if defined(USER_VECT_TAB_ADDRESS)
  /* Configure the Vector Table location -------------------------------------*/
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif

  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
#endif
}

void SysTick_Handler(void) 
{
  ms_ticks++;
  buzzer_update();
}

// -----------  SysTick_Config() Function -----------
int SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
        return 1;  // Value too high, overflow error

    SysTick->LOAD  = ticks - 1UL;                         // Set reload register
    SysTick->VAL   = 0UL;                                 // Clear current value
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |         // Use processor clock
                     SysTick_CTRL_TICKINT_Msk   |         // Enable interrupt
                     SysTick_CTRL_ENABLE_Msk;             // Enable SysTick
    return 0;  // Success
}

void SysTick_Init(void)
{
    SysTick->LOAD = (32000000 / 1000000) - 1;   // 1 µs tick at  MHz
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


void delay_us(uint32_t us)
{
    uint32_t start = SysTick->VAL;
    uint32_t ticks_per_us = (HSI_VALUE / 1000000);
    uint32_t ticks = us * ticks_per_us;
    uint32_t elapsed = 0;
    uint32_t prev = start;

    while (elapsed < ticks)
    {
        uint32_t now = SysTick->VAL;
        if (now < prev)
            elapsed += prev - now;
        else
            elapsed += SysTick->LOAD - (now - prev);
        prev = now;
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);   // 1 ms = 1000 µs
    }
}