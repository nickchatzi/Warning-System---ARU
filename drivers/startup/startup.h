#ifndef SRATUP_H
#define STARTUP_H

extern volatile uint32_t ms_ticks;
void SysTick_Handler(void);


#if !defined  (HSI_VALUE)
  #define HSI_VALUE    8000000U /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/*!< Uncomment the following line if you need to relocate the vector table
     anywhere in Flash or Sram, else the vector table is kept at the automatic
     remap of boot address selected */
#define USER_VECT_TAB_ADDRESS 

#if defined(USER_VECT_TAB_ADDRESS)
/*!< Uncomment the following line if you need to relocate your vector Table
     in Sram else user remap will be done in Flash. */
/* #define VECT_TAB_SRAM */

#if defined(VECT_TAB_SRAM)
#define VECT_TAB_BASE_ADDRESS   SRAM1_BASEADDR      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#else
#define VECT_TAB_BASE_ADDRESS   FLASH_BASEADDR      /*!< Vector Table base address field.
                                                     This value must be a multiple of 0x200. */
#define VECT_TAB_OFFSET         0x00000000U     /*!< Vector Table base offset field.
                                                     This value must be a multiple of 0x200. */
#endif /* VECT_TAB_SRAM */
#endif /* USER_VECT_TAB_ADDRESS */


// ----------- Bit Definitions for SysTick->CTRL -----------
#define SysTick_CTRL_ENABLE_Msk       (1UL << 0)
#define SysTick_CTRL_TICKINT_Msk      (1UL << 1)
#define SysTick_CTRL_CLKSOURCE_Msk    (1UL << 2)

// ----------- Reload value mask (24-bit counter) -----------
#define SysTick_LOAD_RELOAD_Msk       (0xFFFFFFUL)

void SystemInit(void);
int SysTick_Config(uint32_t ticks);
void SysTick_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif