#ifndef STM32F030XX_H_
#define STM32F030XX_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/********************STRUCTURE FOR GPIOx REGISTERS********************/

typedef struct 
{
    volatile uint32_t MODER;                     //Address Offset 0x00               
    volatile uint32_t OTYPER;                    //Address Offset 0x04
    volatile uint32_t OSPEEDR;                   //Address Offset 0x08
    volatile uint32_t PUPDR;                     //Address Offset 0x0C
    volatile uint32_t IDR;                       //Address Offset 0x10
    volatile uint32_t ODR;                       //Address Offset 0x14
    volatile uint32_t BSRR;                      //Address Offset 0x18
    volatile uint32_t LCKR;                      //Address Offset 0x1C
    volatile uint32_t AFR[2];                    //Address Offset ARF[0]->AFRL=0x20 & ARF[1]->AFRH=0x24

}GPIOx_RegDef;

/*********************************************************************/

/********************STRUCTURE FOR SPIx REGISTERS********************/

typedef struct 
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;

}SPI_RegDef;

/*********************************************************************/

/********************STRUCTURE FOR I2Cx REGISTERS********************/

typedef struct 
{
    volatile uint32_t I2C_CR1;
    volatile uint32_t I2C_CR2;
    volatile uint32_t I2C_OAR1;
    volatile uint32_t I2C_OAR2;
    volatile uint32_t I2C_TIMINGR;
    volatile uint32_t I2C_TIMEOUTR;
    volatile uint32_t I2C_ISR;
    volatile uint32_t I2C_ICR;
    volatile uint32_t I2C_PECR;
    volatile uint32_t I2C_RXDR;
    volatile uint32_t I2C_TXDR;

}I2C_RegDef;

/*********************************************************************/

/********************STRUCTURE FOR USARTx REGISTERS********************/

typedef struct 
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;

}USART_RegDef;

/*********************************************************************/

/*********************STRUCTURE FOR TIM REGISTERS*********************/

typedef struct
{
    volatile uint32_t CR1;    
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;

}TIM_RegDef;
/**********************************************************************/

/*********************STRUCTURE FOR RCC REGISTERS*********************/

typedef struct
{
    volatile uint32_t CR;    
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t AHBRSTR;
    volatile uint32_t CFGR2;
    volatile uint32_t CFGR3; 
    volatile uint32_t CR2;  

}RCC_RegDef;
/**********************************************************************/

/*******************STRUCTURE FOR EXTI REGISTERS***********************/

typedef struct
{
    volatile uint32_t IMR;    
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;

}EXTI_RegDef;

/**********************************************************************/

/*******************STRUCTURE FOR SYSCFG REGISTERS*******************/

typedef struct
{  
    volatile uint32_t CFGR1;
    volatile uint32_t RES;
    volatile uint32_t EXTICR[4];
    volatile uint32_t CFGR2;

}SYSCFG_RegDef;

/**********************************************************************/

/*******************STRUCTURE FOR SCB REGISTERS************************/

typedef struct
{
  volatile uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  volatile uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  volatile uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */

} SCB_Type;

/**********************************************************************/

/*****************STRUCTURE FOR SYSTICK REGISTERS**********************/

typedef struct 
{
  volatile uint32_t CTRL;    // Offset 0x00
  volatile uint32_t LOAD;    // Offset 0x04
  volatile uint32_t VAL;     // Offset 0x08
  volatile uint32_t CALIB;   // Offset 0x0C
} SysTick_Type;

/**********************************************************************/


/******BASE ADDRESSES OF FLASH AND SRAM MEMORY******/

#define FLASH_BASEADDR                  0x08000000U
#define SRAM_BASEADDR                   0x20000000U
#define ROM_BASEADDR                    0x1FFFD800U
#define SRAM                            SRAM_BASEADDR

/**************************************************/

/******BASE ADDRESSES OF AHBx AND APBx BUS PERIPHERAL******/

#define PERIPHERAL_BASEADDR             0x40000000U
#define APBPERIPH_BASEADDR              PERIPHERAL_BASEADDR
#define APB1PERIPH_BASEADDR             0x40010000U
#define AHB1PERIPH_BASEADDR             0x40020000U
#define AHB2PERIPH_BASEADDR             0x48000000U

/**********************************************************/

/*******************BASE ADDRESSES OF GPIOx PERIPHERAL****************/

#define GPIOA_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOF_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x1400)

/*********************************************************************/

/******************BASE ADDRESSES OF I2Cx PERIPHERAL******************/

#define I2C1_BASEADDR                   (APBPERIPH_BASEADDR + 0x5400) 
#define I2C2_BASEADDR                   (APBPERIPH_BASEADDR + 0x5800)

/*********************************************************************/

/******************BASE ADDRESSES OF SPIx PERIPHERAL******************/

#define SPI1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR                   (APBPERIPH_BASEADDR + 0x3800)

/*********************************************************************/

/******************BASE ADDRESSES OF USARTx PERIPHERAL******************/

#define USART1_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR                 (APBPERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                 (APBPERIPH_BASEADDR + 0x4800)
#define USART4_BASEADDR                 (APBPERIPH_BASEADDR + 0x4C00)
#define USART5_BASEADDR                 (APBPERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR                 (APB1PERIPH_BASEADDR + 0x1400)

/*********************************************************************/


/*******************BASE ADDRESSES OF TIMER PERIPHERALS***************/

#define TIM1_BASEADDR                  (APB1PERIPH_BASEADDR + 0x2C00)
#define TIM3_BASEADDR                  (APBPERIPH_BASEADDR + 0x0400)
#define TIM6_BASEADDR                  (APBPERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR                  (APBPERIPH_BASEADDR + 0x1400)
#define TIM14_BASEADDR                 (APBPERIPH_BASEADDR + 0x2000)
#define TIM15_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4000)
#define TIM16_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400)
#define TIM17_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800)

/*********************************************************************/

/*************BASE ADDRESSES OF EXT1 & SYSCFG PERIPHERAL*************/

#define EXTI_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0400)
#define SYSCFG_BASEADDR                 (APB1PERIPH_BASEADDR + 0x0000)
#define RCC_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1000)
#define SCB_BASE                        (0xE000ED00UL) 

/*********************************************************************/

/******GPIO DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT******/

#define GPIOA                           ((GPIOx_RegDef*) GPIOA_BASEADDR)                                      
#define GPIOB                           ((GPIOx_RegDef*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIOx_RegDef*) GPIOC_BASEADDR) 
#define GPIOD                           ((GPIOx_RegDef*) GPIOD_BASEADDR) 
#define GPIOF                           ((GPIOx_RegDef*) GPIOF_BASEADDR) 

/***********************************************************************/

/**********SPI DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT**********/

#define SPI1                            ((SPI_RegDef*) SPI1_BASEADDR)                                      
#define SPI2                            ((SPI_RegDef*) SPI2_BASEADDR) 

/***********************************************************************/

/**********I2C DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT**********/

#define I2C1                            ((I2C_RegDef*) I2C1_BASEADDR)                                      
#define I2C2                            ((I2C_RegDef*) I2C2_BASEADDR) 

/***********************************************************************/

/**********USART DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT**********/

#define USART1                          ((USART_RegDef*) USART1_BASEADDR)                                      
#define USART2                          ((USART_RegDef*) USART2_BASEADDR)
#define USART3                          ((USART_RegDef*) USART3_BASEADDR)
#define USART4                          ((USART_RegDef*) USART4_BASEADDR) 
#define USART5                          ((USART_RegDef*) USART5_BASEADDR)   
#define USART6                          ((USART_RegDef*) USART6_BASEADDR) 

/***********************************************************************/

/********TIMER DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT**********/

#define TIM1                            ((TIM_RegDef*) TIM1_BASEADDR)                                      
#define TIM3                            ((TIM_RegDef*) TIM3_BASEADDR) 
#define TIM6                            ((TIM_RegDef*) TIM6_BASEADDR) 
#define TIM7                            ((TIM_RegDef*) TIM7_BASEADDR) 
#define TIM14                           ((TIM_RegDef*) TIM14_BASEADDR) 
#define TIM15                           ((TIM_RegDef*) TIM15_BASEADDR)
#define TIM16                           ((TIM_RegDef*) TIM16_BASEADDR)
#define TIM17                           ((TIM_RegDef*) TIM17_BASEADDR)

/***********************************************************************/

/******PERIPHERAL DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT******/

#define RCC                             ((RCC_RegDef*) RCC_BASEADDR)

#define RCC_CR_MSION                    ((uint32_t) 0x40021000) 

#define SCB                             ((SCB_Type *) SCB_BASE) 

#define EXTI                            ((EXTI_RegDef*) EXTI_BASEADDR)

#define SYSCFG                          ((SYSCFG_RegDef*) SYSCFG_BASEADDR)

#define SYSTICK_BASE                    (0xE000E010UL)
    
#define SysTick                         ((SysTick_Type *) SYSTICK_BASE)

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS***************/

#define GPIOA_PCLK_EN()                 ( RCC->AHBENR |= (1<<17) )
#define GPIOB_PCLK_EN()                 ( RCC->AHBENR |= (1<<18) )
#define GPIOC_PCLK_EN()                 ( RCC->AHBENR |= (1<<19) )
#define GPIOD_PCLK_EN()                 ( RCC->AHBENR |= (1<<20) )
#define GPIOF_PCLK_EN()                 ( RCC->AHBENR |= (1<<22) )
/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS***************/

#define I2C1_PCLK_EN()                  ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()                  ( RCC->APB1ENR |= (1<<22) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR SPIx PERIPHERALS***************/

#define SPI1_PCLK_EN()                  ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()                  ( RCC->APB1ENR |= (1<<14) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR USARTx PERIPHERALS**************/

#define USART1_PCLK_EN()                ( RCC->APB2ENR |= (1<<14) )
#define USART2_PCLK_EN()                ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()                ( RCC->APB1ENR |= (1<<18) )
#define USART4_PCLK_EN()                ( RCC->APB1ENR |= (1<<19) )
#define USART5_PCLK_EN()                ( RCC->APB1ENR |= (1<<20) )
#define USART6_PCLK_EN()                ( RCC->APB2ENR |= (1<<5)  )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR TIMx PERIPHERALS***************/

#define TIM1_PCLK_EN()                  ( RCC->APB2ENR |= (1<<11) )
#define TIM3_PCLK_EN()                  ( RCC->APB1ENR |= (1<<1)  )
#define TIM6_PCLK_EN()                  ( RCC->APB1ENR |= (1<<4)  )
#define TIM7_PCLK_EN()                  ( RCC->APB1ENR |= (1<<5)  )
#define TIM14_PCLK_EN()                 ( RCC->APB1ENR |= (1<<8)  )
#define TIM15_PCLK_EN()                 ( RCC->APB2ENR |= (1<<16) )
#define TIM16_PCLK_EN()                 ( RCC->APB2ENR |= (1<<17) )
#define TIM17_PCLK_EN()                 ( RCC->APB2ENR |= (1<<18) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS***************/

#define SYSCFG_PCLK_EN()                ( RCC->APB2ENR |= (1<<0) )

/************************************************************************/

/***************CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS***************/

#define GPIOA_PCLK_DIS()                 ( RCC->AHBENR &= ~(1<<17) )
#define GPIOB_PCLK_DIS()                 ( RCC->AHBENR &= ~(1<<18) )
#define GPIOC_PCLK_DIS()                 ( RCC->AHBENR &= ~(1<<19) )
#define GPIOD_PCLK_DIS()                 ( RCC->AHBENR &= ~(1<<20) )
#define GPIOF_PCLK_DIS()                 ( RCC->AHBENR &= ~(1<<22) )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR SPIx PERIPHERALS***************/

#define SPI1_PCLK_DIS()                  ( RCC->APB2ENR  &= ~(1<<12) )
#define SPI2_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<14) )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS***************/

#define I2C1_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<22) )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR USARTx PERIPHERALS**************/

#define USART1_PCLK_DIS()                ( RCC->APB2ENR &= ~(1<<14) )
#define USART2_PCLK_DIS()                ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DIS()                ( RCC->APB1ENR &= ~(1<<18) )
#define USART4_PCLK_DIS()                ( RCC->APB1ENR &= ~(1<<19) )
#define USART5_PCLK_DIS()                ( RCC->APB1ENR &= ~(1<<20) )
#define USART6_PCLK_DIS()                ( RCC->APB2ENR &= ~(1<<5)  )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR TIMx PERIPHERALS***************/

#define TIM1_PCLK_DIS()                  ( RCC->APB2ENR &= ~(1<<11) )
#define TIM3_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<1) )
#define TIM6_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<4) )
#define TIM7_PCLK_DIS()                  ( RCC->APB1ENR &= ~(1<<5) )
#define TIM14_PCLK_DIS()                 ( RCC->APB1ENR &= ~(1<<8) )
#define TIM15_PCLK_DIS()                 ( RCC->APB2ENR &= ~(1<<16) )
#define TIM16_PCLK_DIS()                 ( RCC->APB2ENR &= ~(1<<17) )
#define TIM17_PCLK_DIS()                 ( RCC->APB2ENR &= ~(1<<18) )

/***********************************************************************/

/**************CLOCK DISABLE MACRO FOR SYSCFG PERIPHERALS**************/

#define SYSCFG_PCLK_DIS()                ( RCC->APB2ENR |= ~(1<<0) )

/***********************************************************************/

/*******************MACROS TO RESET GPIOx PERIPHERALS*******************/

#define GPIOA_REG_RESET()                do{ ( RCC->AHBRSTR |= (1<<17) ); ( RCC->AHBRSTR &= ~(1<<17) ); } while(0)
#define GPIOB_REG_RESET()                do{ ( RCC->AHBRSTR |= (1<<18) ); ( RCC->AHBRSTR &= ~(1<<18) ); } while(0)
#define GPIOC_REG_RESET()                do{ ( RCC->AHBRSTR |= (1<<19) ); ( RCC->AHBRSTR &= ~(1<<19) ); } while(0)
#define GPIOD_REG_RESET()                do{ ( RCC->AHBRSTR |= (1<<20) ); ( RCC->AHBRSTR &= ~(1<<20) ); } while(0)
#define GPIOF_REG_RESET()                do{ ( RCC->AHBRSTR |= (1<<22) ); ( RCC->AHBRSTR &= ~(1<<22) ); } while(0)

/***********************************************************************/

/*******************MACROS TO RESET SPIx PERIPHERALS*******************/

#define SPI1_REG_RESET()                 do{ ( RCC->APB2RSTR |= (1<<12) ); ( RCC->APB2RSTR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<14) ); ( RCC->APB1RSTR &= ~(1<<14) ); } while(0)

/***********************************************************************/

/*******************MACROS TO RESET I2Cx PERIPHERALS*******************/

#define I2C2_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<22) ); ( RCC->APB1RSTR &= ~(1<<22) ); } while(0)
#define I2C1_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<21) ); ( RCC->APB1RSTR &= ~(1<<21) ); } while(0)

/***********************************************************************/

/*******************MACROS TO RESET USARTx PERIPHERALS*******************/

#define USART1_REG_RESET()               do{ ( RCC->APB2RSTR |= (1<<14) ); ( RCC->APB2RSTR &= ~(1<<14) ); } while(0)
#define USART2_REG_RESET()               do{ ( RCC->APB1RSTR |= (1<<17) ); ( RCC->APB1RSTR &= ~(1<<17) ); } while(0)
#define USART3_REG_RESET()               do{ ( RCC->APB1RSTR |= (1<<18) ); ( RCC->APB1RSTR &= ~(1<<18) ); } while(0)
#define USART4_REG_RESET()               do{ ( RCC->APB1RSTR |= (1<<19) ); ( RCC->APB1RSTR &= ~(1<<19) ); } while(0)
#define USART5_REG_RESET()               do{ ( RCC->APB1RSTR |= (1<<20) ); ( RCC->APB1RSTR &= ~(1<<20) ); } while(0)
#define USART6_REG_RESET()               do{ ( RCC->APB2RSTR |= (1<<5) );  ( RCC->APB2RSTR &= ~(1<<5) );  } while(0)

/***********************************************************************/

/*******************MACROS TO RESET TIMx PERIPHERALS*******************/

#define TIM1_REG_RESET()                 do{ ( RCC->APB2RSTR |= (1<<11)); ( RCC->APB2RSTR &= ~(1<<11));} while(0)
#define TIM3_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<1) ); ( RCC->APB1RSTR &= ~(1<<1)); } while(0)
#define TIM6_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<4) ); ( RCC->APB1RSTR &= ~(1<<4)); } while(0)
#define TIM7_REG_RESET()                 do{ ( RCC->APB1RSTR |= (1<<5) ); ( RCC->APB1RSTR &= ~(1<<5)); } while(0)
#define TIM14_REG_RESET()                do{ ( RCC->APB1RSTR |= (1<<8));  ( RCC->APB1RSTR &= ~(1<<8)); } while(0)
#define TIM15_REG_RESET()                do{ ( RCC->APB2RSTR |= (1<<16)); ( RCC->APB2RSTR &= ~(1<<16));} while(0)
#define TIM16_REG_RESET()                do{ ( RCC->APB2RSTR |= (1<<17)); ( RCC->APB2RSTR &= ~(1<<17));} while(0)
#define TIM17_REG_RESET()                do{ ( RCC->APB2RSTR |= (1<<18)); ( RCC->APB2RSTR &= ~(1<<18));} while(0)

/***********************************************************************/

/************PORT CODE MACRO THAT RETURNS GPIOx BASE ADDRESS************/

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 :\
                                        (x == GPIOB) ? 1 :\
                                        (x == GPIOC) ? 2 :\
                                        (x == GPIOD) ? 3 :\
                                        (x == GPIOF) ? 4 :0)
                            
/***********************************************************************/

/*******************IRQ NUMBERS MACROS FOR EXTIx LINES******************/

//GPIO EXTI IRQ NUMBERS
#define IRQ_NO_EXTI0_1                  5
#define IRQ_NO_EXTI2_3                  6
#define IRQ_NO_EXTI4_15                 7

//SPI IRQ NUMBERS
#define IRQ_NO_SPI1                     25
#define IRQ_NO_SPI2                     26

//I2C IRQ NUMBERS
#define IRQ_NO_I2C1                     23
#define IRQ_NO_I2C2                     24

//UART/USART IRQ NUMBERS
#define IRQ_NO_USART1	                27
#define IRQ_NO_USART2	                28
#define IRQ_NO_USART3_4_5_6	            29

//TIMER IRQ NUMBERS
#define IRQ_NO_TIM1_BRK_UP_TRG_COM      13
#define IRQ_NO_TIM1_CC                  14
#define IRQ_NO_TIM3                     16
#define IRQ_NO_TIM6                     17
#define IRQ_NO_TIM14                    19
#define IRQ_NO_TIM15                    20
#define IRQ_NO_TIM16                    21
#define IRQ_NO_TIM17                    22

/***********************************************************************/

/***********************PROCESSOR SPECIFIC DETAILS**********************/

/*
ARM Cortex M0 Processor NVIC ISERx register Addresses
*/

#define NVIC_ISER                      ( (volatile uint32_t*) 0xE000E100 )

/*
ARM Cortex M0 Processor NVIC ICERx register Addresses
*/

#define NVIC_ICER                      ( (volatile uint32_t*) 0xE000E180 )

/*
ARM Cortex M0 Processor NVIC Priority register Address
*/

#define NVIC_PR_BASE_ADDR               ( (volatile uint32_t*) 0xE000E400 ) 

#define NO_PR_BITS_IMPLEMENTED          4

/***********************************************************************/

/*****************************GENERIC MACROS****************************/

#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define GPIO_PIN_RESET                  RESET
#define FLAG_SET 			            SET
#define FLAG_RESET                      RESET
#define __FPU_PRESENT                   1U 
#define __FPU_USED                      0U
#define FALSE                           0
#define TRUE                            1

/***********************************************************************/

#endif