#include "stm32f030.h"

//#define USE_FULL_ASSERT
//
//#ifdef USE_FULL_ASSERT
//#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
//#else
//#define assert_param(expr) ((void)0)
//#endif
//
///****************STRUCTURE FOR TIMx PIN CONFIGURATION****************/
//
//typedef struct 
//{
//    uint16_t Prescaler;
//    uint16_t Period;
//    uint8_t CounterMode;     // Up/Down/Center
//    uint8_t AutoReloadPreload;  // ENABLE/DISABLE
//
//} TIM_Config;
//
///*********************************************************************/
//
///********************STRUCTURE FOR I2Cx PIN HANDLE********************/
//
//typedef struct 
//{
//    TIM_RegDef *pTIMx;
//    TIM_Config TIM_Config;
//
//} TIM_Handle;
//
///*********************************************************************/
//
///*********************AUTO RELOAD-PRELOAD STATUS**********************/
//
//#define TIM_ENABLE_RELOAD                   SET
//#define TIM_DISABLE_RELOAD                  RESET
//
///*********************************************************************/
//
//
///********************COUNTER MODE STATUS (UP/DOWN)********************/
//
//#define TIM_CENTER_MODE_1_COUNTER           1
//#define TIM_CENTER_MODE_2_COUNTER           2
//#define TIM_CENTER_MODE_3_COUNTER           3
//#define TIM_UP_COUNTER                      4
//#define TIM_DOWN_COUNTER                    5
//
///*********************************************************************/
//
///********************CR1 REGISTER BIT DEFINITIONS*********************/
//
//#define CEN                                 0
//#define UDIS                                1
//#define URS                                 2
//#define OPM                                 3
//#define DIR                                 4
//#define CMS                                 5
//#define ARPE                                7
//#define CKD                                 8
//#define UIFREMAP                            11
//
///*********************************************************************/
//
///*****************CR2/DIER REGISTER BIT DEFINITIONS*******************/
//
//#define UIE                                 0
//#define UDE                                 8
//
///*********************************************************************/
//
///********************TIM STATUS FLAGS DEFINITIONS*********************/
//
//#define TIM_READY                           0
//#define TIM_BUSY                            1
//#define TIM_ERROR                           2
//
///*********************************************************************/
//
//
////Peripheral Clock Setup
//void TIM_PeriClockControl(TIM_RegDef *pTIMx, uint8_t EnorDi);
//
////Init and Deinit
//void TIM_Init(TIM_Handle *pTIMHandle);     
//void TIM_DeInit(TIM_RegDef *pTIMx);
//
////Start and Stop the timer
//void TIM_Start(TIM_Handle *pTIMHandle);         
//void TIM_Stop(TIM_Handle *pTIMHandle); 
//
////Start and Stop the timer using interrupts
//uint8_t TIM_Start_IT(TIM_Handle *pTIMHandle);
//uint8_t TIM_Stop_IT(TIM_Handle *pTIMHandle);
//
//void TIM_DelayMs(TIM_RegDef *pTIMx, uint32_t ms);   // Blocking delay using polling
//void TIM_SetTimeBase(TIM_RegDef *pTIMx, uint16_t prescaler, uint32_t period); 
//
////IRQ Config and ISR Handling
//void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
//void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
//void TIM_IRQHandling(TIM_Handle *pHandle);
//
////Counter based APIs
//uint32_t TIM_GetCounter(TIM_RegDef *pTIMx);
//void TIM_SetCounter(TIM_RegDef *pTIMx, uint32_t value);
//
////Flag status APIs
//uint8_t TIM_GetFlagStatus(TIM_RegDef *pTIMx, uint32_t Flag);
//void TIM_ClearFlag(TIM_RegDef *pTIMx, uint32_t Flag);
//
//void TIM_PWM_Init(TIM_Handle *pTIMHandle);    // Sets up mode, CCR, polarity
//void TIM_PWM_Start(TIM_RegDef *pTIMx, uint8_t Channel);
//void TIM_PWM_Stop(TIM_RegDef *pTIMx, uint8_t Channel);
//void TIM_SetDutyCycle(TIM_RegDef *pTIMx, uint8_t Channel, float duty_percent);
//
//void TIM_IC_Init(TIM_Handle *pTIMHandle);
//uint32_t TIM_IC_ReadCapture(TIM_RegDef *pTIMx, uint8_t Channel);

#include <stdint.h>

void TIM14_Init_30s(void);
void TIM3_Init_10s(void);

#define RCC_BASE        0x40021000
#define TIM14_BASE       0x40002000
#define TIM3_BASE        0x40000400

#define RCC_APB1ENR      (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define TIM14_PSC        (*(volatile uint32_t *)(TIM14_BASE + 0x28))
#define TIM14_ARR        (*(volatile uint32_t *)(TIM14_BASE + 0x2C))
#define TIM14_DIER       (*(volatile uint32_t *)(TIM14_BASE + 0x0C))
#define TIM14_CR1        (*(volatile uint32_t *)(TIM14_BASE + 0x00))
#define TIM14_SR         (*(volatile uint32_t *)(TIM14_BASE + 0x10))
#define TIM14_EGR        (*(volatile uint32_t *)(TIM14_BASE + 0x14))

#define TIM3_CR1         (*(volatile uint32_t *)(TIM3_BASE + 0x00))
#define TIM3_DIER        (*(volatile uint32_t *)(TIM3_BASE + 0x0C))
#define TIM3_SR          (*(volatile uint32_t *)(TIM3_BASE + 0x10))
#define TIM3_EGR         (*(volatile uint32_t *)(TIM3_BASE + 0x14))
#define TIM3_PSC         (*(volatile uint32_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR         (*(volatile uint32_t *)(TIM3_BASE + 0x2C))
