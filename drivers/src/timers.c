#include "timers.h"

///************************** Peripheral Clock Setup *********************************
// 
// * @fn          -TIM_PeriClockControl
// * 
// * @brief       -This function enables or disables peripheral clock for the given timer port
// * 
// * @param[in]   -base address of the timer peripheral
// * @param[in]   -ENABLE or DISABLE macros
// * @param[in]   -
// * 
// * @return      -void
// * 
// * @note        -none
// 
//*/
//
//void TIM_PeriClockControl(TIM_RegDef *pTIMx, uint8_t EnorDi)
//{
//    if(EnorDi == ENABLE)
//    {
//        if (pTIMx == TIM1)
//        {
//            TIM1_PCLK_EN();
//        }
//        else if (pTIMx == TIM3)
//        {
//            TIM3_PCLK_EN();
//        }    
//        else if (pTIMx == TIM6)
//        {
//            TIM6_PCLK_EN();
//        }    
//        else if (pTIMx == TIM7)
//        {
//            TIM7_PCLK_EN();
//        }    
//        else if (pTIMx == TIM14)
//        {
//            TIM14_PCLK_EN();
//        }    
//        else if (pTIMx == TIM15)
//        {
//            TIM15_PCLK_EN();
//        }    
//        else if (pTIMx == TIM16)
//        {
//            TIM16_PCLK_EN();
//        }    
//        else if (pTIMx == TIM17)
//        {
//            TIM17_PCLK_EN();
//        }                              
//    }
//    else
//    {
//        if (pTIMx == TIM1)
//        {
//            TIM1_PCLK_DIS();
//        }
//        else if (pTIMx == TIM3)
//        {
//            TIM3_PCLK_DIS();
//        } 
//        else if (pTIMx == TIM6)
//        {
//            TIM6_PCLK_DIS();
//        }  
//        else if (pTIMx == TIM7)
//        {
//            TIM7_PCLK_DIS();
//        }  
//        else if (pTIMx == TIM14)
//        {
//            TIM14_PCLK_DIS();
//        }  
//        else if (pTIMx == TIM15)
//        {
//            TIM15_PCLK_DIS();
//        }  
//        else if (pTIMx == TIM16)
//        {
//            TIM16_PCLK_DIS();
//        }  
//        else if (pTIMx == TIM17)
//        {
//            TIM17_PCLK_DIS();
//        }             
//    }
//}
///**********************************************************************************/
//
///**********************************************************************************/
//
///************************************* Init ****************************************
// 
// * @fn          -TIM_Init
// * 
// * @brief       -This function initializes the timer functionality
// * 
// * @param[in]   -Base address of the TIM port
// * @param[in]   -
// * @param[in]   -
// * 
// * @return      -void
// * 
// * @note        -none
// 
//*/
//
///*
//void TIM_Init(TIM_Handle *pTIMHandle)
//{
//    // Check validity
//	assert_param(pTIMHandle->TIM_Config.Prescaler <= 65535);
//    assert_param(pTIMHandle->TIM_Config.Period <= 65535);
//
//    //Configure the Prescaler
//    pTIMHandle->pTIMx->PSC = pTIMHandle->TIM_Config.Prescaler;
//
//    //Configure the ARR
//    pTIMHandle->pTIMx->ARR = pTIMHandle->TIM_Config.Period;
//
//    //Configure the Counter Mode (Up/Down/Center)
//    if (pTIMHandle->pTIMx != TIM6 && pTIMHandle->pTIMx != TIM7)
//    {
//        // Reset DIR and CMS bits
//        pTIMHandle->pTIMx->CR1 &= ~(1 << DIR);   // DIR
//        pTIMHandle->pTIMx->CR1 &= ~(3 << CMS);   // CMS
//
//        switch (pTIMHandle->TIM_Config.CounterMode)
//        {
//            case TIM_UP_COUNTER:
//                break;
//        
//            case TIM_DOWN_COUNTER:
//                pTIMHandle->pTIMx->CR1 |= (1 << DIR);  
//                break;
//        
//            case TIM_CENTER_MODE_1_COUNTER:
//            case TIM_CENTER_MODE_2_COUNTER:
//            case TIM_CENTER_MODE_3_COUNTER:
//                pTIMHandle->pTIMx->CR1 |= (pTIMHandle->TIM_Config.CounterMode << CMS); // CMS = x
//                break;
//    }
// }
//
//    //Configure the Auto-reload preload mode (APRE)
//    if (pTIMHandle->TIM_Config.AutoReloadPreload == ENABLE)
//    {
//        pTIMHandle->pTIMx->CR1 |= (1 << ARPE);
//    }
//    else
//    {
//        pTIMHandle->pTIMx->CR1 &= ~(1 << ARPE);
//    }
//}  
///**********************************************************************************/
//
///******************************* Deinit ********************************************
// 
// * @fn          -TIM_DeInit
// * 
// * @brief       -This function de-initializes the TIM and resets all the registers of a peripheral.
// * 
// * @param[in]   -Base address of the TIM port
// * @param[in]   -
// * @param[in]   -
// * 
// * @return      -void
// * 
// * @note        -none
// 
//*/
//void TIM_DeInit(TIM_RegDef *pTIMx)
//{
//    if (pTIMx == TIM1)
//    {
//        TIM1_REG_RESET();
//    }
//    else if (pTIMx == TIM3)
//    {
//        TIM3_REG_RESET();
//    }
//    else if (pTIMx == TIM6)
//    {
//        TIM6_REG_RESET();
//    }
//    else if (pTIMx == TIM7)
//    {
//        TIM7_REG_RESET();
//    }
//    else if (pTIMx == TIM14)
//    {
//        TIM14_REG_RESET();
//    }
//    else if (pTIMx == TIM15)
//    {
//        TIM15_REG_RESET();
//    }
//    else if (pTIMx == TIM16)
//    {
//        TIM16_REG_RESET();
//    }
//    else if (pTIMx == TIM17)
//    {
//        TIM17_REG_RESET();
//    }
//}
///**********************************************************************************/
//
///******************************* START TIMER **************************************
// 
// * @fn          -TIM_Start
// * 
// * @brief       -This function will start the counter.
// * 
// * @param[in]   -Base address of the timer port.
// * @param[in]   -
// * 
// * @return      -void
// * 
// * @note        -none
// 
//*/
//void TIM_Start(TIM_Handle *pTIMHandle)
//{
//    pTIMHandle->pTIMx->CR1 |= (1 << CEN);
//
//}    
///**********************************************************************************/
//
///******************************* STOP TIMER **************************************
// 
// * @fn          -TIM_Stop
// * 
// * @brief       -This function will stop the counter.
// * 
// * @param[in]   -Base address of the timer port.
// * @param[in]   -
// * 
// * @return      -void
// * 
// * @note        -none
// 
//*/
//void TIM_Stop(TIM_Handle *pTIMHandle)
//{
//    pTIMHandle->pTIMx->CR1 &= ~(1 << CEN);
//}
///**********************************************************************************/
//
///********************** Start the timer using Interrupt *****************************
// 
// * @fn          -TIM_Start_IT
// * 
// * @brief       -
// * 
// * @param[in]   -Base address of the I2C peripheral
// * @param[in]   -
// * 
// * @return      -State of the timer
// * 
// * @note        -Non-blocking Call
// 
//*/
//
//uint8_t TIM_Start_IT(TIM_Handle *pTIMHandle)
//{
//    if (pTIMHandle == NULL) return TIM_ERROR;
//
//    // Optional: check if already running
//    if (pTIMHandle->pTIMx->CR1 & (1 << CEN))
//        return TIM_BUSY;
//
//    // Enable update interrupt
//    pTIMHandle->pTIMx->DIER |= (1 << UIE);
//
//    // Enable counter
//    pTIMHandle->pTIMx->CR1 |= (1 << CEN);
//
//    return TIM_READY;
//}
///**********************************************************************************/
//
///********************** Stop the timer using Interrupt *****************************
// 
// * @fn          -TIM_Stop_IT
// * 
// * @brief       -
// * 
// * @param[in]   -Base address of the I2C peripheral
// * @param[in]   -
// * 
// * @return      -State of the timer
// * 
// * @note        -Non-blocking Call
// 
//*/
//
//uint8_t TIM_Stop_IT(TIM_Handle *pTIMHandle)
//{
//    if (pTIMHandle == NULL)
//        return TIM_ERROR;
//
//    // Optional: Check if already stopped
//    if (!(pTIMHandle->pTIMx->CR1 & (1 << CEN)))
//        return TIM_BUSY;  // Already stopped
//
//    // Disable counter
//    pTIMHandle->pTIMx->CR1 &= ~(1 << CEN);
//
//    // Disable update interrupt
//    pTIMHandle->pTIMx->DIER &= ~(1 << UIE);
//
//    return TIM_READY;
//}
///**********************************************************************************/
//
//
//void TIM_DelayMs(TIM_RegDef *pTIMx, uint32_t ms);   // Blocking delay using polling
//void TIM_SetTimeBase(TIM_RegDef *pTIMx, uint16_t prescaler, uint32_t period); 
//
////IRQ Config and ISR Handling
//void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
//{
//
//}
//void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
//{
//
//}
//void TIM_IRQHandling(TIM_Handle *pHandle)
//{
//    
//}
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
//
//static void assert_failed(uint8_t *file, uint32_t line)
//{
//    while(1);
//}

void TIM14_Init_30s(void)
{
    RCC_APB1ENR |= (1 << 8);

    TIM14_CR1 = 0;      // make sure timer is OFF
    TIM14_DIER = 0;      // no interrupts yet

    TIM14_PSC = 8000 - 1;
    TIM14_ARR = 30000 - 1;

    TIM14_SR = 0;        // clear flags
    TIM14_EGR = 1;        // update event

    TIM14_DIER = 1;       // enable update interrupt
    TIM14_CR1  = 1;       // start timer
}

void TIM3_Init_10s(void)
{
    RCC_APB1ENR |= (1 << 1);  // TIM3 clock enable
    TIM3_SR = 0; 
    TIM3_CR1 = 0;             // timer off
    TIM3_DIER = 0;            // disable interrupts

    TIM3_PSC = 8000 - 1;      // 1 kHz tick (1 ms)
    TIM3_ARR = 10000 - 1;     // 10 seconds = 10000 ms

    TIM3_SR = 0;              // clear flags
    TIM3_EGR = 1;             // update event

    TIM3_DIER = 1;            // enable update interrupt
    TIM3_CR1  = 1;            // start timer
}


