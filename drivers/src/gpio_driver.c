#include "gpio_driver.h"

/**************************Peripheral Clock Setup*********************************
 
 * @fn          -GPIO_PeripheralControl
 * 
 * @brief       -This function enables or disables peripheral clock for the given GPIO port
 * 
 * @param[in]   -base address of the gpio peripheral
 * @param[in]   -ENABLE or DISABLE macros
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_PeripheralControl(GPIOx_RegDef *pGpiox, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (pGpiox == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGpiox == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGpiox == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGpiox == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGpiox == GPIOF)
        {
            GPIOF_PCLK_EN();
        }                                     
    }
    else
    {
        if (pGpiox == GPIOA)
        {
            GPIOA_PCLK_DIS();
        }
        else if (pGpiox == GPIOB)
        {
            GPIOB_PCLK_DIS();
        }
        else if (pGpiox == GPIOC)
        {
            GPIOC_PCLK_DIS();
        }
        else if (pGpiox == GPIOD)
        {
            GPIOD_PCLK_DIS();
        }
        else if (pGpiox == GPIOF)
        {
            GPIOF_PCLK_DIS();
        }                   
    }
}

/*************************************************************************/

/********************************Init**************************************
 
 * @fn          -GPIO_Init
 * 
 * @brief       -This function initializes the GPIO
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void GPIO_Init(GPIO_Handle *pGPIOHandle)
{
    uint32_t temp = 0;

     /*Enable the peripheral clock of used GPIO*/
     
    GPIO_PeripheralControl(pGPIOHandle->pGpiox, ENABLE);

    /*This if statement will configure the mode of gpio pin*/

    if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
                
        temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode) << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
        
        pGPIOHandle->pGpiox->MODER &= ~ (0x3 << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );  //clear the bits first
        
        pGPIOHandle->pGpiox->MODER |= temp;  //store the value of temp into the actual register.
        
    }
    else
    {
        pGPIOHandle->pGpiox->MODER &= ~(0x3 << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber));  // Clear bits to set as input

        if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            EXTI->FTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
            
            EXTI->RTSR &= ~(1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            EXTI->RTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
            
            EXTI->FTSR &= ~(1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            EXTI->RTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
            
            EXTI->FTSR |= (1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber);
        }

        /*Configure the GPIO port selection in SYCFG_EXTICR*/
        uint8_t temp1 = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber / 4;    
        uint8_t temp2 = pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber % 4;     

        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGpiox);  

        SYSCFG_PCLK_EN();

        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

        /*Enable the EXTI interrupt delivery using IMR*/
        EXTI->IMR |= 1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber;
    }

    /*Configure the speed*/

    temp = 0;

    temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );

    pGPIOHandle->pGpiox->OSPEEDR &= ~ (0x3 << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );  //clear the bits first
    
    pGPIOHandle->pGpiox->OSPEEDR |= temp;    //store the value of temp into the actual register.

    temp = 0;

    /*Configure the pupd settings*/

    temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );

    pGPIOHandle->pGpiox->PUPDR &= ~ (0x3 << (2 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );  //clear the bits first
    
    pGPIOHandle->pGpiox->PUPDR |= temp;

    temp = 0;

     /*Configure the optype*/ 
     
     temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinOPType << (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );

     pGPIOHandle->pGpiox->OTYPER &= ~ (0x1 << pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber );  //clear the bits first
     
     pGPIOHandle->pGpiox->OTYPER |= temp;

     temp = 0;

     /*Configure the alt functionality*/ 
    #if 1

    if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
     {
        if (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber <= GPIO_PIN_NO_7)
        {
            temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber ) );

            pGPIOHandle->pGpiox->AFR[0] &= ~ (0xF << (4 * pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber) );

            pGPIOHandle->pGpiox->AFR[0] |= temp;
        }
        else
        {
            temp = (pGPIOHandle->Gpio_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber - 8) ) );

            pGPIOHandle->pGpiox->AFR[1] &= ~ (0xF << (4 * (pGPIOHandle->Gpio_PinConfig.GPIO_PinNumber - 8) ) );
            
            pGPIOHandle->pGpiox->AFR[1] |= temp;      
        }
        temp = 0; 

    #else    
        if (pGPIOHandle->Gpio_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
        {
            uint8_t temp1, temp2;

            temp1 = pGPIOHandle-> Gpio_PinConfig.GPIO_PinNumber / 8;
            temp2 = pGPIOHandle-> Gpio_PinConfig.GPIO_PinNumber % 8;

            pGPIOHandle->pGpiox->AFR[temp1] &= ~(0xF << (4*temp2));
            pGPIOHandle->pGpiox->AFR[temp1] |= (pGPIOHandle->Gpio_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );

    }
    #endif 
     }
}
/***************************************************************************/

/*******************************Deinit**************************************
 
 * @fn          -GPIO_Deinit
 * 
 * @brief       -This function de-initializes the GPIO and resets all the registors of a peripheral.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_DeInit(GPIOx_RegDef *pGpiox)
{
    if (pGpiox == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGpiox == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGpiox == GPIOC)
    {
        GPIOC_REG_RESET();    
    }
    else if (pGpiox == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGpiox == GPIOF)
    {
        GPIOF_REG_RESET();
    }  
}
/*********************************************************************************/

/****************************Data Read From Pin***********************************
 
 * @fn          -GPIO_ReadFromInputPin
 * 
 * @brief       -This function will read data from a pin set as input and return that data.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -Pin number of the selected GPIO
 * @param[in]   -
 * 
 * @return      -0 or 1
 * 
 * @note        -none
 
*/
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber)
{
uint8_t value = 0;

value = (uint8_t)( ( pGpiox->IDR >> PinNumber) & 0x00000001 );

return value;
}
/**********************************************************************************/

/****************************Data Read From Port***********************************
 
 * @fn          -GPIO_ReadFromInputPort
 * 
 * @brief       -This function will read data from a port set as input and return that data.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -0 or 1
 * 
 * @note        -none
 
*/

uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef *pGpiox)
{
    uint16_t value = 0;

    value = (uint16_t) pGpiox->IDR;
    
    return value;    
}
/*******************************************************************************/

/****************************Data Write To Pin***********************************
 
 * @fn          -GPIO_WriteToOutputPin
 * 
 * @brief       -This function will write data to a gpio pin set as output.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -Pin number of the selected GPIO
 * @param[in]   -Value to write (0 or 1)
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_WriteToOutputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGpiox->ODR |= ( 1 << PinNumber );
    }
    else
    {
        pGpiox->ODR &= ~( 1 << PinNumber );
    }
}
/********************************************************************************/

/****************************Data Write To Port***********************************
 
 * @fn          -GPIO_WriteToOutputPort
 * 
 * @brief       -This function will write data to a gpio port set as output.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -Value to write (0 or 1)
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_WriteToOutputPort(GPIOx_RegDef *pGpiox, uint16_t Value)
{  
    pGpiox->ODR = Value;
}
/*******************************************************************************/

/****************************Toggle Output Pin***********************************
 
 * @fn          -GPIO_ToggleOutputPin
 * 
 * @brief       -This function will toggle a gpio pin set as output.
 * 
 * @param[in]   -Base address of the gpio port
 * @param[in]   -Pin number of the selected GPIO
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_ToggleOutputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber)
{
   pGpiox->ODR ^= (1 << PinNumber);
}

/*********************************************************************************/

/***************************IRQ Interrupt Config***********************************
 
 * @fn          -GPIO_IRQInterruptConfig
 * 
 * @brief       -Configure the NVIC registers to either enable or disable a given interrupt.
 * 
 * @param[in]   -interrupt number that needs to be enabled or disabled.
 * @param[in]   -ENABLE or DISABLE interrupt macros.
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(IRQNumber < 32)  // Only IRQ numbers 0–31 are valid on Cortex-M0
    {
        if(EnorDi == ENABLE)
        {
            *NVIC_ISER |= (1 << IRQNumber);
        }
        else
        {
            *NVIC_ICER |= (1 << IRQNumber);
        }
    }
}

/********************************************************************************/

/***************************IRQ Priority Config***********************************
 
 * @fn          -GPIO_IRQPriorityConfig
 * 
 * @brief       -Configure the priority for the given interrupt.
 * 
 * @param[in]   -interrupt number that needs to be enabled or disabled.
 * @param[in]   -Set the priority
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;  
    uint8_t iprx_section = IRQNumber % 4;  
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);  

    //*(NVIC_PR_BASE_ADDR + (iprx * 4)) &= ~(0xFF << shift_amount); //Clear previous priority
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount ); 
}
/********************************************************************************/

/***************************IRQ Handling Config***********************************
 
 * @fn          -GPIO_IRQHandling
 * 
 * @brief       -Configure the priority for the given interrupt
 * 
 * @param[in]   -Pin number of the selected GPIO.
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (EXTI->PR & (1 << PinNumber))
    {
        EXTI->PR |= (1 << PinNumber);
    }
}
/********************************************************************************/