
#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include "stm32f030.h"

/****************STRUCTURE FOR GPIOx PIN CONFIGURATION****************/

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;                /*<Possible values from @PERIPHERAL DEFINITIONS GPIOx MODER>*/
    uint8_t GPIO_PinSpeed;               /*<Possible values from @PERIPHERAL DEFINITIONS GPIOx OSPEEDR>*/
    uint8_t GPIO_PinPuPdControl;         /*<Possible values from @PERIPHERAL DEFINITIONS GPIOx PUPDR>*/
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;

}Gpio_PinConfig;

/*********************************************************************/

/********************STRUCTURE FOR GPIOx PIN HANDLE********************/

typedef struct
{
    GPIOx_RegDef *pGpiox;
    Gpio_PinConfig Gpio_PinConfig;

}GPIO_Handle;

/*********************************************************************/

/***********************DEFINITIONS PIN NUMBERS***********************/

#define GPIO_PIN_NO_0                0
#define GPIO_PIN_NO_1                1
#define GPIO_PIN_NO_2                2
#define GPIO_PIN_NO_3                3
#define GPIO_PIN_NO_4                4
#define GPIO_PIN_NO_5                5
#define GPIO_PIN_NO_6                6
#define GPIO_PIN_NO_7                7
#define GPIO_PIN_NO_8                8
#define GPIO_PIN_NO_9                9
#define GPIO_PIN_NO_10               10
#define GPIO_PIN_NO_11               11
#define GPIO_PIN_NO_12               12
#define GPIO_PIN_NO_13               13
#define GPIO_PIN_NO_14               14
#define GPIO_PIN_NO_15               15

/*********************************************************************/

/*****************PERIPHERAL DEFINITIONS GPIOx MODER******************/

#define GPIO_MODE_IN                 0  //INPUT MODE
#define GPIO_MODE_OUT                1  //OUTPUT MODE
#define GPIO_MODE_ALTFN              2  //ALTERNATE FUNCTION MODE
#define GPIO_MODE_ANALOG             3  //ANALOG MODE
#define GPIO_MODE_IT_FT              4  //INTERRUPT FALLING EDGE
#define GPIO_MODE_IT_RT              5  //INTERRUPT RISING EDGE
#define GPIO_MODE_IT_RFT             6  //INTERRUPT RISING FALLING TRIGGER

/*********************************************************************/

/*****************PERIPHERAL DEFINITIONS GPIOx OTYPER*****************/

#define GPIO_OP_TYPE_PP             0   //OUTPUT PUSH-PULL
#define GPIO_OP_TYPE_OD             1   //OUTPUT OPEN-DRAIN

/*********************************************************************/

/*****************PERIPHERAL DEFINITIONS GPIOx OSPEEDR*****************/

#define GPIO_SPEED_LOW              0   //LOW SPEED
#define GPIO_SPEED_MEDIUM           1   //MEDIUM SPEED
#define GPIO_SPEED_FAST             2   //HIGH SPEED
#define GPIO_SPEED_HIGH             3   //VERY HIGH SPEED

/*********************************************************************/

/*****************PERIPHERAL DEFINITIONS GPIOx PUPDR*****************/

#define GPIO_NO_PUPD                0   //NO PULL-UP, PULL-DOWN
#define GPIO_PIN_PU                 1   //PULL-UP
#define GPIO_PIN_PD                 2   //PULL-DOWN

/*********************************************************************/

//Peripheral Clock Setup
void GPIO_PeripheralControl(GPIOx_RegDef *pGpiox, uint8_t EnorDi);

//Init and Deinit
void GPIO_Init(GPIO_Handle *pGPIOHandle);
void GPIO_DeInit(GPIOx_RegDef *pGpiox);

//Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef *pGpiox);
void GPIO_WriteToOutputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIOx_RegDef *pGpiox, uint16_t Value);
void GPIO_ToggleOutputPin(GPIOx_RegDef *pGpiox, uint8_t PinNumber);

//IRQ Config and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif