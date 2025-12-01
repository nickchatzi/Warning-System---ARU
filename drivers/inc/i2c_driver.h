
#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "stm32f030.h"

/****************STRUCTURE FOR I2Cx PIN CONFIGURATION****************/

typedef struct
{
    uint8_t I2C_Freq;
    uint8_t I2C_Mode;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_FMDutyCycle;
    uint8_t I2C_NoStretch;
    uint8_t I2C_OwnAddressMode;
    uint8_t I2C_SlaveAddressMode;

}I2C_Config;

/*********************************************************************/

/********************STRUCTURE FOR I2Cx PIN HANDLE********************/

typedef struct
{
    I2C_RegDef  *pI2Cx;         //Holds the base address (x:1,2,3) peripheral
    I2C_Config  I2C_Config;
    uint8_t     *pTxBuffer;     //To store the application Tx buffer address    
    uint8_t     *pRxBuffer;     //To store the application Rx buffer address
    uint32_t    TxLen;          //To store Tx length
    uint32_t    RxLen;          //To store Rx length
    uint8_t     TxRxState;      //To store communication state
    uint8_t     DevAddr;        //To store slave device address
    uint32_t    RxSize;         //To store Rx size
    uint8_t     Sr;             //To store repeated start value

}I2C_Handle;

/*********************************************************************/

/**********************FREQ CONFIGURATION MACROS**********************/

#define I2C_BUS_10KHZ                       0
#define I2C_BUS_100KHZ                      1
#define I2C_BUS_200KHZ                      2
#define I2C_BUS_400KHZ                      3
#define I2C_BUS_1000KHZ                     4

/*********************************************************************/

/**********************MODE CONFIFURATION MACROS**********************/

#define I2C_SCL_SPEED_SM                    0
#define I2C_SCL_SPEED_FM                    1
#define I2C_SCL_SPEED_FMP                   2

/*********************************************************************/

/************************SR CONFIFURATION MACROS**********************/

#define I2C_DISABLE_SR  	                RESET
#define I2C_ENABLE_SR   	                SET

/*********************************************************************/

/********************NO STRETCH CONFIFURATION MACROS******************/

#define I2C_ENABLE_NO_STRETCH     	        RESET
#define I2C_DISABLE_NO_STRETCH  	        SET

/*********************************************************************/

/*****************ADDRESS MODE CONFIFURATION MACROS*******************/

#define I2C_7BIT_ADDRESS_MODE  	            0
#define I2C_10BIT_ADDRESS_MODE  	        1

/*********************************************************************/

/********************CR1 REGISTER BIT DEFINITIONS*********************/

#define I2C_PE                                  0     
#define I2C_TXIE                                1
#define I2C_RXIE                                2
#define I2C_ADDRIE                              3
#define I2C_NACKIE                              4
#define I2C_STOPIE                              5
#define I2C_TCIE                                6
#define I2C_ERRIE                               7
#define I2C_DNF                                 8
#define I2C_ANFOFF                              12
#define I2C_TXDMAEN                             14
#define I2C_RXDMAEN                             15
#define I2C_SCB_I2C                             16
#define I2C_NOSTRETCH                           17

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define I2C_SADD                                0    
#define I2C_RD_WRN                              10
#define I2C_ADD10                               11
#define I2C_HEAD10R                             12
#define I2C_START                               13
#define I2C_STOP                                14
#define I2C_NACK                                15
#define I2C_NBYTES                              16
#define I2C_RELOAD                              24
#define I2C_AUTOEND                             25
#define I2C_PECBYTE                             26

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define I2C_OA1                                 0    
#define I2C_OA1MODE                             10
#define I2C_OA1EN                               15

/*********************************************************************/

/******************TIMINGR REGISTER BIT DEFINITIONS*******************/

#define I2C_SCLL                                0    
#define I2C_SCLH                                8
#define I2C_SDADEL                              16
#define I2C_SCLDEL                              20
#define I2C_RES                                 24
#define I2C_PRESC                               28

/*********************************************************************/

/********************ISR REGISTER BIT DEFINITIONS*********************/

#define I2C_TXE                                 0    
#define I2C_TXIS                                1
#define I2C_RXNE                                2
#define I2C_ADDR                                3
#define I2C_NACKF                               4
#define I2C_STOPF                               5
#define I2C_TC                                  6
#define I2C_TCR                                 7
#define I2C_BERR                                8
#define I2C_ARLO                                9
#define I2C_OVR                                 10
#define I2C_PECERR                              11
#define I2C_TIMEOUT                             12
#define I2C_ALERT                               13
#define I2C_BUSY                                15
#define I2C_DIR                                 16
#define I2C_ADDCODE                             17

/*********************************************************************/

/********************ICR REGISTER BIT DEFINITIONS*********************/

#define I2C_ADDRCF                                3    
#define I2C_NACKCF                                4
#define I2C_STOPCF                                5
#define I2C_BERRCF                                8
#define I2C_ARLOCF                                9
#define I2C_OVRCF                                 10
#define I2C_PECCF                                 11
#define I2C_TIMOUTCF                              12
#define I2C_ALERTCF                               13

/*********************************************************************/

/****************RXDR, TXDR REGISTER BIT DEFINITIONS******************/

#define I2C_RXDATA                                0
#define I2C_RTDATA                                0

/*********************************************************************/

/*****************************I2C EVENTS******************************/

#define I2C_EVENT_RX_CMPLT                    1
#define I2C_EVENT_TX_CMPLT                    2
#define I2C_EVENT_STOP                        3
#define I2C_EVENT_TCR                         4
#define I2C_EVENT_TC                          5
#define I2C_EVENT_ADDR                        6
#define I2C_EVENT_NACKF                       7

/*********************************************************************/

/*****************************I2C ERRORS******************************/

#define I2C_EVENT_BERR                        1
#define I2C_EVENT_ARLO                        2
#define I2C_EVENT_OVR                         3
#define I2C_EVENT_PECERR                      4
#define I2C_EVENT_TIMEOUT                     5
#define I2C_EVENT_ALERT                       6

/*********************************************************************/

/********************I2C STATUS FLAGS DEFINITIONS*********************/

#define I2C_FLAG_TXE                        ( 1 << TXE)     //status to check if TX buffer is empty or not.
#define I2C_FLAG_RXNE                       ( 1 << RXNE)    //status to check if RX buffer is empty or not.
#define I2C_FLAG_ADDR                       ( 1 << ADDR)    //status to check if the received slave address matched with one of the enabled slave addresses.
#define I2C_FLAG_NACKF                      ( 1 << NACKF)   //status to check if a NACK is received after a byte transmission.
#define I2C_FLAG_STOPF                      ( 1 << STOPF)   //status to check if a STOP condition is detected on the bus.
#define I2C_FLAG_TC                         ( 1 << TC)      //status to check if transfer is completed.
#define I2C_FLAG_BERR                       ( 1 << BERR)    //status to check when a misplaced START or STOP condition is detected.
#define I2C_FLAG_ARLO                       ( 1 << ARLO)    //status to check in case of arbitration loss.
#define I2C_FLAG_OVR                        ( 1 << OVR)     //status to check when an overrun/underrun error occurs. (ONLY for slave mode with NOSTRETCH = 1).
#define I2C_FLAG_TIMEOUT                    ( 1 << TIMEOUT) //status to check when a timeout or extended clock timeout occurred.
#define I2C_FLAG_BUSY                       ( 1 << BUSY)    //status to check that a communication is in progress on the bus (bus is busy)
/*********************************************************************/

/********************I2C STATUS FLAGS DEFINITIONS*********************/

#define I2C_READY                           0
#define I2C_BUSY_IN_RX                      1
#define I2C_BUSY_IN_TX                      2
#define I2C_ISR_BUSY                        4

/*********************************************************************/

#define I2C_DISABLE_SR  	RESET	// No Repeated Start: Stop condition will be sent
#define I2C_ENABLE_SR   	SET		// Repeated Start: No Stop condition will be sent

//Enable or Disable Peripheral
void I2C_PeripheralControl(I2C_RegDef *pI2Cx, uint8_t EnorDi);

//Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef *pI2Cx, uint8_t EnorDi);

//Init and Deinit
void I2C_Init(I2C_Handle *pI2CHandle);
void I2C_DeInit(I2C_RegDef *pI2Cx);

//Get status from the transmission
uint8_t I2C_GetFlagStatus(I2C_RegDef *pI2Cx , uint32_t FlagName);

//Send data
void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr);

//Receive data
void I2C_SlaveSendData(I2C_RegDef *pI2Cx ,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef *pI2Cx);

//send and receive data using interrupts
uint8_t  I2C_MasterSendDataIT(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr);
uint8_t  I2C_MasterReceiveDataIT(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr);

//IRQ Config and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle *pI2CHandle);

//Application Callback
void I2CApplicationEventCallback(I2C_Handle *pI2CHandle, uint8_t AppEv);

#endif