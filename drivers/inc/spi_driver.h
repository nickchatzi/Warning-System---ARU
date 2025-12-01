
#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include "stm32f030.h"

/****************STRUCTURE FOR SPIx PIN CONFIGURATION****************/

typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;

}SPI_Config;

/*********************************************************************/

/********************STRUCTURE FOR SPIx PIN HANDLE********************/

typedef struct
{
    SPI_RegDef  *pSPIx;         //Holds the base address (x:1,2,3) peripheral
    SPI_Config  SPIConfig;
    uint8_t     *pTxBuffer;     //To store the application Tx buffer address.
    uint8_t     *pRxBuffer;     //To store the application Rx buffer address.
    uint32_t    TxLen;          //To store Tx Length
    uint32_t    RxLen;          //To store Rx Length
    uint8_t     TxState;        //To store Tx State
    uint8_t     RxState;        //To store Rx State

}SPI_Handle;

/*********************************************************************/

/*************************DEVICE MODE MACROS**************************/

#define SPI_DEVICE_MODE_SLAVE               0
#define SPI_DEVICE_MODE_MASTER              1

/*********************************************************************/

/*********************BUS CONFIGURATION MACROS*************************/

#define SPI_BUS_CONFIG_FD                   1
#define SPI_BUS_CONFIG_HD                   2
#define SPI_BUS_CONFIG_SIMPLEX_RX           3

/*********************************************************************/


/*********************SERIAL CLOCK SPEED MACROS************************/

#define SPI_SCLK_SPEED_DIV2                 0
#define SPI_SCLK_SPEED_DIV4                 1
#define SPI_SCLK_SPEED_DIV8                 2
#define SPI_SCLK_SPEED_DIV16                3
#define SPI_SCLK_SPEED_DIV32                4
#define SPI_SCLK_SPEED_DIV64                5
#define SPI_SCLK_SPEED_DIV128               6
#define SPI_SCLK_SPEED_DIV256               7

/*********************************************************************/

/*****************************DFF MACROS******************************/

#define SPI_DFF_8BITS                       0
#define SPI_DFF_16BITS                      1

/*********************************************************************/

/*****************************CPOL MACROS*****************************/

#define SPI_CPOL_LOW                        0
#define SPI_CPOL_HIGH                       1

/*********************************************************************/

/*****************************CPHA MACROS*****************************/

#define SPI_CPHA_LOW                        0
#define SPI_CPHA_HIGH                       1

/*********************************************************************/

/*****************************SSM MACROS******************************/

#define SPI_SSM_DIS                         0
#define SPI_SSM_EN                          1

/*********************************************************************/

/*****************************SPI STATES******************************/

#define SPI_READY 					        0
#define SPI_BUSY_IN_RX 				        1
#define SPI_BUSY_IN_TX 				        2

/*********************************************************************/


/*****************************SPI EVENTS******************************/

#define SPI_EVENT_TX_CMPLT                  1
#define SPI_EVENT_RX_CMPLT                  2
#define SPI_EVENT_MODF_ERR                  3
#define SPI_EVENT_OVR_ERR                   4
#define SPI_EVENT_CRC_ERR                   5
#define SPI_EVENT_FRE_ERR                   6

/*********************************************************************/

/********************CR1 REGISTER BIT DEFINITIONS*********************/

#define CPHA                                0    
#define CPOL                                1
#define MSTR                                2
#define BR                                  3
#define SPE                                 6
#define LSBFIRST                            7
#define SSI                                 8
#define SSM                                 9
#define RXONLY                              10
#define CRCL                                11
#define CRCNEXT                             12
#define CRCEN                               13
#define BIDIOE                              14
#define BIDIMODE                            15

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define RXDMAEN                             0    
#define TXDMAEN                             1
#define SSOE                                2
#define NSSP                                3
#define FRF                                 4
#define ERRIE                               5
#define RXNEIE                              6
#define TXEIE                               7
#define DS                                  8
#define FRXTH                               12
#define LDMA_RX                             13
#define LDMA_TX                             14

/*********************************************************************/

/*********************SR REGISTER BIT DEFINITIONS*********************/

#define RXNE                                0    
#define TXE                                 1
#define CRCERR                              4
#define MODF                                5
#define OVR                                 6
#define BSY                                 7
#define FRE                                 8
#define FRLVL                               9
#define FTLVL                               11

/*********************************************************************/

/**********************STATUS FLAGS DEFINITIONS***********************/

#define SPI_FLAG_TXE                        ( 1 << TXE)   //status to check if TX buffer is empty or not
#define SPI_FLAG_RXNE                       ( 1 << RXNE)   //status to check if RX buffer is empty or not
#define SPI_FLAG_BUSY                       ( 1 << BSY)   //status to check if transmittion of data has ended or not

/*********************************************************************/

//Enable or Disable Peripheral
void SPIPeripheralControl(SPI_RegDef *pSPIx, uint8_t EnorDi);

//Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef *pSPIx, uint8_t EnorDi);

//Init and Deinit
void SPI_Init(SPI_Handle *pSPIHandle);
void SPI_DeInit(SPI_RegDef *pSPIx);

//SET or RESET ssi and ssoe bits
void SPI_SSIConfig(SPI_RegDef *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef *pSPIx, uint8_t EnorDi);

//Get status from the transmission
uint8_t SPI_GetFlagStatus(SPI_RegDef *pSPIx , uint32_t FlagName);

void SPI_CloseTransmission(SPI_Handle *pSPIHandle);
void SPI_CloseReception(SPI_Handle *pSPIHandle);

//Data Sent and Receive
void SPI_SendData(SPI_RegDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//Data Sent and Receive using INTERRUPTS
uint8_t SPI_SendDataIT(SPI_Handle *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Config and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle *pHandle);

//Application Callback
void SPIApplicationEventCallback(SPI_Handle *pSPIHandle, uint8_t AppEv);

#endif