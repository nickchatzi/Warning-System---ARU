#include "spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle *pSPIHandle);
static void spi_modf_interrupt_handle(SPI_Handle *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle *pSPIHandle);
static void spi_crcerr_interrupt_handle(SPI_Handle *pSPIHandle);
static void spi_fre_interrupt_handle(SPI_Handle *pSPIHandle);


/************************** Peripheral Clock Setup *********************************
 
 * @fn          -SPI_PeripheralControl
 * 
 * @brief       -This function enables or disables peripheral clock for the given SPI port
 * 
 * @param[in]   -base address of the SPI peripheral
 * @param[in]   -ENABLE or DISABLE macros
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPI_PeriClockControl(SPI_RegDef *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }                         
    }
    else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DIS();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DIS();
        }        
    }
}

/*************************************************************************/

/******************************** Init **************************************
 
 * @fn          -SPI_Init
 * 
 * @brief       -This function initializes the SPI communication protocol
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPI_Init(SPI_Handle *pSPIHandle)
{
    uint32_t temp = 0;

     /*Enable the peripheral clock of used SPI*/
     SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    /*Configure the mode (master or slave)*/
    temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

    /*Configure the bus configuration:
    CLEAR CR1->BIDIMODE register will configure full-duplex mode*/

    if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {   
        temp &= ~(1 << BIDIMODE);
    }
    /*SET CR1->BIDIMODE register will configure half-duplex mode*/
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        temp |= (1 << BIDIMODE);
    }
    /*CLEAR CR1->BIDIMODE AND SET CR1->RXONLY register will configure simplex mode*/
    else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
    {
        temp &= ~(1 << BIDIMODE);
        temp |= (1 << RXONLY);
    }

    temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << BR;

    temp |= pSPIHandle->SPIConfig.SPI_DFF << CRCL;

    temp |= pSPIHandle->SPIConfig.SPI_CPOL << CPOL;

    temp |= pSPIHandle->SPIConfig.SPI_CPHA << CPHA;

    temp |= pSPIHandle->SPIConfig.SPI_SSM << SSM;

    pSPIHandle->pSPIx->CR1 = temp;

}

/*************************************************************************/

/******************************* Deinit **************************************
 
 * @fn          -SPI_DeInit
 * 
 * @brief       -This function de-initializes the SPI and resets all the registers of a peripheral.
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPI_DeInit(SPI_RegDef *pSPIx)
{
    if (pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
}
/*********************************************************************************/

/******************************* SPI ENABLE **************************************
 
 * @fn          -SPIPeripheralControl
 * 
 * @brief       -This function will enable the functionality of SPI.
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -Enable or Disable SPI
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPIPeripheralControl(SPI_RegDef *pSPIx, uint8_t EnorDi)
{
    
    if(EnorDi == ENABLE) 
    {
        pSPIx->CR1 |= (1 << SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPE);
    }
}
/*********************************************************************************/

/******************************* SSI ENABLE **************************************
 
 * @fn          -SPI_SSIConfig
 * 
 * @brief       -This function will SET or RESET SSI bit from CR1 register.
 *               Call this function when master's NSS is not connected to any pin in hardware.
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -Enable or Disable SPI
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -The function will not have an affect if SSM = 0.
 
*/

void SPI_SSIConfig(SPI_RegDef *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) 
    {
        pSPIx->CR1 |= (1 << SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SSI);
    }
}
/*********************************************************************************/

/******************************* SSOE ENABLE **************************************
 
 * @fn          -SSOE_SSIConfig
 * 
 * @brief       -This function will SET or RESET SSOE bit from CR2 register.
 *               Call this function when master's NSS is connected to slave's NSS.
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -Enable or Disable SPI
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPI_SSOEConfig(SPI_RegDef *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) 
    {
        pSPIx->CR2 |= (1 << SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1 << SSOE);
    }
}
/*********************************************************************************/

/******************************* Get Flag Status **************************************
 
 * @fn          -SPI_GetFlagStatus
 * 
 * @brief       -This function will return 1 if SPI is busy (on-going transmission) and 0 if not.
 * 
 * @param[in]   -Base address of the SPI port
 * @param[in]   -Choose a flag from the spi_driver.h (STATUS FLAGS DEFINITIONS section)
 * @param[in]   -
 * 
 * @return      -1 or 0
 * 
 * @note        -Call it inside a while loop before disabling SPI peripheral to make sure that transmission has succesfully ended.
 
*/

uint8_t SPI_GetFlagStatus(SPI_RegDef *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
    
	return FLAG_RESET;
}
/*********************************************************************************/

/********************************* Sent Data ****************************************
 
 * @fn          -SPI_SendData
 * 
 * @brief       -This function sents data to all configured devices.
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -void
 * 
 * @note        -Blocking Call
 
*/

void SPI_SendData(SPI_RegDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        //Wait until TXE is empty
        while (!(pSPIx->SR & (1 << TXE))); 

        //Then check the DFF (CRCL) BIT in CR1 register
        if (pSPIx->CR1 & (1 << CRCL)) 
        {
            pSPIx->DR = *((uint16_t*) pTxBuffer);   //Typecast so the pointer will become 16-bit pointer type
            Len--;
            Len--;
            (uint16_t*) pTxBuffer++;
        
        }
        else
        {
             *((volatile uint8_t*)&pSPIx->DR) = *pTxBuffer; 
            Len--;
            pTxBuffer++;
        }
    }
}
/*********************************************************************************/

/******************************** Receive Data **************************************
 
 * @fn          -SPI_ReceiveData
 * 
 * @brief       -This function receives data to all configured devices.
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -void
 * 
 * @note        -None
 
*/

void SPI_ReceiveData(SPI_RegDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        //Wait until RXE is empty
        while (!(pSPIx->SR & (1 << RXNE))); 
        /*FUTURE UPDATE ADD WATCHDOG SET UP TO PREVENT WHILE HANGING*/

        //Then check the DFF (CRCL) BIT in CR1 register
        if (pSPIx->CR1 & (1 << CRCL)) 
        {
            *((uint16_t*) pRxBuffer) = pSPIx->DR;   //load theb data from DR to RX buffer address
            Len--;
            Len--;
            (uint16_t*) pRxBuffer++;
        
        }  
        else
        {
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}
/*********************************************************************************/

/************************* Sent Data using Interrupt ********************************
 
 * @fn          -SPI_SendDataIT
 * 
 * @brief       -This function enables TXEIE control bit to get an interrupt whenever TXE flag is set in the SR register.
 *               When TXE flag is set, interrupt will be triggered and ISR will be executed.
 *               Data transmission will be handled by the ISR API.
 *               Basically, this API is not responsible for sending data. It just saves all info (pointer, length...) and enables TXEIE interrupt
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -State of TXE bit
 * 
 * @note        -Non-blocking Call
 
*/

uint8_t SPI_SendDataIT(SPI_Handle *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX)
    {
        /* Save the Tx buffer address and length info in some global variables */
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        /* Mark the SPI state as busy in transmission so no other executions will take over SPI peripheral until transission is over */
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        /* Enable TXEIE bit to get interrupt whenever TXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1<<TXEIE);
    }
    return state;
}
/*********************************************************************************/

/************************Receive Data using Interrupt******************************
 
 * @fn          -SPI_ReceiveDataIT
 * 
 * @brief       -This function enables RXEIE control bit to get an interrupt whenever RXE flag is set in the SR register.
 *               When RXE flag is set, interrupt will be triggered and ISR will be executed.
 *               Data reception will be handled by the ISR API.
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -State of RXE bit
 * 
 * @note        -None
 
*/

uint8_t SPI_ReceiveDataIT(SPI_Handle *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX)
    {
        /* Save the Rx buffer address and length info in some global variables */
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        /* Mark the SPI state as busy in transmission so no other executions will take over SPI peripheral until transission is over */
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        /* Enable RXEIE bit to get interrupt whenever RXE flag is set in SR */
        pSPIHandle->pSPIx->CR2 |= (1<<RXNEIE);
    }
    return state;
}
/*********************************************************************************/

/***************************IRQ Interrupt Config***********************************
 
 * @fn          -SPI_IRQInterruptConfig
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER |= ( 1 << IRQNumber );
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER |= ( 1 << IRQNumber );
        } 
    }     
}
/*********************************************************************************/

/***************************IRQ Priority Config***********************************
 
 * @fn          -SPI_IRQPriorityConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;  
    uint8_t iprx_section = IRQNumber % 4;  
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);  

    //*(NVIC_PR_BASE_ADDR + (iprx * 4)) &= ~(0xFF << shift_amount); //Clear previous priority
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount ); 
}
/*********************************************************************************/

/***************************IRQ Handling Config***********************************
 
 * @fn          -SPI_IRQHandling
 * 
 * @brief       -Configure the reason an interrupt occured and handle the appropriate interrupt.
 *               There are 6 possible reasons an interrupt occured:
 * 
 *                          INTERRUPT EVENT                  |   EVENT FLAG    |     ENABLE CONTROL BIT
 *               --------------------------------------------|-----------------|--------------------------
 *               1. Transmit Tx buffer ready to be loaded    |      TXE        |           TXEIE
 *               2. Data received in Rx buffer               |      RXNE       |           RXNEIE
 *               3. Master Mode fault event                  |      MODF       |           ERRIE
 *               4. Overrun Error                            |      OVR        |           ERRIE
 *               5. CRC Error                                |      CRCERR     |           ERRIE
 *               6. TI frame format error                    |      FRE        |           ERRIE
 * 
 * @param[in]   -Pin number of the selected SPI.
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void SPI_IRQHandling(SPI_Handle *pHandle)
{
    if (pHandle == NULL) return;

    uint8_t temp1, temp2;
    /*Check TXE flag*/
    temp1 = pHandle->pSPIx->SR & (1 << TXE);
    temp2 = pHandle->pSPIx->CR2 & (1 << TXEIE);

    if (temp1 && temp2)
    {
        /*Handle TXE*/
        spi_txe_interrupt_handle(pHandle);
    }
    /*Check RXNE flag*/
    temp1 = pHandle->pSPIx->SR & (1 << RXNE);
    temp2 = pHandle->pSPIx->CR2 & (1 << RXNEIE);

    if (temp1 && temp2)
    {
        /*Handle RXNE*/
        spi_rxne_interrupt_handle(pHandle);
    }
    /*Check MODF flag*/
    temp1 = pHandle->pSPIx->SR & (1 << MODF);
    temp2 = pHandle->pSPIx->CR2 & (1 << ERRIE);

    if (temp1 && temp2)
    {
        /*Handle MODF*/
        spi_modf_interrupt_handle(pHandle);
    }

        /*Check OVR flag*/
    temp1 = pHandle->pSPIx->SR & (1 << OVR);
    temp2 = pHandle->pSPIx->CR2 & (1 << ERRIE);

    if (temp1 && temp2)
    {
        /*Handle OVR*/
        spi_ovr_err_interrupt_handle(pHandle);
    }

    /*Check CRCERR flag*/
    temp1 = pHandle->pSPIx->SR & (1 << CRCERR);
    temp2 = pHandle->pSPIx->CR2 & (1 << ERRIE);

    if (temp1 && temp2)
    {
        /*Handle CRCERR*/
        spi_crcerr_interrupt_handle(pHandle);
    }
        /*Check FRE flag*/
    temp1 = pHandle->pSPIx->SR & (1 << FRE);
    temp2 = pHandle->pSPIx->CR2 & (1 << ERRIE);

    if (temp1 && temp2)
    {
        /*Handle FRE*/
        spi_fre_interrupt_handle(pHandle);
    }
}
/*********************************************************************************/

/*******PRIVATE FUNCTIONS FOR IRQHandling API (NOT TO BE USED BY THE USER)********/

static void spi_txe_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    //Check the DFF (CRCL) BIT in CR1 register
    if (pSPIHandle->pSPIx->CR1 & (1 << CRCL)) 
    {
        pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer); 
        pSPIHandle->TxLen--;
        pSPIHandle->TxLen--;
        (uint16_t*) pSPIHandle->pTxBuffer++;
    }
    else
    {
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer; 
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }
    if (!pSPIHandle->TxLen)
    {
        /* TXLen=0 so close the spi transmission and inform app that TX is over */
        SPI_CloseTransmission(pSPIHandle);

        SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}
static void spi_rxne_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

        //Check the DFF (CRCL) BIT in CR1 register
    if (pSPIHandle->pSPIx->CR1 & (1 << CRCL)) 
    {
        *((uint16_t*) pSPIHandle->pTxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR; 
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer--;
        pSPIHandle->pRxBuffer--;
    }
    else
    {
        *(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR; 
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer--;
    }
    if (!pSPIHandle->RxLen)
    {
        /* TXLen=0 so close the spi transmission and inform app that TX is over */
        SPI_CloseReception(pSPIHandle);

        SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}
static void spi_modf_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    volatile uint8_t temp;

    temp = pSPIHandle->pSPIx->SR;

    if (pSPIHandle->pSPIx->CR1 & (1 << SSM))
    {
        pSPIHandle->pSPIx->CR1 |= (1 << SSI);
    }

    pSPIHandle->pSPIx->CR1 |= (1 << SPE) | (1 << MSTR);

    SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_MODF_ERR);
}

static void spi_ovr_err_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    volatile uint8_t temp;
    /* Clear the OVR flag */
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void) temp;
    /* Inform the application */
    SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

static void spi_crcerr_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    pSPIHandle->pSPIx->SR = ~(1 << CRCERR);

    SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_CRC_ERR);
}

static void spi_fre_interrupt_handle(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    volatile uint8_t temp;

    temp = pSPIHandle->pSPIx->SR;

    SPIPeripheralControl(pSPIHandle->pSPIx, DISABLE);

    SPIApplicationEventCallback(pSPIHandle, SPI_EVENT_FRE_ERR);

    SPIPeripheralControl(pSPIHandle->pSPIx, ENABLE);
}

void SPI_CloseTransmission(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    /* Close the spi transmission */
    pSPIHandle->pSPIx->CR2 &= ~(1 << TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle *pSPIHandle)
{
    if (pSPIHandle == NULL) return;

    /* Close the spi transmission */
    pSPIHandle->pSPIx->CR2 &= ~(1 << RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;    
}

void SPIApplicationEventCallback(SPI_Handle *pSPIHandle, uint8_t AppEv)
{
}

/***************************************END******************************************/