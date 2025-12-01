#include "i2c_driver.h"
#include "rcc_driver.h"

static uint32_t timingSettings(I2C_Handle *pI2CHandle);

/*Generate start or stop operation*/
static void I2C_GenerateStartCondition(I2C_RegDef *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef *pI2Cx);

/*Help functions for interrupt events*/
static void i2c_rxne_event_handle(I2C_Handle *pI2CHandle);
static void i2c_txis_event_handle(I2C_Handle *pI2CHandle);
static void i2c_stopf_event_handle(I2C_Handle *pI2CHandle);
static void i2c_tcr_event_handle(I2C_Handle *pI2CHandle);
static void i2c_tc_event_handle(I2C_Handle *pI2CHandle);
static void i2c_addr_event_handle(I2C_Handle *pI2CHandle);
static void i2c_nackf_event_handle(I2C_Handle *pI2CHandle);

/*Help functions for interrupt events*/
static void i2c_berr_event_handle(I2C_Handle *pI2CHandle);
static void i2c_arlo_event_handle(I2C_Handle *pI2CHandle);
static void i2c_ovr_event_handle(I2C_Handle *pI2CHandle);
static void i2c_pecerr_event_handle(I2C_Handle *pI2CHandle);
static void i2c_timeout_event_handle(I2C_Handle *pI2CHandle);
static void i2c_alert_event_handle(I2C_Handle *pI2CHandle);

/******************************* I2C ENABLE **************************************
 
 * @fn          -I2C_PeripheralControl
 * 
 * @brief       -This function will enable the functionality of I2C.
 * 
 * @param[in]   -Base address of the I2C port
 * @param[in]   -Enable or Disable I2C
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_PeripheralControl(I2C_RegDef *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) 
    {
        pI2Cx->I2C_CR1 |= (1 << I2C_PE);
    }
    else
    {
        pI2Cx->I2C_CR1 &= ~(1 << I2C_PE);
    }
}
/**********************************************************************************/

/************************** Peripheral Clock Setup *********************************
 
 * @fn          -I2C_PeripheralControl
 * 
 * @brief       -This function enables or disables peripheral clock for the given I2C port
 * 
 * @param[in]   -base address of the I2C peripheral
 * @param[in]   -ENABLE or DISABLE macros
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void I2C_PeriClockControl(I2C_RegDef *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
                        
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DIS();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DIS();
        }          
    }
}
/*************************************************************************/

/******************************** Init **************************************
 
 * @fn          -I2C_Init
 * 
 * @brief       -This function initializes the I2C communication protocol
 * 
 * @param[in]   -Base address of the TIM port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_Init(I2C_Handle *pI2CHandle)
{
    uint32_t tempreg = 0;

    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    pI2CHandle->pI2Cx->I2C_TIMINGR = 0x00000000;
    pI2CHandle->pI2Cx->I2C_TIMINGR = 0x2000090E;

    //Set the frequency of I2C communication
    //tempreg |= timingSettings(pI2CHandle);
    //pI2CHandle->pI2Cx->I2C_TIMINGR = tempreg;

    //No stretch configuration
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_NoStretch << I2C_NOSTRETCH;
    pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

    //Program the device own address
    tempreg = 0;

    if (pI2CHandle->I2C_Config.I2C_OwnAddressMode == I2C_7BIT_ADDRESS_MODE)
    {
        tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    }
    else
    {
        tempreg |= (1 << I2C_OA1MODE);
        tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OA1;
    }

    tempreg |= (1 << I2C_OA1EN);
    
    pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;
}
/*************************************************************************/

/******************************* Deinit **************************************
 
 * @fn          -I2C_DeInit
 * 
 * @brief       -This function de-initializes the I2C and resets all the registers of a peripheral.
 * 
 * @param[in]   -Base address of the I2C port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_DeInit(I2C_RegDef *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
}
/*****************************************************************************************/

/********************************* Master Sent Data ****************************************
 
 * @fn          -I2C_MasterSendData
 * 
 * @brief       -This function sents data to all configured devices.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * @param[in]   -The address of the slave
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
#if 0
void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr)
{
    /* Wait until bus is free */
    while (pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_BUSY))
    {
        if (pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_NACKF))
        {
            pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_NACKCF);
            return;
        }
    }

    /* Clear CR2 configuration */
    pI2CHandle->pI2Cx->I2C_CR2 &= ~((0x3FF << 0) | (0xFF << I2C_NBYTES) | (1 << I2C_RD_WRN) | (1 << I2C_AUTOEND));

    /* Set slave address (7-bit) */
    pI2CHandle->pI2Cx->I2C_CR2 |= ((slaveAddr & 0x7F) << 1);

    /* Set number of bytes */
    pI2CHandle->pI2Cx->I2C_CR2 |= (length << I2C_NBYTES);

    /* Set write mode */
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_RD_WRN);

    /* Manual STOP mode */
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_AUTOEND);

    /* Generate START */
    pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_START);

    /* Transmit bytes */
    while (length > 0)
    {
        /* Wait for TXIS */
        while (!(pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TXIS)))
        {
            if (pI2CHandle->pI2Cx->I2C_ISR & ((1 << I2C_NACKF) | (1 << I2C_BERR) | (1 << I2C_ARLO)))
            {
                pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_NACKCF) | (1 << I2C_BERRCF) | (1 << I2C_ARLOCF);
                return;
            }
        }

        /* Send byte */
        pI2CHandle->pI2Cx->I2C_TXDR = *pTxBuffer++;
        length--;
    }

    /* If no repeated start is needed, send STOP */
    if (Sr == I2C_DISABLE_SR)
    {
        pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_STOP);
    }

    /* Wait for STOPF */
    while (!(pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_STOPF)));

    /* Clear STOPF */
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_STOPCF);
}



#else
void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr)
{
    // Wait until BUSY flag is cleared (bus free)
    while (pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_BUSY));

    // Clear old configuration in CR2
    pI2CHandle->pI2Cx->I2C_CR2 &= ~((0x3FF << 0) | (0xFF << I2C_NBYTES) | (1 << I2C_RD_WRN));  // Clear SADD, NBYTES, RD_WRN

    // Configure CR2
    if (pI2CHandle->I2C_Config.I2C_SlaveAddressMode == I2C_10BIT_ADDRESS_MODE)
    {
        pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_ADD10);
        pI2CHandle->pI2Cx->I2C_CR2 |= (slaveAddr & 0x3FF);     // 10-bit address 
    }
    else
    {
        pI2CHandle->pI2Cx->I2C_CR2 |= (slaveAddr << 1);     // 7-bit address
    }

    pI2CHandle->pI2Cx->I2C_CR2 |= (length << I2C_NBYTES);       // Set NBYTES
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_RD_WRN);           // Write mode

    // Enable TX interrupt (optional if polling TXIS)
    pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_TXIE);

    // Generate START condition
    pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_START);

    // Transmit all bytes
    while (length > 0)
    {
        // Wait until TXIS is set
        while (!(pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TXIS)));

        // Send next byte
        pI2CHandle->pI2Cx->I2C_TXDR = *pTxBuffer++;
        length--;
    }

    // Wait until Transfer Complete (TC) flag is set
    while (!(pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TC)));

    // If no repeated start is requested, send STOP
    if (Sr == I2C_DISABLE_SR)
    {
        pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_STOP);
    }
}
#endif

/********************************************************************************************/

/********************************* Master Receive Data ****************************************
 
 * @fn          -I2C_MasterReceiveData
 * 
 * @brief       -This function receives data from slave to master.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * @param[in]   -The address of the slave
 * 
 * @return      -void
 * 
 * @note        -none
 */

void I2C_MasterReceiveData(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr)
{
    // Clear previous slave address and set new one in CR2
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(0x3FF);          // Clear previous address
    pI2CHandle->pI2Cx->I2C_CR2 |= (slaveAddr << 1);  // Set new 7-bit slave address (shifted)

    // Set number of bytes to receive
    pI2CHandle->pI2Cx->I2C_CR2 |= (length << I2C_NBYTES); 

    // Set Read mode (bit 10 in CR2)
    pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_RD_WRN);   // RD_WRN = 1 (Read mode)

    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);   

    // Wait until RXNE (Receive Buffer Not Empty) flag is set
    for (uint32_t i = 0; i < length; i++)
    {
        while (!(pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_RXNE))); // Wait until RXNE is set

        // Read received data from RXDR
        pRxBuffer[i] = pI2CHandle->pI2Cx->I2C_RXDR;
    }

    if( Sr == I2C_DISABLE_SR){

        I2C_GenerateStopCondition(pI2CHandle->pI2Cx); 
    }

    // Wait until STOP flag is set
    while (pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_STOPF));

    // Clear STOP flag
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_STOPCF);
}
/************************************************************************************/

/************************* Sent Data using Interrupt ********************************
 
 * @fn          -I2C_MasterSendDataIT
 * 
 * @brief       -
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * @param[in]   -The address of the slave
 * 
 * @return      -State of TXE bit
 * 
 * @note        -Non-blocking Call
 
*/

uint8_t I2C_MasterSendDataIT(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = slaveAddr;
        pI2CHandle->Sr = Sr;

        // Generate START condition and send slave address
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Enable interrupts for TX, error, and stop events
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_TXIE);  // Enable TX interrupt
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_ERRIE); // Enable error interrupts
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_STOPIE); // Enable STOP interrupt
    }

    return busystate;
}

/************************************************************************************/

/************************ Receive Data using Interrupt ******************************
 
 * @fn          -I2C_MasterReceiveDataIT
 * 
 * @brief       -
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * @param[in]   -The address of the slave
 * 
 * @return      -State of RXE bit
 * 
 * @note        -None
 
*/

uint8_t I2C_MasterReceiveDataIT(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slaveAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pRxBuffer;
        pI2CHandle->TxLen = length;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->DevAddr = slaveAddr;
        pI2CHandle->Sr = Sr;

        // Generate START condition and send slave address
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Enable interrupts for TX, error, and stop events
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_TXIE);  // Enable TX interrupt
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_ERRIE); // Enable error interrupts
        pI2CHandle->pI2Cx->I2C_CR1 |= (I2C_STOPIE); // Enable STOP interrupt
    }

    return busystate;
}

/***************************************************************************************/

/********************************* Slave Sent Data ****************************************
 
 * @fn          -I2C_SlaveSendData
 * 
 * @brief       -This function sents data to the master.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void I2C_SlaveSendData(I2C_RegDef *pI2Cx , uint8_t data)
{
    // Wait until TXIS (Transmit Interrupt Status) flag is set
    while (!(pI2Cx->I2C_ISR & (1 << I2C_TXIS)));

    //Then send the data
    pI2Cx->I2C_TXDR = data;
}
/******************************************************************************************/

/********************************* Slave Receive Data ****************************************
 
 * @fn          -I2C_SlaveReceiveData
 * 
 * @brief       -This function receives data from master.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

uint8_t I2C_SlaveReceiveData(I2C_RegDef *pI2Cx)
{
    // Wait until RXNE (Receive Buffer Not Empty) flag is set
    while (!(pI2Cx->I2C_ISR & (1 << I2C_RXNE)));

    //Then receive and return the data
    return (uint8_t) pI2Cx->I2C_RXDR;
}
/*******************************************************************************************/

/************************************ Get Flag Status *****************************************
 
 * @fn          -I2C_GetFlagStatus
 * 
 * @brief       -This function will return 1 if I2C is busy (on-going transmission) and 0 if not.
 * 
 * @param[in]   -Base address of the I2C port
 * @param[in]   -Choose a flag from the i2c_driver.h (STATUS FLAGS DEFINITIONS section)
 * @param[in]   -
 * 
 * @return      -1 or 0
 * 
 * @note        -None.
 
*/

uint8_t I2C_GetFlagStatus(I2C_RegDef *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->I2C_ISR & FlagName)
	{
		return FLAG_SET;
	}
    
	return FLAG_RESET;
}
/*************************************************************************/

/*************************** IRQ Interrupt Config ***********************************
 
 * @fn          -I2C_IRQInterruptConfig
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

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        *NVIC_ISER |= (1 << IRQNumber);
    }
    else
    {
        *NVIC_ICER |= (1 << IRQNumber);
    }
}

/*************************************************************************/

/*************************** IRQ Priority Config ***********************************
 
 * @fn          -I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;  
    uint8_t iprx_section = IRQNumber % 4;  
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);  

    //*(NVIC_PR_BASE_ADDR + (iprx * 4)) &= ~(0xFF << shift_amount); //Clear previous priority
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount ); 
}
/*************************************************************************/

/********************* IRQ Event Interrupt Config ***************************
 
 * @fn          -I2C_EV_IRQHandling
 * 
 * @brief       -Interrupt handling for interrupts generated by I2C events.
 * 
 * @param[in]   -Base address of the I2C peripheral.
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_EV_IRQHandling(I2C_Handle *pI2CHandle)
{
    if (pI2CHandle == NULL) return;

    uint8_t temp1, temp2;

    /*Interrupt event: Receive buffer not empty*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_RXNE);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_RXIE);

    if (temp1 && temp2)
    {
        i2c_rxne_event_handle(pI2CHandle);
    }

    /*Interrupt event: Trandmit buffer interrupt status*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TXIS);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_TXIE);    

    if (temp1 && temp2)
    {
        i2c_txis_event_handle(pI2CHandle);
    }

    /*Interrupt event: STOP detection interrupt flag*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_STOPF);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_STOPIE);    

    if (temp1 && temp2)
    {
        i2c_stopf_event_handle(pI2CHandle);
    }

    /*Interrupt event: Tranfer complete reload**/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TCR);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_TCIE);    

    if (temp1 && temp2)
    {
        i2c_tcr_event_handle(pI2CHandle);
    }

    /*Interrupt event: Tranfer complete */
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TC);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_TCIE);    

    if (temp1 && temp2)
    {
        i2c_tc_event_handle(pI2CHandle);
    }

    /*Interrupt event: Address matched */
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_ADDR);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ADDRIE);    

    if (temp1 && temp2)
    {
        i2c_addr_event_handle(pI2CHandle);
    }

    /*Interrupt event: NACK reception */
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_NACKF);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_NACKIE);    

    if (temp1 && temp2)
    {
        i2c_nackf_event_handle(pI2CHandle);
    }

}
/*************************************************************************/

/********************* IRQ Error Interrupt Config ****************************
 
 * @fn          -I2C_ER_IRQHandling
 * 
 * @brief       -Interrupt handling for interrupts generated by I2C errors.
 * 
 * @param[in]   -Base address of the I2C peripheral.
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_ER_IRQHandling(I2C_Handle *pI2CHandle)
{
    if (pI2CHandle == NULL) return;

    uint8_t temp1, temp2;

    /*Interrupt event: Receive buffer not empty*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_BERR);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);

    if (temp1 && temp2)
    {
        i2c_berr_event_handle(pI2CHandle);
    }

    /*Interrupt event: Trandmit buffer interrupt status*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_ARLO);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);    

    if (temp1 && temp2)
    {
        i2c_arlo_event_handle(pI2CHandle);
    }

    /*Interrupt event: STOP detection interrupt flag*/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_OVR);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);    

    if (temp1 && temp2)
    {
        i2c_ovr_event_handle(pI2CHandle);
    }

    /*Interrupt event: Tranfer complete reload**/
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_PECERR);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);    

    if (temp1 && temp2)
    {
        i2c_pecerr_event_handle(pI2CHandle);
    }

    /*Interrupt event: Tranfer complete */
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_TIMEOUT);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);    

    if (temp1 && temp2)
    {
        i2c_timeout_event_handle(pI2CHandle);
    }

    /*Interrupt event: Address matched */
    temp1 = pI2CHandle->pI2Cx->I2C_ISR & (1 << I2C_ALERT);
    temp2 = pI2CHandle->pI2Cx->I2C_CR1 & (1 << I2C_ERRIE);    

    if (temp1 && temp2)
    {
        i2c_alert_event_handle(pI2CHandle);
    }
}
/*************************************************************************/

static void I2C_GenerateStartCondition(I2C_RegDef *pI2Cx)
{
	//  Generate the START condition
    pI2Cx->I2C_CR2 |= ( 1 << I2C_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef *pI2Cx)
{
    // Generate STOP condition after receiving last byte
    pI2Cx->I2C_CR2 |= (1 << I2C_STOP); 
}

static uint32_t timingSettings(I2C_Handle *pI2CHandle)
{
    uint32_t temp = 0;

    if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_SM)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_10KHZ)
        {
            temp |= ( 0xC7 << I2C_SCLL );
            temp |= ( 0xC3 << I2C_SCLH );
            temp |= ( 0x02 << I2C_SDADEL );
            temp |= ( 0x04 << I2C_SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x03 << I2C_PRESC );

            //pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else if(pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_100KHZ)
        {
            temp |= ( 0x14 << I2C_SCLL );
            temp |= ( 0x10 << I2C_SCLH );
            temp |= ( 0x02 << I2C_SDADEL );
            temp |= ( 0x04 << I2C_SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x03 << I2C_PRESC );
        }
        else
        {
            printf("Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }
    else if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_FM)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_200KHZ)
        {
            temp |= ( 0x3D << I2C_SCLL );
            temp |= ( 0x0B << I2C_SCLH );
            temp |= ( 0x00 << I2C_SDADEL );
            temp |= ( 0x01 << I2C_SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x00 << I2C_PRESC );

            //pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else if(pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_400KHZ)
        {
            temp |= ( 0x09 << I2C_SCLL );
            temp |= ( 0x03 << I2C_SCLH );
            temp |= ( 0x02 << I2C_SDADEL );
            temp |= ( 0x03 << I2C_SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x01 << I2C_PRESC );
        }
        else
        {
            printf("Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }
    else if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_FMP)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_1000KHZ)
        {
            temp |= ( 0x04 << I2C_SCLL );
            temp |= ( 0x02 << I2C_SCLH );
            temp |= ( 0x00 << I2C_SDADEL );
            temp |= ( 0x00 << I2C_SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x00 << I2C_PRESC );

            //pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else
        {
            printf("Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }

    return temp;
}

/*******PRIVATE FUNCTIONS FOR EVENT IRQHandling API (NOT TO BE USED BY THE USER)********/

static void i2c_rxne_event_handle(I2C_Handle *pI2CHandle)
{
    // Read data from RXDR (clears RXNE flag automatically)
    *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->I2C_RXDR;
    
    // Move buffer pointer to next location
    pI2CHandle->pRxBuffer++;

    // Decrease remaining length
    pI2CHandle->RxLen--;

    // If last byte received
    if (pI2CHandle->RxLen == 0)
    {
        // Disable RXNE interrupt (RXIE)
        pI2CHandle->pI2Cx->I2C_CR1 &= ~I2C_RXIE;

        // If automatic STOP condition is needed, generate STOP
        if (pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx); 
        }

        // Reset the I2C state
        pI2CHandle->TxRxState = I2C_READY;

        I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_CMPLT);
    }
}

static void i2c_txis_event_handle(I2C_Handle *pI2CHandle)
{
    pI2CHandle->pI2Cx->I2C_TXDR = *(pI2CHandle->pTxBuffer);
    
    // Move buffer pointer to next location
    pI2CHandle->pTxBuffer++;

    // Decrease remaining length
    pI2CHandle->TxLen--;

    // If last byte received
    if (pI2CHandle->TxLen == 0)
    {
        // Disable RXNE interrupt (RXIE)
        pI2CHandle->pI2Cx->I2C_CR1 &= ~I2C_TXIE;

        // If automatic STOP condition is needed, generate STOP
        if (pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx); 
        }

        // Reset the I2C state
        pI2CHandle->TxRxState = I2C_READY;

        I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_CMPLT);
    }
}


static void i2c_stopf_event_handle(I2C_Handle *pI2CHandle)
{
    //Set STOPCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_STOPCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);
}

static void i2c_tcr_event_handle(I2C_Handle *pI2CHandle)
{
    // Determine the remaining bytes to transfer
    uint8_t new_nbytes;

    if (pI2CHandle->TxLen > 255)
    {
        new_nbytes = 255;   // Transfer 255 bytes first
    }
    else
    {
        new_nbytes = pI2CHandle->TxLen;  // Transfer the remaining bytes
    }

    // Clear previous NBYTES value and update with new size
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(0xFF << I2C_NBYTES);
    pI2CHandle->pI2Cx->I2C_CR2 |= ((new_nbytes & 0xFF) << I2C_NBYTES);

    // Decrease the remaining transfer length
    pI2CHandle->TxLen -= new_nbytes;

    // If this is the last transfer, enable AUTOEND to generate STOP condition
    if (pI2CHandle->TxLen == 0)
    {
        pI2CHandle->pI2Cx->I2C_CR2 |= I2C_AUTOEND;
    }

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_TCR);
}

static void i2c_tc_event_handle(I2C_Handle *pI2CHandle)
{
    // If Repeated Start is disabled, generate STOP condition
    if (pI2CHandle->Sr == I2C_DISABLE_SR)
    {
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx); 
    }

    // Reset I2C state to READY
    pI2CHandle->TxRxState = I2C_READY;

    // Notify the application about the Transfer Complete event
    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_TC);
}

static void i2c_addr_event_handle(I2C_Handle *pI2CHandle)
{
    //Set ADDRCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_ADDRCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_ADDR);
}

static void i2c_nackf_event_handle(I2C_Handle *pI2CHandle)
{
    //Set NACKCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_NACKCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_NACKF);
}
/***********************************************************************************/

/*****PRIVATE FUNCTIONS FOR ERROR IRQHandling API (NOT TO BE USED BY THE USER)******/

static void i2c_berr_event_handle(I2C_Handle *pI2CHandle)
{
    //Set BERRCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_BERRCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_BERR);
}

static void i2c_arlo_event_handle(I2C_Handle *pI2CHandle)
{
    //Set ARLOCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_ARLOCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_ARLO);
}

static void i2c_ovr_event_handle(I2C_Handle *pI2CHandle)
{
    //Set OVRCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_OVRCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_OVR);
}

static void i2c_pecerr_event_handle(I2C_Handle *pI2CHandle)
{
    //Set PECCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_PECCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_PECERR);
}

static void i2c_timeout_event_handle(I2C_Handle *pI2CHandle)
{
    //Set TIMOUTCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_TIMOUTCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_TIMEOUT);
}

static void i2c_alert_event_handle(I2C_Handle *pI2CHandle)
{
    //Set ALERTCF bit from the ICR register
    pI2CHandle->pI2Cx->I2C_ICR |= (1 << I2C_ALERTCF);

    // Reset the I2C state
    pI2CHandle->TxRxState = I2C_READY;

    I2CApplicationEventCallback(pI2CHandle, I2C_EVENT_ALERT);
}
/***********************************************************************************/

void I2CApplicationEventCallback(I2C_Handle *pI2CHandle, uint8_t AppEv)
{
}

/************************************END*********************************************/
