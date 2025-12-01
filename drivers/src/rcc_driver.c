#include "rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};
uint16_t MSI_Range[12] = {100,200,400,800,1000,2000,4000,8000,16000,24000,32000,48000};

typedef struct
{
	uint8_t pllp;
	uint8_t pllq;
	uint8_t pllr;
}PLLDividers;

static void assert_failed(uint8_t *file, uint32_t line);

static uint8_t RCC_GetPLLOutputClock()
{
	return 0;
}

static uint16_t RCC_GetMSIOutputClock()
{
	uint8_t temp;
	uint16_t msiclk;

	temp = (RCC->CR >> MSIRANGE) & 0xF;

	msiclk = MSI_Range[temp * 1000];

	return msiclk;
}

/*CALCULATE THE PCLK2*/
uint32_t RCC_GetCLKValue(void)
{
	uint32_t clksrc;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		return RCC_GetMSIOutputClock();
	}
	else if (clksrc == 1)
	{
		return HSI;    
	}
	else if (clksrc == 2)
	{
		return HSE;   
	}
	else
    {
        return 0;
    }
}

/*CALCULATE THE PCLK1*/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = HSI;    
	}
	else if (clksrc == 2)
	{
		SystemClk = HSE;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> HPRE) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> PPRE1) & 0x7;

	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}
	pclk1 = (SystemClk/ahbp) / apb1p;

	return pclk1;
}

/*CALCULATE THE PCLK2*/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = HSI;    
	}
	else if (clksrc == 2)
	{
		SystemClk = HSE;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> HPRE) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> PPRE2) & 0x7;

	if (temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB2_PreScaler[temp-4];
	}
	pclk2 = (SystemClk/ahbp) / apb2p;

	return pclk2;
}

/******************************* SET HSI CLOCK **************************************
 
 * @fn          -setHSIclock
 * 
 * @brief       -This function will set HSI as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setHSIclock(RCC_Handle *pRCCHandle)
{
	uint32_t temp = 0;

	// Clear MSI bit
	pRCCHandle->pRCC->CR &= ~(1 << MSION);

	// Enable HSI clock
	pRCCHandle->pRCC->CR |= (1 << HSION);

	// Wait for HSI to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << HSIRDY))); 

	// Select HSI as system clock 
	pRCCHandle->pRCC->CFGR  |= (1 << SW);  

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (1 << SWS));  

	temp = pRCCHandle->pRCC->CFGR;

	//Configure the AHB prescaler
	temp &= ~(0xF << HPRE);
	temp |= (pRCCHandle->RCC_Config.RCC_AHB_Presc << HPRE);

	//Configure the APB1(Low Speed) prescaler
	temp &= ~(0x7 << PPRE1);
	temp |= (pRCCHandle->RCC_Config.RCC_APB1 << PPRE1);

	//Configure the APB2(High Speed) prescaler
	temp &= ~(0x7 << PPRE2);
	temp |= (pRCCHandle->RCC_Config.RCC_APB2 << PPRE2);
	
	pRCCHandle->pRCC->CFGR = temp; 
	
}

/******************************* SET HSE CLOCK **************************************
 
 * @fn          -setHSEclock
 * 
 * @brief       -This function will set HSE as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setHSEclock(RCC_Handle *pRCCHandle)
{
	uint32_t temp = 0;

	// Enable HSE clock
	pRCCHandle->pRCC->CR |= (1 << HSEON);  

	// Wait for HSE to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << HSERDY))); 

	temp = pRCCHandle->pRCC->CFGR;
	// Select HSE as system clock
	temp |= (0x2 << SW);  

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (0x2 << SWS));  

	//Configure the AHB prescaler
    temp &= ~(0xF << HPRE);
    temp |= (pRCCHandle->RCC_Config.RCC_AHB_Presc << HPRE);

	//Configure the APB1(Low Speed) prescaler
    temp &= ~(0x7 << PPRE1);
    temp |= (pRCCHandle->RCC_Config.RCC_APB1 << PPRE1);

	//Configure the APB2(High Speed) prescaler
    temp &= ~(0x7 << PPRE2);
    temp |= (pRCCHandle->RCC_Config.RCC_APB2 << PPRE2);

	pRCCHandle->pRCC->CFGR = temp;

}


static void assert_failed(uint8_t *file, uint32_t line)
{
    while(1);
}