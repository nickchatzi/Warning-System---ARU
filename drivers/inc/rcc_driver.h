#ifndef RCC_CONFIG_H
#define RCC_CONFIG_H

#define USE_FULL_ASSERT

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0)
#endif

#include "stm32f030.h"


/****************STRUCTURE FOR RCC CONFIGURATION****************/

typedef struct
{
    uint8_t RCC_AHB_Presc;
    uint8_t RCC_APB1;
    uint8_t RCC_APB2;
    uint8_t RCC_MSIRange;
    uint8_t RCC_MSIRangeStandby;
    uint8_t RCC_PLLM;    
    uint8_t RCC_PLLN;
    uint8_t RCC_PLLP;
    uint8_t RCC_PLLQ;
    uint8_t RCC_PLLR;

}RCC_Config;

/*********************************************************************/

/***********************STRUCTURE FOR RCC HANDLE**********************/

typedef struct
{
    RCC_RegDef  *pRCC; 
    RCC_Config RCC_Config;

}RCC_Handle; 

/*********************************************************************/

/********************PRESCALER ARRAY DEFINITIONS**********************/

extern uint16_t AHB_PreScaler[8];
extern uint8_t APB1_PreScaler[4]; 
extern uint8_t APB2_PreScaler[4]; 
extern uint16_t MSI_Range[12];

/*********************************************************************/

/********************CLOCK CONFIGURATION FREQUENCY*********************/

#define HSI                 16000000ULL
#define HSE                 8000000ULL

/*********************************************************************/

/***********************RCC AHB PRESCALER MACROS**********************/

#define AHB_NO_DIV          0
#define AHB_DIV_2           8
#define AHB_DIV_4           9
#define AHB_DIV_8           10
#define AHB_DIV_16          11
#define AHB_DIV_64          12
#define AHB_DIV_128         13
#define AHB_DIV_256         14
#define AHB_DIV_512         15

/*********************************************************************/

/***********************RCC APB1 PRESCALER MACROS*********************/

#define APB1_NO_DIV          0
#define APB1_DIV_2           4
#define APB1_DIV_4           5
#define APB1_DIV_8           6
#define APB1_DIV_16          7

/*********************************************************************/

/***********************RCC APB2 PRESCALER MACROS*********************/

#define APB2_NO_DIV          0
#define APB2_DIV_2           4
#define APB2_DIV_4           5
#define APB2_DIV_8           6
#define APB2_DIV_16          7

/*********************************************************************/

/***********************RCC MSI PRESCALER MACROS**********************/

#define MSI_CLOCK_100KHZ    0
#define MSI_CLOCK_200KHZ    1
#define MSI_CLOCK_400KHZ    2
#define MSI_CLOCK_800KHZ    3
#define MSI_CLOCK_1MHZ      4
#define MSI_CLOCK_2MHZ      5
#define MSI_CLOCK_4MHZ      6
#define MSI_CLOCK_8MHZ      7
#define MSI_CLOCK_16MHZ     8
#define MSI_CLOCK_24MHZ     9
#define MSI_CLOCK_32MHZ     10
#define MSI_CLOCK_48MHZ     11

/*********************************************************************/

/***********************RCC PLLM MACROS*******************************/

#define PLLM_DIV_1          0
#define PLLM_DIV_2          1
#define PLLM_DIV_3          2
#define PLLM_DIV_4          3
#define PLLM_DIV_5          4
#define PLLM_DIV_6          5
#define PLLM_DIV_7          6
#define PLLM_DIV_8          7

/*********************************************************************/

/***********************RCC PLLN MACROS*******************************/

#define PLLN_MULT_8         8
#define PLLN_MULT_9         9
#define PLLN_MULT_10        10
#define PLLN_MULT_11        11
#define PLLN_MULT_12        12
#define PLLN_MULT_13        13
#define PLLN_MULT_14        14
#define PLLN_MULT_15        15
#define PLLN_MULT_16        16
#define PLLN_MULT_17        17
#define PLLN_MULT_18        18
#define PLLN_MULT_19        19
#define PLLN_MULT_20        20
#define PLLN_MULT_21        21
#define PLLN_MULT_22        22
#define PLLN_MULT_23        23
#define PLLN_MULT_24        24
#define PLLN_MULT_25        25
#define PLLN_MULT_26        26
#define PLLN_MULT_27        27
#define PLLN_MULT_28        28
#define PLLN_MULT_29        29
#define PLLN_MULT_30        30
#define PLLN_MULT_31        31
#define PLLN_MULT_32        32
#define PLLN_MULT_33        33
#define PLLN_MULT_34        34
#define PLLN_MULT_35        35
#define PLLN_MULT_36        36
#define PLLN_MULT_37        37
#define PLLN_MULT_38        38
#define PLLN_MULT_39        39
#define PLLN_MULT_40        40
#define PLLN_MULT_41        41
#define PLLN_MULT_42        42
#define PLLN_MULT_43        43
#define PLLN_MULT_44        44
#define PLLN_MULT_45        45
#define PLLN_MULT_46        46
#define PLLN_MULT_47        47
#define PLLN_MULT_48        48
#define PLLN_MULT_49        49
#define PLLN_MULT_50        50
#define PLLN_MULT_51        51
#define PLLN_MULT_52        52
#define PLLN_MULT_53        53
#define PLLN_MULT_54        54
#define PLLN_MULT_55        55
#define PLLN_MULT_56        56
#define PLLN_MULT_57        57
#define PLLN_MULT_58        58
#define PLLN_MULT_59        59
#define PLLN_MULT_60        60
#define PLLN_MULT_61        61
#define PLLN_MULT_62        62
#define PLLN_MULT_63        63
#define PLLN_MULT_64        64
#define PLLN_MULT_65        65
#define PLLN_MULT_66        66
#define PLLN_MULT_67        67
#define PLLN_MULT_68        68
#define PLLN_MULT_69        69
#define PLLN_MULT_70        70
#define PLLN_MULT_71        71
#define PLLN_MULT_72        72
#define PLLN_MULT_73        73
#define PLLN_MULT_74        74
#define PLLN_MULT_75        75
#define PLLN_MULT_76        76
#define PLLN_MULT_77        77
#define PLLN_MULT_78        78
#define PLLN_MULT_79        79
#define PLLN_MULT_80        80
#define PLLN_MULT_81        81
#define PLLN_MULT_82        82
#define PLLN_MULT_83        83
#define PLLN_MULT_84        84
#define PLLN_MULT_85        85
#define PLLN_MULT_86        86

/*********************************************************************/

/***********************RCC PLLP MACROS*******************************/

#define PLLP_DIV_7          0
#define PLLP_DIV_17         1
#define PLLP_NOT USED       2

/*********************************************************************/


/***********************RCC PLLQ MACROS*******************************/

#define PLLQ_DIV_2          0
#define PLLQ_DIV_4          1
#define PLLQ_DIV_6          2
#define PLLQ_DIV_8          3
#define PLLQ_NOT USED       4

/*********************************************************************/

/***********************RCC PLLR MACROS*******************************/

#define PLLR_DIV_2          0
#define PLLR_DIV_4          1
#define PLLR_DIV_6          2
#define PLLR_DIV_8          3
#define PLLR_NOT USED       4

/*********************************************************************/

/********************CR REGISTER BIT DEFINITIONS**********************/

#define MSION               0
#define MSIRDY              1
#define MSIPLLEN            2
#define MSIRGSEL            3
#define MSIRANGE            4
#define HSION               8
#define HSIKERON            9
#define HSIRDY              10
#define HSIASFS             11
#define HSEON               16
#define HSERDY              17
#define HSEBYP              18
#define CSSON               19
#define PLLON               24
#define PLLRDY              25
#define PLLSAI1ON           26
#define PLLSAI1RDY          27
#define PLLSAI2ON           28
#define PLLSAI2RDY          29

/**********************************************************************/

/********************CRGR REGISTER BIT DEFINITIONS*********************/

#define SW                  0
#define SWS                 2
#define HPRE                4
#define PPRE1               8
#define PPRE2               11
#define STOPWUCK            15
 
/**********************************************************************/

/********************PLLCFGR REGISTER BIT DEFINITIONS**********************/

#define PLLSRC               0
#define PLLM                 4
#define PLLN                 8
#define PLLPEN               16
#define PLLP                 17
#define PLLQEN               20
#define PLLQ                 21
#define PLLREN               24
#define PLLR                 25
#define PLLPDIV              27

/**********************************************************************/

/********************CRS REGISTER BIT DEFINITIONS*********************/

#define LSION               0
#define LSIRDY              1
#define MSISRANGE           8
 
/**********************************************************************/

uint32_t RCC_GetCLKValue(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
void setHSIclock(RCC_Handle *pRCCHandle);
void setHSEclock(RCC_Handle *pRCCHandle);
void setMSIclock(RCC_Handle *pRCCHandle);
void setPLLclock(RCC_Handle *pRCCHandle);

#endif