#include "all_inits.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "stm32f030.h"
#include "rcc_driver.h"

I2C_Handle I2C1Handle;

void I2C1_GPIOInits(void)
{
  GPIO_Handle I2CPins;
  
  I2CPins.pGpiox = GPIOB;
  I2CPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  I2CPins.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
  I2CPins.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  I2CPins.Gpio_PinConfig.GPIO_PinAltFunMode = 1;
  I2CPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  //SCL
  I2CPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
  GPIO_Init(&I2CPins);

  //SDA
  I2CPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
  GPIO_Init(&I2CPins);
}

void I2C_Inits(void)
{
I2C1Handle.pI2Cx = I2C1;
I2C1Handle.I2C_Config.I2C_Freq = I2C_BUS_100KHZ;
I2C1Handle.I2C_Config.I2C_Mode = I2C_SCL_SPEED_SM;
I2C1Handle.I2C_Config.I2C_SlaveAddressMode = I2C_7BIT_ADDRESS_MODE;

I2C_Init(&I2C1Handle);
}

void SPI1_GPIOInits(void)
{
  GPIO_Handle SPIPins;

  SPIPins.pGpiox = GPIOB;
  SPIPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  SPIPins.Gpio_PinConfig.GPIO_PinAltFunMode = 0;
  SPIPins.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  SPIPins.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  SPIPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  //SCL
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  GPIO_Init(&SPIPins);

  //MOSI
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
  GPIO_Init(&SPIPins);

}

void SPI1_Inits(void)
{
  SPI_Handle SPI1handle;
  SPI1handle.pSPIx = SPI1;
  SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
  SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
  SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
  SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
  SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;  
  
  SPI_Init(&SPI1handle);
}

void RSTPIN_Init()
{
  GPIO_Handle GpioRst;

  GpioRst.pGpiox = GPIOA;
  GpioRst.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
  GpioRst.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioRst.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioRst.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioRst.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioRst);
}

void RSPIN_Init()
{
  GPIO_Handle GpioRs;

  GpioRs.pGpiox = GPIOA;
  GpioRs.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
  GpioRs.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioRs.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioRs.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioRs.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioRs);
}

void CSPIN_Init()
{
  GPIO_Handle GpioCS;

  GpioCS.pGpiox = GPIOB;
  GpioCS.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioCS.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioCS.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioCS.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioCS.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioCS);
}

void ButtonUp_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOA;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  button.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&button);
}

void ButtonEnter_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOA;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  button.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


  GPIO_Init(&button);
}

void ButtonDown_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOA;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  button.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


  GPIO_Init(&button);
}

void Redled_init(void)
{
  GPIO_Handle led;

  led.pGpiox = GPIOB;
  led.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
  led.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  led.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  led.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  led.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&led);
}

void Blueled_init(void)
{
  GPIO_Handle led;

  led.pGpiox = GPIOB;
  led.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
  led.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  led.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  led.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  led.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&led);
}

void buzzer_init(void)
{
  GPIO_Handle buzzer;

  buzzer.pGpiox = GPIOA;
  buzzer.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
  buzzer.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  buzzer.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  buzzer.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  buzzer.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&buzzer);
}

void baclight_init(void)
{
  GPIO_Handle baclight;

  baclight.pGpiox = GPIOB;
  baclight.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  baclight.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  baclight.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  baclight.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  baclight.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&baclight);
}

void heat_relay_init(void)
{
  GPIO_Handle heat_relay;

  heat_relay.pGpiox = GPIOA;
  heat_relay.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
  heat_relay.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  heat_relay.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  heat_relay.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  heat_relay.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&heat_relay);
}

void cold_relay_init(void)
{
  GPIO_Handle cold_relay;

  cold_relay.pGpiox = GPIOA;
  cold_relay.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  cold_relay.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  cold_relay.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  cold_relay.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  cold_relay.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&cold_relay);
}

void hardware_init()
{
  RSTPIN_Init();
  RSPIN_Init();
  CSPIN_Init();
  ButtonUp_init();
  ButtonEnter_init(); 
  ButtonDown_init(); 
  Redled_init();
  Blueled_init();
  buzzer_init();
  baclight_init();
  heat_relay_init();
  cold_relay_init();
  SPI1_GPIOInits();
  SPI1_Inits();
  I2C1_GPIOInits();
  I2C_Inits();
  TIM14_Init_30s();
  TIM3_Init_10s();
  SPI_SSIConfig(SPI1, ENABLE);
  SPIPeripheralControl(SPI1, ENABLE);
}