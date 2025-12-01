#ifndef MAIN_H
#define MAIN_H

void delay(void);

void I2C1_GPIOInits(void);

void I2C_Inits(void);

void irq_handlers_init();

extern volatile uint8_t new_data_ready;
extern volatile float latest_temp;


#endif