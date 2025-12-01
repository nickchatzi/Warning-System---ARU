#ifndef LCD_H
#define LCD_H

#include "gpio_driver.h"
#include "spi_driver.h"
#include "stm32f030.h"

extern uint8_t current_page;    // Range: 0 to 7 
extern uint8_t current_column;  // Range: 0 to 127 (screen width)

void LCD_Select(void);
void LCD_Unselect(void);
void lcd_reset(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_set_page(uint8_t page);
void lcd_set_column(uint8_t column);
void lcd_write_char(char c);
void lcd_write_string(const char *str);
void lcd_set_black_background(void);
extern const uint8_t font5x8[][5];

void lcd_write_char_big(char c);
void lcd_write_string_big(const char *str);
void lcd_set_cursor(uint8_t page, uint8_t column);

void delay();

#endif