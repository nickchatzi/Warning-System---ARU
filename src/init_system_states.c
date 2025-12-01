#include "init_system_states.h"
#include "display.h"
#include "tmp75.h"
#include "startup.h"
#include "lcd.h"
#include "buzzer.h"

/******************************* PRIVATE DECLARATIONS ************************************/

SystemState_t systemState;
static void redLed_on(void);
static void redLed_off(void);
static void blueLed_on(void);
static void blueLed_off(void);
static void activate_backlight(void);
static void display_state_normal(void);
static void display_state_high_temp(void);
static void display_state_low_temp(void);
static void activate_heat_relay(void);
static void disable_heat_relay(void);
static void activate_cold_relay(void);
static void disable_cold_relay(void);

/******************************* SET STATE **************************************
 
 * @fn          -set_state
 * 
 * @brief       -Set the state of the system depending the ambient temperature.
 * 
 * @param[in]   -none
 *
 * @return      -state.
 * 
 * @note        -none
 
*/

SystemState_t set_state(void)
{
    float temp_float = convertTemp(); 
    int16_t temp = (int16_t)(temp_float * 10.0f);

    if (temp >= upper_threshold)
    {
        return STATE_HIGH_TEMP;
    }
    else if (temp <= lower_threshold)
    {
        return STATE_LOW_TEMP;;
    }
    else
    {
        return STATE_NORMAL;
    }
}

/******************************* SYSTEM STATE PROCESS **************************************
 
 * @fn          -system_state_process
 * 
 * @brief       -Main loop function for processing the system states. All outputs are being handled here
 * 
 * @param[in]   -Sysyem state 
 *
 * @return      -void.
 * 
 * @note        -none
 
*/


void system_state_process(const SystemState_t s)
{
    switch (s)
    {
    case STATE_HIGH_TEMP:
        redLed_on();
        blueLed_off();
        buzzer_on(FAST_BEEP_PERIOD_MS);
        display_state_high_temp();
        activate_backlight();
        activate_heat_relay();
        disable_cold_relay();
        break;

    case STATE_LOW_TEMP:
        blueLed_on();
        redLed_off();
        buzzer_on(SLOW_BEEP_PERIOD_MS);
        display_state_low_temp();
        activate_backlight();
        activate_cold_relay();
        disable_heat_relay();
        break;

    default:
        redLed_off();
        blueLed_off();
        buzzer_off();
        display_state_normal();
        disable_cold_relay();
        disable_heat_relay();
        break;
    }
}

/******************************* PRIVATE DEFINITIONS ************************************/

static void clear_state_line(void)
{
    lcd_set_page(4);
    lcd_set_column(70);

    for (int i = 0; i < 12; i++)   
    {
        lcd_write_char(' ');
    }
}


static void display_state_normal(){
    char state[] = "State:";
    char normal[] = "Normal";

    lcd_set_page(3);
    lcd_set_column(70);

    for (int i = 0; i < strlen(state); i++) {
        lcd_write_char(state[i]);
    }

    clear_state_line();
    lcd_set_page(4);
    lcd_set_column(70);
  
    for (int i = 0; i < strlen(normal); i++) {
        lcd_write_char(normal[i]);
    }
}

static void display_state_high_temp(){
    char state[] = "State:";
    char high_temp[] = "High Temp";

    lcd_set_page(3);
    lcd_set_column(70);

    for (int i = 0; i < strlen(state); i++) {
        lcd_write_char(state[i]);
    }
    clear_state_line();
    lcd_set_page(4);
    lcd_set_column(70);
  
    for (int i = 0; i < strlen(high_temp); i++) {
        lcd_write_char(high_temp[i]);
    }
}

static void display_state_low_temp(){
    char state[] = "State:";
    char low_temp[] = "Low Temp";

    lcd_set_page(3);
    lcd_set_column(70);

    for (int i = 0; i < strlen(state); i++) {
        lcd_write_char(state[i]);
    }

    clear_state_line();
    lcd_set_page(4);
    lcd_set_column(70);
  
    for (int i = 0; i < strlen(low_temp); i++) {
        lcd_write_char(low_temp[i]);
    }
}

static void redLed_on(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, SET);
}

static void redLed_off(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, RESET);
}

static void blueLed_on(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, SET);
}

static void blueLed_off(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, RESET);
}

static void activate_backlight(void)
{
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, SET);
}

static void activate_heat_relay(void)
{
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_11, RESET);
}

static void disable_heat_relay(void)
{
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_11, SET);
}

static void activate_cold_relay(void)
{
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_12, RESET);
}

static void disable_cold_relay(void)
{
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_12, SET);
}