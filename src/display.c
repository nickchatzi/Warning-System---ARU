#include "display.h"
#include "lcd.h"
#include "tmp75.h"
#include "startup.h"
#include "main.h"
#include "buzzer.h"
#include "timers.h"

/******************************* GLOBAL VARIABLES ************************************/

volatile uint8_t selected_option = 0;       
volatile uint8_t confirmed_selection = 0; 

extern volatile uint8_t flag_up = 0;
extern volatile uint8_t flag_down = 0;
extern volatile uint8_t flag_enter = 0;

volatile int16_t lower_threshold = 100;
volatile int16_t upper_threshold = 250;

/******************************* PRIVATE DECLARATIONS ************************************/

static void user_toggle_selection(void);

extern uint32_t last_btn_time = 0;


/******************************* DISPLAY TEMPERATURE **************************************
 
 * @fn          -convertTemp
 *
 * @brief       -covert the 16 bit temperature buffer into a float.
 * 
 * @param[in]   -
 *
 * @return      -The ambient temperature as a float variable.
 * 
 * @note        -none
 
*/

float convertTemp(void)
{
  int16_t raw = (temperature[0] << 8) | temperature[1];
  raw >>= 4;
  return raw * 0.0625f;
}

/******************************* DISPLAY TEMPERATURE **************************************
 
 * @fn          -displayTemp
 * 
 * @brief       -Display the ambient temperature on the LCD.
 * 
 * @param[in]   -Ambient temperature 
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

static void displayTemp(float tempC)
{
  char buffer[16];
  char line[] = "---------------------";
  sprintf(buffer, "%.1f C", tempC);

  lcd_set_page(2);
  lcd_set_column(1);
  
  for (int i = 0; i < strlen(line); i++) {
    lcd_write_char(line[i]);
  }

  for(int x = 10; x < 128; x++)
  {
    lcd_set_page(4);
    lcd_set_column(x);
    lcd_send_data(0x00);

    lcd_set_page(5);
    lcd_set_column(x);
    lcd_send_data(0x00);
  }
  current_page   = 3;   // top half
  current_column = 10;  // left position

  lcd_write_string_big(buffer);

  lcd_set_page(5);
  lcd_set_column(1);
  
  for (int i = 0; i < strlen(line); i++) {
    lcd_write_char(line[i]);
  }
}

void display_temp()
{
  if(new_data_ready)
  {
    new_data_ready = 0;

    update_min_max();

    displayTemp(latest_temp);
  }
}

/******************************* DISPLAY UPPER TEMPERATURE **************************************
 
 * @fn          -display_upper_threshold
 * 
 * @brief       -Display the upper threshold temperature on the LCD.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void display_upper_threshold(void)
{
  char buffer[24];

  int16_t value = upper_threshold;

  char sign = (value < 0) ? '-' : ' ';

  if (value < 0)
    value = -value;  

  int whole   = value / 10;
  int decimal = value % 10;

  lcd_set_page(0);
  lcd_set_column(8);

  sprintf(buffer, "High thres: %c%d.%d C", sign, whole, decimal);

  for (int i = 0; buffer[i] != '\0'; i++)
    lcd_write_char(buffer[i]);
}

/******************************* DISPLAY LOWER TEMPERATURE **************************************
 
 * @fn          -display_lower_threshold
 * 
 * @brief       -Display the lower threshold temperature on the LCD.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void display_lower_threshold(void)
{
  char buffer[24];

  int16_t value = lower_threshold;

  char sign = (value < 0) ? '-' : ' ';

  if (value < 0)
    value = -value;

  int whole   = value / 10;
  int decimal = value % 10;

  lcd_set_page(1);
  lcd_set_column(8);

  sprintf(buffer, "Low thres:  %c%d.%d C", sign, whole, decimal);

  for (int i = 0; buffer[i] != '\0'; i++)
    lcd_write_char(buffer[i]);
}

/******************************* DISPLAY MAXIMUM TEMPERATURE **************************************
 
 * @fn          -display_max_temp
 * 
 * @brief       -Display the maximum reading temperature on the LCD.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void display_max_temp(void)
{
  char buffer[24];

  lcd_set_page(6);
  lcd_set_column(8);

  sprintf(buffer, "Max Temp: %.1f C", max_temp);

  for (int i = 0; buffer[i] != '\0'; i++)
    lcd_write_char(buffer[i]);
}

/******************************* DISPLAY MINIMUM TEMPERATURE **************************************
 
 * @fn          -display_min_temp
 * 
 * @brief       -Display the minimum reading temperature on the LCD.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/
void display_min_temp(void)
{
  char buffer[24];

  lcd_set_page(7);
  lcd_set_column(8);

  sprintf(buffer, "Min Temp: %.1f C", min_temp);

  for (int i = 0; buffer[i] != '\0'; i++)
    lcd_write_char(buffer[i]);
}

/************************ MAIN THRESHOLD TEMPERATURE SETTING ********************************
 
 * @fn          -main_threshold_temp_set
 * 
 * @brief       -This is the main function that sets the upper or lower threshold depending the indicator's position.
 *              -The value will be increased or decreased by 0.5.
 *              -Upper threshold cannot go below lower threshold and lower theshold cannot go above upper threshold.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void main_threshold_temp_set(void)
{
  if ((ms_ticks - last_btn_time) >= 150)
  {
    if (flag_enter)
    {
      flag_enter = 0;
      activate_backlight_counter();
      user_toggle_selection();
      last_btn_time = ms_ticks;
      buzzer_single_beep();
    }

    if (flag_up)
    {
      flag_up = 0;
      buzzer_single_beep();
      activate_backlight_counter();

      if (selected_option == 0)
      {
        upper_threshold += 5;
        if (upper_threshold > 1000) upper_threshold = 1000;
      }
      else
      {
        lower_threshold += 5;
        if (lower_threshold > 1000) lower_threshold = 1000;
        if (lower_threshold > upper_threshold) lower_threshold = upper_threshold;
      }

      last_btn_time = ms_ticks;
    }

    if (flag_down)
    {
      flag_down = 0;
      buzzer_single_beep();
      activate_backlight_counter();

      if (selected_option == 0)
      {
        upper_threshold -= 5;
        if (upper_threshold < -400) upper_threshold = -400;
        if (upper_threshold < lower_threshold) upper_threshold = lower_threshold;
      }
      else
      {
        lower_threshold -= 5;
        if (lower_threshold < -400) lower_threshold = -400;
      }

      last_btn_time = ms_ticks;
    }
  }
  else
  {
    flag_enter = flag_up = flag_down = 0;
  }
}

/************************ TOGGLE SELECTION ********************************
 
 * @fn          -user_toggle_selection
 * 
 * @brief       -Change the indicator's position and update global var "selected option" when the button "enter" is being pressed.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

static void user_toggle_selection(void)
{
  // erase both arrows
  lcd_set_page(0);
  lcd_set_column(1);
  lcd_write_char(' ');

  lcd_set_page(1);
  lcd_set_column(1);
  lcd_write_char(' ');

  selected_option ^= 1;

  lcd_set_page(selected_option);
  lcd_set_column(1);
  lcd_write_char('>');
}

void activate_backlight_counter()
{
  TIM3->CNT = 0;
  TIM3_CR1  = 1;    // start timer
  GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, SET); // backlight ON
}