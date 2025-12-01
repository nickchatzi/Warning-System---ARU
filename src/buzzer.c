#include "buzzer.h"
#include "display.h"

/******************************* PRIVATE DECLARATIONS ************************************/

static volatile uint8_t beep_once = 0;
static volatile uint8_t single_beep_request = 0;

static uint32_t last_toggle_time = 0;
static uint8_t  buzzer_state = 0;


// Current beep request (1, 2, or 3)
static uint8_t beep_count = 0;

// State machine
static uint8_t beep_stage = 0;       // Which beep we are on (0,1,2,...)
static uint8_t buzzer_active = 0;    // 0 = idle, 1 = running

static uint32_t beep_start = 0;

/******************************* BUZZER INIT **************************************
 
 * @fn          -buzzer_init
 * 
 * @brief       -Initialize the function of the buzzer.
 * 
 * @param[in]   -Time in which the buzzer will stay on in ms.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

static void buzzer_init(uint32_t duration_ms)
{
    static uint32_t start_time = 0;
    static uint8_t  active = 0;

    if (active == 0)
    {
        start_time = ms_ticks;
        active = 1;
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_10, SET);  // buzzer ON
    }

    if ((ms_ticks - start_time) >= duration_ms)
    {
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_10, RESET); // buzzer OFF
        active = 0;
        beep_once = 0;    // clear request
    }
}

/**************************** BUZZER SINGLE BEEP ***********************************
 
 * @fn          -buzzer_single_beep
 * 
 * @brief       -Buzz once (for when button is pressed).
 * 
 * @param[in]   -none.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_single_beep(void)
{
  buzzer_init(120);
  single_beep_request = 1;

  if (single_beep_request)
  {
    buzzer_init(120);

    if ((ms_ticks - last_btn_time) >= 120)
      single_beep_request = 0;
  }
}

/**************************** BUZZER DOUBLE BEEP REQ ***********************************
 
 * @fn          -buzzer_double_beep_request
 * 
 * @brief       -Buzz two times.
 * 
 * @param[in]   -none.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_double_beep_request(void)
{
    if (!buzzer_active) {
        buzzer_active = 1;
        beep_count = 2;
        beep_stage = 0;
        beep_start = ms_ticks;   
    }
}

/**************************** BUZZER TRIPLE BEEP REQ ***********************************
 
 * @fn          -buzzer_triple_beep_request
 * 
 * @brief       -Buzz three times.
 * 
 * @param[in]   -none.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_triple_beep_request(void)
{
    if (!buzzer_active) {
        buzzer_active = 1;
        beep_count = 3;
        beep_stage = 0;
        beep_start = ms_ticks;  
    }
}

/******************************* BUZZER UPDATE **************************************
 
 * @fn          -buzzer_update
 * 
 * @brief       -Tone generator for update_min_max.
 * 
 * @param[in]   -none.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_update(void)
{
    if (!buzzer_active)
        return;

    uint32_t elapsed = ms_ticks - beep_start;

    switch (beep_stage)
    {
        case 0: // Beep ON using tone generator
            buzzer_on(8);
            if (elapsed >= 150) {
                GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_10, RESET); 
                beep_stage = 1;
                beep_start = ms_ticks;
            }
            break;

        case 1: // OFF period
            if (elapsed >= 120) {
                if (beep_count > 1) {
                    beep_count--;
                    beep_stage = 0;
                    beep_start = ms_ticks;
                } else {
                    buzzer_active = 0;
                }
            }
            break;
    }
}

/******************************* BUZZER ON **************************************
 
 * @fn          -buzzer_on
 * 
 * @brief       -Set the buzzer.
 * 
 * @param[in]   -Time in which the buzzer will stay on in ms.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_on(uint32_t period)
{
    if ((ms_ticks - last_toggle_time) >= period)
    {
        last_toggle_time = ms_ticks;

        buzzer_state ^= 1;

        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_10);
    }    
}

/******************************* BUZZER OFF **************************************
 
 * @fn          -buzzer_off
 * 
 * @brief       -Reset the buzzer.
 * 
 * @param[in]   -none.
 *
 * @return      -void.
 * 
 * @note        -none
 
*/

void buzzer_off(void)
{
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_10, RESET);
}

