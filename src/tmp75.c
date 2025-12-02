#include "timers.h"
#include "all_inits.h"
#include "startup.h"
#include "tmp75.h"
#include "buzzer.h"
#include "display.h"

/****************************** PRIVATE DECLARATIONS *********************************/

static float temp_buffer[TEMP_READINGS_PER_HOUR];   /*Ambient temperature buffer of the last 1 hour ((2 readings / min) * 60mins = 120 readings)*/
static uint8_t temp_index = 0;      /*Index variable to track where to store the next reading*/
static uint8_t buffer_count = 0;    /*Holds the ammount of actual data inside the buffer. Basically how many readings have been stored so far*/
volatile uint8_t new_min_event = 0;
volatile uint8_t new_max_event = 0;
static uint8_t  redLed_blink_active = 0;
static uint32_t redLed_start_time = 0;
static uint32_t redLed_blink_duration = 0;

static void redLed_blink_request(uint32_t duration_ms);
static void blueLed_blink_request(uint32_t duration_ms);

/******************************* GLOBAL VARIABLES ************************************/

uint8_t temperature[2];     /*Ambient temperature buffer. Updates every 30 second with the new reading*/
float min_temp = 0.0f;      /*Minimum temperature float*/
float max_temp = 0.0f;      /*Maximum temperature float*/

/************************ INITIALIZE SENSOR ********************************
 
 * @fn          -initTMP75
 *
 * @brief       -Initialize TMP75 by putting him in shutdown mode. 
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -!!!ISSUE: I2C SEND AND RECEIVE DATA BUG!!!!
 
*/

void initTMP75(void)
{
    uint8_t configRegByte = 0x01;

    I2C_PeripheralControl(I2C1, ENABLE);

    I2C_MasterSendData(&I2C1Handle,&configRegByte,1,SLAVE_ADDRESS, I2C_ENABLE_SR);

    I2C_MasterSendData(&I2C1Handle,&configRegByte,1,SLAVE_ADDRESS, I2C_DISABLE_SR);

}

/************************ RECEIVE AMBIENT TEMPERATURE ********************************
 
 * @fn          -receive_temp_measurement
 *
 * @brief       -General purpose timer 14 is being used as a countdown timer. 
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void receive_temp_measurement(void)
{
    uint8_t TempRegByte = 0x00;
    //uint8_t osMode = 0x80;
  
    if (TIM14_SR & (1 << 0)) 
    {
        TIM14_SR &= ~(1 << 0);  

        I2C_PeripheralControl(I2C1, ENABLE);

        //I2C_MasterSendData(&I2C1Handle,&osMode,1,SLAVE_ADDRESS, I2C_DISABLE_SR);

        //delay_ms(150);

        //I2C1_REG_RESET();

        //I2C_PeripheralControl(I2C1, ENABLE);

        I2C_MasterSendData(&I2C1Handle,&TempRegByte,1,SLAVE_ADDRESS, I2C_ENABLE_SR);

        //I2C1_REG_RESET();

        //I2C_PeripheralControl(I2C1, ENABLE);

        I2C_MasterReceiveData(&I2C1Handle, temperature, 2, SLAVE_ADDRESS, I2C_DISABLE_SR);

        delay_ms(15);

        I2C1_REG_RESET();
    }
}

/************************ STORE TEMPERATURE ********************************
 
 * @fn          -store_temperature
 *
 * @brief       -Store the temperature readings of the last hour into a buffer. 
 * 
 * @param[in]   -Ambient temperature
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void store_temperature(float temp)
{
    temp_buffer[temp_index] = temp; //write the new reading into the buffer at index = 0,1,2,3...

    if (buffer_count < TEMP_READINGS_PER_HOUR)  //for each new reading, increase buffer count until buffer is full (1 hour has passed).
        buffer_count++;

    temp_index++;

    if (temp_index >= TEMP_READINGS_PER_HOUR)   //Reading 121 will over-write reading 1, so buffer will always store the readings of the last hour.
        temp_index = 0;
}


/************************ UPDATE MIN/MAX ********************************
 
 * @fn          -update_min_max
 *
 * @brief       -Checks last temp reading and updates the min or max variables if needed. 
 *              -If new min or max reading recorded buzz twice for min and 3 times for max.
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

void update_min_max(void)
{
    if (buffer_count == 0) return;     //If no data has been stored yet exit

    float old_min = min_temp;       //Re-initialize min and max as the first temperature readings.
    float old_max = max_temp;

    min_temp = temp_buffer[0];
    max_temp = temp_buffer[0];

    for (int i = 1; i < buffer_count; i++)
    {
        if (temp_buffer[i] < min_temp)
            min_temp = temp_buffer[i];

        if (temp_buffer[i] > max_temp)
            max_temp = temp_buffer[i];
    }

    // Trigger event if new min appeared
    if (min_temp < old_min)
    {     
        new_min_event = 1;
        buzzer_double_beep_request();
        activate_backlight_counter();
        blueLed_blink_request(2000);
    }

    // Trigger event if new max appeared
    if (max_temp > old_max) 
    {
        new_max_event = 1;
        buzzer_triple_beep_request();
        activate_backlight_counter();
        redLed_blink_request(2000);
    }           
}

/************************ RED LED BLINK REQUEST ********************************
 
 * @fn          -redLed_blink_request
 *
 * @brief       -Blink the red led once             
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

static void redLed_blink_request(uint32_t duration_ms)
{
    redLed_blink_duration = duration_ms;
    redLed_blink_active = 1;
    redLed_start_time = ms_ticks;
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, SET);
}

/************************ BLUE LED BLINK REQUEST ********************************
 
 * @fn          -blueLed_blink_request
 *
 * @brief       -Blink the blue led once             
 * 
 * @param[in]   -
 *
 * @return      -void
 * 
 * @note        -none
 
*/

static void blueLed_blink_request(uint32_t duration_ms)
{
    redLed_blink_duration = duration_ms;
    redLed_blink_active = 1;
    redLed_start_time = ms_ticks;
    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, SET);
}
