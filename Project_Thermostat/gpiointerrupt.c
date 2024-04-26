/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Definitions */
#define DISPLAY(x) UART_write(uart, &output, x)

typedef struct task {
    int state;  // Current state of the task
    unsigned long period;   // Rate at which the task should tick
    unsigned long elapsedTime;  // Time since task's previous tick
    int (*TickFct)(int);    // Function to call for task's tick
} task;


// Driver Handles - Global variables
I2C_Handle i2c;
UART_Handle uart;
Timer_Handle timer0;

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// UART Global Variables
char output[64];
int bytesToSend;

// Timer Global Variables
volatile unsigned char TimerFlag = 0;

// Thermostat Global Variables
const unsigned char numTasks = 3;
const unsigned long timerPeriod = 100; // 0.1s, 100ms, 100000us
const unsigned long checkButtonPeriod = 200;   // 0.2s, 200ms, 200000us
const unsigned long checkTemperaturePeriod = 500;  // 0.5s, 500ms, 500000us
const unsigned long checkDisplayPeriod = 1000; // 1s, 1000ms, 1000000us

enum Button_States {Button_Idle, Button_Increase, Button_Decrease} Button_State;
int TickFct_AdjustSetPoint(int state);

enum Heating_States {Heating_Off, Heating_On, Heating_Start} Heating_State;
int TickFct_ProcessHeating(int state);

enum Display_States {Display_Start, Display_Off, Display_On};
int TickFct_DisplayStatus(int state);

int16_t roomTemperature = 0;    // Room Temperature initialized to 0.
int16_t setPoint = 25;  // Thermostat set-point initialized to 25°C (77°F).
int secondsSinceReset = 0; // Total time elapsed since thermostat began running in seconds.
int buttonFlag = -1; // Button flag triggered by interrupt initiated to -1 for idle. 0 for decreasing and 1 for increasing

task tasks[numTasks];

//I2C Initialization
// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses.

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
         i2cTransaction.slaveAddress = sensors[i].address;
         txBuffer[0] = sensors[i].resultReg;

         DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
         if (I2C_transfer(i2c, &i2cTransaction))
         {
             DISPLAY(snprintf(output, 64, "Found\n\r"));
             found = true;
             break;
         }
         DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

// UART Initialization
void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;

}


// Timer Initialization
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; // 0.1s, 100ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Change button flag for decreasing set-point */
    if (buttonFlag != 0) {
        buttonFlag = 0;
    }
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Change button flag for increasing set-point */
    if (buttonFlag != 1) {
        buttonFlag = 1;
    }
}


// GPIO Initialization
void initGPIO(void)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

}

//Function to get current room temperature
int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
        temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }

    return temperature;
}


// TASK 1: Adjust Temperature Set-Point
int TickFct_AdjustSetPoint(int state) {

    if (buttonFlag == 0) {
        Button_State = Button_Decrease;
    }

    if (buttonFlag == 1) {
        Button_State = Button_Increase;
    }

    switch (state)
    {
        case Button_Increase:
            if (setPoint < 100)  // Temperature set-point cannot go above 100°C (212°F).
            {
                setPoint++;
            }

            Button_State = Button_Idle;

            break;

        case Button_Decrease:
            if (setPoint > 0)  // Temperature set-point cannot go below 100°C (32°F).
            {
                setPoint--;
            }

            Button_State = Button_Idle;

            break;
    }

    buttonFlag = -1;

    state = Button_State;

    return state;
}


// TASK 2: Read room temperature and turn heat on or off if necessary
int TickFct_ProcessHeating(int state) {

    roomTemperature = readTemp();

    if (roomTemperature < setPoint)  // Turn on the heat.
    {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        Heating_State = Heating_On;
    }
    else                                // Turn off the heat.
    {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        Heating_State = Heating_Off;
    }

    state = Heating_State;

    return state;
}


// TASK3: Display Server Data
int TickFct_DisplayStatus(int state) {

    state = Display_Off;

    if (secondsSinceReset != 0) {
        DISPLAY(snprintf(output,
                             64,
                             "<%02d,%02d,%d,%04d>\n\r",
                             roomTemperature,
                             setPoint,
                             Heating_State,
                             secondsSinceReset));
        state = Display_On;
    }

    secondsSinceReset++;

    return state;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    unsigned char i = 0;

    tasks[i].state = Button_Idle;
    tasks[i].period = checkButtonPeriod;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_AdjustSetPoint;
    ++i;

    tasks[i].state = Heating_Start;
    tasks[i].period = checkTemperaturePeriod;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_ProcessHeating;
    ++i;

    tasks[i].state = Display_Start;
    tasks[i].period = checkDisplayPeriod;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &TickFct_DisplayStatus;

    //initialize drivers
    initUART();
    initI2C();
    initGPIO();
    initTimer();

    // Loop forever.
    while (1)
    {
        unsigned int i = 0;
        for (i = 0; i < numTasks; ++i)
        {
            if ( tasks[i].elapsedTime >= tasks[i].period )
            {
                tasks[i].state = tasks[i].TickFct(tasks[i].state);
                tasks[i].elapsedTime = 0;
             }
             tasks[i].elapsedTime += timerPeriod;
        }

        // Wait for timer period.
        while(!TimerFlag){}
        // Set the timer flag variable to FALSE.
        TimerFlag = 0;
    }

    return (NULL);
}
