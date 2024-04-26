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
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/** Morse Code message states */
enum Morse_Messages {SOS_Message, OK_Message} Current_Message;

enum SOS_States {
    SOS_Start, SOS_s1On, SOS_s1Off,
    SOS_letter1Off, SOS_oOn1, SOS_oOn2,
    SOS_oOn3, SOS_oOff, SOS_letter2Off,
    SOS_s2On, SOS_s2Off, SOS_wordOff
} SOS_State;

enum OK_States {
    OK_Start, OK_oOn1, OK_oOn2,
    OK_oOn3, OK_oOff, OK_letterOff,
    OK_kOn1, OK_kOn2, OK_kOn3,
    OK_kOn4, OK_kOff, OK_wordOff
} OK_State;

int s1;
int o;
int s2;
int k;
int off;

/**
 *  ======== gpioButtonFxn ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0 or CONFIG_GPIO_BUTTON_1.
 *  CONFIG_GPIO_BUTTON_1 may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn(uint_least8_t index)
{
    switch(Current_Message) {
        case SOS_Message:
            Current_Message = OK_Message;
            break;
        case OK_Message:
            Current_Message = SOS_Message;
            SOS_State = SOS_Start;
            break;
        default:
            break;
    }
}


/*
 *  ======== timerCallback ========
 *  Callback function for the timer interrupt.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    if (Current_Message == SOS_Message) { // SOS_State Transitions

        switch(SOS_State) {
            case SOS_Start:
                s1 = 0;
                o = 0;
                s2 = 0;
                off = 0;

                SOS_State = SOS_s1On;
                break;

            case SOS_s1On:
                s1++;

                SOS_State = SOS_s1Off;
                break;

            case SOS_s1Off:
                if (s1 >= 3) {
                    off = 0;

                    SOS_State = SOS_letter1Off;
                }
                else if (s1 < 3) {
                    SOS_State = SOS_s1On;
                }
                break;

            case SOS_letter1Off:
                if (off < 2) {
                    SOS_State = SOS_letter1Off;
                }
                else if (off >= 2) {
                    SOS_State = SOS_oOn1;
                }
                break;

            case SOS_oOn1:
                o++;

                SOS_State = SOS_oOn2;
                break;

            case SOS_oOn2:
                SOS_State = SOS_oOn3;
                break;

            case SOS_oOn3:
                SOS_State = SOS_oOff;
                break;

            case SOS_oOff:
                if (o < 3) {
                    SOS_State = SOS_oOn1;
                }
                else if (o >= 3) {
                    off = 0;

                    SOS_State = SOS_letter2Off;
                }
                break;

            case SOS_letter2Off:
                if (off < 2) {
                    SOS_State = SOS_letter2Off;
                }
                else if (off >= 2) {
                    SOS_State = SOS_s2On;
                }
                break;

            case SOS_s2On:
                s2++;

                SOS_State = SOS_s2Off;
                break;

            case SOS_s2Off:
                if (s2 >= 3) {
                    off = 0;

                    SOS_State = SOS_wordOff;
                }
                else if (s2 < 3) {
                    SOS_State = SOS_s2On;
                }
                break;

            case SOS_wordOff:
                if (off >= 6) {
                    SOS_State = SOS_Start;
                }
                else if (off < 6) {
                    SOS_State = SOS_wordOff;
                }
                break;

            default:
                break;

        }

        switch (SOS_State) { // SOS_State Actions
            case SOS_s1On:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                break;

            case SOS_s1Off:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                break;

            case SOS_letter1Off:
                off++;
                break;

            case SOS_oOn1:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                break;

            case SOS_oOff:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;

            case SOS_letter2Off:
                off++;
                break;

            case SOS_s2On:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                break;

            case SOS_s2Off:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                break;

            case SOS_wordOff:
                off++;
                break;

            default:
                break;
        }
    }
    else if (Current_Message == OK_Message) {
        switch(OK_State) { // OK_State Transitions
            case OK_Start:
                o = 0;
                k = 0;
                off = 0;

                OK_State = OK_oOn1;
                break;

            case OK_oOn1:
                o++;

                OK_State = OK_oOn2;
                break;

            case OK_oOn2:
                OK_State = OK_oOn3;
                break;

            case OK_oOn3:
                OK_State = OK_oOff;
                break;

            case OK_oOff:
                if (o < 3) {
                    OK_State = OK_oOn1;
                }
                else if (o >= 3) {
                    off = 0;

                    OK_State = OK_letterOff;
                }
                break;

            case OK_letterOff:
                if (off < 2) {
                    OK_State = OK_letterOff;
                }
                else if (off >= 2) {
                    OK_State = OK_kOn1;
                }
                break;

            case OK_kOn1:
                k++;

                OK_State = OK_kOn2;
                break;

            case OK_kOn2:
                OK_State = OK_kOn3;
                break;

            case OK_kOn3:
                OK_State = OK_kOff;
                break;

            case OK_kOff:
                if (k == 1) {
                    OK_State = OK_kOn4;
                }
                else if (k == 2 ) {
                    OK_State = OK_kOn1;
                }
                else if (k >= 3) {
                    off = 0;

                    OK_State = OK_wordOff;
                }
                break;

            case OK_kOn4:
                k++;

                OK_State = OK_kOff;
                break;

            case OK_wordOff:
                if (off >= 6) {
                    OK_State = OK_Start;
                }
                else if (off < 6) {
                    OK_State = OK_wordOff;
                }
                break;

            default:
                break;
        }

        switch (OK_State) { // OK_State Actions
            case OK_oOn1:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                break;

            case OK_oOff:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;

            case OK_letterOff:
                off++;
                break;

            case OK_kOn1:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                break;

            case OK_kOff:
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                break;

            case OK_kOn4:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                break;

            case OK_wordOff:
                off++;
                break;

            default:
                break;
        }
    }
}

/**
 *  ======== timerInit ========
 *  Initialization function for the timer interrupt on timer0.
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
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
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /** Call driver init functions */
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /** Begin with LED_0 and LED_1 turned off */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    /** Begin with Morse_Messages states set to SOS_Message */
    Current_Message = SOS_Message;

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn);

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
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    return (NULL);
}
