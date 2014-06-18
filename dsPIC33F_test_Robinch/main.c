/*
* Project   : Template dsPIC33F
* File      : main.c
* Compiler  : Microchip xC16
* µC        : 33FJ64MC802
* Start Date: Juillet 2012
* Revision  : 2.0
*    ____________      _           _
*   |___  /| ___ \    | |         | |
*      / / | |_/ /___ | |__   ___ | |_
*     / /  |    // _ \| '_ \ / _ \| __|
*    / /   | |\ \ (_) | |_) | (_) | |_
*   /_/    |_| \_\___/|____/ \___/'\__|
*			      7robot.fr
*/
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include "user.h"          /* Functions / Parameters                          */
#include "libpic30.h"


/******************************************************************************/
/* Configuartion                                                              */
/******************************************************************************/

// Select Oscillator and switching.
_FOSCSEL(FNOSC_FRCPLL & IESO_OFF);
// Select clock.
_FOSC(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
// Watchdog Timer.
_FWDT(FWDTEN_OFF);
// Select debug channel.
_FICD(ICS_PGD1 & JTAGEN_OFF);

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */
volatile uint16_t i = 0;
volatile uint16_t cpt = 0;
volatile uint16_t channel = 0;
volatile uint16_t channel_current = 0;
volatile uint16_t channel_dist = 0;
volatile uint16_t Buff_adc_current[2];
volatile uint16_t Buff_adc_dist[3] = {0,0,0};
volatile uint16_t Buff_adc_sector[3] = {0,0,0};

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    // Initialize IO ports and peripherals.
    ConfigureOscillator();
    InitApp();
    printf("Program Running\n");
    InitADC();


        OC1RS = 234;
        _LATC4 = 1;

    while(1)
    {
        //Do nothing
        __delay_ms(100);
    }
}

