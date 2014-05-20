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
#include "user.h"          /* Functions / Parameters                           */

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
_FICD(ICS_PGD3 & JTAGEN_OFF);

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */
volatile uint16_t channel = 0;
volatile uint16_t Dist;
volatile uint16_t Sector[3] = {0,0,0};
volatile uint16_t Old_Sector[3] = {0,0,0};
volatile uint16_t Stick[2];

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    // Initialize IO ports and peripherals.
    ConfigureOscillator();
    //Init debug on UART, TX->RP8, RX->RP9
    InitDebug(8,9);
    printf("Program Running\n");
    //Init User Applications
    InitApp();
    //Init ADC
    InitADC();

    while(1)
    {
        //Do nothing
    }
}

