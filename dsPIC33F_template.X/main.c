/*
* Template dsPIC33F
* Compiler : Microchip xC16
* µC : 33FJ64MC802
* Juillet 2012
*    ____________      _           _
*   |___  /| ___ \    | |         | |
*      / / | |_/ /___ | |__   ___ | |_
*     / /  |    // _ \| '_ \ / _ \| __|
*    / /   | |\ \ (_) | |_) | (_) | |_
*   /_/    |_| \_\___/|____/ \___/'\__|
*			      7robot.fr
*/

#include <p33Fxxxx.h>      /* Includes device header file                     */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "user.h"        /* Function / Parameters                           */
#include <libpic30.h>


/******************************************************************************/
/* Configuartion                                                              */
/******************************************************************************/

// Select Oscillator and switching.
_FOSCSEL(FNOSC_FRCPLL & IESO_OFF);
// Select clock.
/* IOL1WAY_OFF (IMPORTANT, ce registre bloque le remappage des pins qui ne peux
   etre fait qu'une seule fois, il faut donc le mettre à OFF pour les AX12 !!*/
_FOSC(POSCMD_NONE & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD);
// Watchdog Timer.
_FWDT(FWDTEN_OFF);
// Select debug channel.
_FICD(ICS_PGD1 & JTAGEN_OFF);


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBDbits.PLLDIV = 41; // M=43
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE  = 0; // N2=2
    // Fosc = M/(N1.N2)*Fin

    // activation de la priorité des interruptions
    _NSTDIS = 0;

    Setup();

    while (1)
    {
        Loop();
    }
}

