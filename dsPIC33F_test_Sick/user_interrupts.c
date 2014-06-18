/*
* Project   : Template dsPIC33F
* File      : user_interrupt.c
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
#include "user.h"          /* Function / Parameters                           */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

uint16_t time = 0;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void ConfigureOscillator(void)
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBDbits.PLLDIV = 41; // M=43
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE  = 0; // N2=2
    // Fosc = M/(N1.N2)*Fin
}

void InitApp(void)
{
    // activation de la priorité des interruptions
    _NSTDIS = 0;

    //Init des E/S
    _TRISA0 = 0;
    _TRISB8 = 0;
    led = 0;
    led2 = 0;

    //Configuration du Timer 2, période 20ms pour l'OC1
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, 3125);
    ConfigIntTimer2(T2_INT_ON & T2_INT_PRIOR_4);

}

/******************************************************************************/
/*                                                                            */
/*                      Interrupt Routines                                    */
/*                                                                            */
/******************************************************************************/

void __attribute__((interrupt,auto_psv)) _T2Interrupt(void)
{
    //led = led^1;            // On bascule l'état de la LED

    _T2IF = 0;              // On baisse le FLAG
}


/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs) User */
/* Guide for an up to date list of the available interrupt options.           */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* dsPIC33F Primary Interrupt Vector Names:                                   */
/*                                                                            */
/* _INT0Interrupt      _C1Interrupt                                           */
/* _IC1Interrupt       _DMA3Interrupt                                         */
/* _OC1Interrupt       _IC3Interrupt                                          */
/* _T1Interrupt        _IC4Interrupt                                          */
/* _DMA0Interrupt      _IC5Interrupt                                          */
/* _IC2Interrupt       _IC6Interrupt                                          */
/* _OC2Interrupt       _OC5Interrupt                                          */
/* _T2Interrupt        _OC6Interrupt                                          */
/* _T3Interrupt        _OC7Interrupt                                          */
/* _SPI1ErrInterrupt   _OC8Interrupt                                          */
/* _SPI1Interrupt      _DMA4Interrupt                                         */
/* _U1RXInterrupt      _T6Interrupt                                           */
/* _U1TXInterrupt      _T7Interrupt                                           */
/* _ADC1Interrupt      _SI2C2Interrupt                                        */
/* _DMA1Interrupt      _MI2C2Interrupt                                        */
/* _SI2C1Interrupt     _T8Interrupt                                           */
/* _MI2C1Interrupt     _T9Interrupt                                           */
/* _CNInterrupt        _INT3Interrupt                                         */
/* _INT1Interrupt      _INT4Interrupt                                         */
/* _ADC2Interrupt      _C2RxRdyInterrupt                                      */
/* _DMA2Interrupt      _C2Interrupt                                           */
/* _OC3Interrupt       _DCIErrInterrupt                                       */
/* _OC4Interrupt       _DCIInterrupt                                          */
/* _T4Interrupt        _DMA5Interrupt                                         */
/* _T5Interrupt        _U1ErrInterrupt                                        */
/* _INT2Interrupt      _U2ErrInterrupt                                        */
/* _U2RXInterrupt      _DMA6Interrupt                                         */
/* _U2TXInterrupt      _DMA7Interrupt                                         */
/* _SPI2ErrInterrupt   _C1TxReqInterrupt                                      */
/* _SPI2Interrupt      _C2TxReqInterrupt                                      */
/* _C1RxRdyInterrupt                                                          */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.  There is no Alternate Vector or 'AIVT' for the 33E family.   */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <compiler installation directory>/doc directory for the latest compiler    */
/* release.                                                                   */
/*                                                                            */
/******************************************************************************/

//Classe la distance en secteur
/*void __attribute__ ((interrupt, auto_psv)) _ADC1Interrupt(void)
 {
    StartDebugTimer();

    switch(channel)
    {
        case 0:
            Dist = ADC1BUF0>>6;
            if(Sector[0] > Dist+1 || (Dist > 0 && Sector[0] < Dist-1))
            {
                Sector[0] = Dist;
                printf("Capteur 1, New Sector : %d (Value : %d)\n", Sector[0]>>1, Dist);
            }
            _CH0SA = Dist_2;
        break;
        case 1:
            Dist  = ADC1BUF0>>6;
            if(Sector[1] > Dist+1 || (Dist > 0 && Sector[1] < Dist-1))
            {
                Sector[1] = Dist;
                printf("Capteur 2, New Sector : %d (Value : %d)\n", Sector[1]>>1, Dist);
            }
            _CH0SA = Dist_3;
        break;
        case 2:
            Dist  = ADC1BUF0>>6;
            if(Sector[2] > Dist+1 || (Dist > 0 && Sector[2] < Dist-1))
            {
                Sector[2] = Dist;
                printf("Capteur 3, New Sector : %d (Value : %d)\n", Sector[0]>>1, Dist);
            }
            _CH0SA = Stick_X;
        break;
        case 3:
            Stick[0]  = ADC1BUF0;
            _CH0SA = Stick_Y;
        break;
        case 4:
            Stick[1]  = ADC1BUF0;
            _CH0SA = Dist_1;
        break;
    }
    channel = (channel+1)%5;

    time = ReadStopDebugTimer();
    if(channel  ==0)
    {led = led^1;}

    _AD1IF = 0;        //Clear the interrupt flag
 }*/