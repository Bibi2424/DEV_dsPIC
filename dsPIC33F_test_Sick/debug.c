/*
* Project   : Template dsPIC33F
* File      : debug.c
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

#include <p33Fxxxx.h>      /* Includes device header file                     */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include "user.h"          /* Function / Parameters                           */
#include <uart.h>
#include "debug.h"



/******************************************************************************/
/* Debug Functions                                                            */
/******************************************************************************/

void InitDebug(int U1TX, int U1RX)
{
    //Timer0 pour les temps
    OpenTimer1(T2_OFF & T2_GATE_OFF & T2_PS_1_256 & T2_SOURCE_INT, 65535);

    //UART Module
    OpenUART1(UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW
        & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK
        & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN
        & UART_NO_PAR_8BIT & UART_1STOPBIT,
          UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO
        & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_TX_BUF_NOT_FUL & UART_INT_RX_CHAR
        & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR,
          BRGVAL_DEBUG);

    UNLOCK_RP;
    switch(U1TX)
    {
        case 0: _RP0R=3; break;
        case 1: _RP1R=3; break;
        case 2: _RP2R=3; break;
        case 3: _RP3R=3; break;
        case 4: _RP4R=3; break;
        case 5: _RP5R=3; break;
        case 6: _RP6R=3; break;
        case 7: _RP7R=3; break;
        case 8: _RP8R=3; break;
        case 9: _RP9R=3; break;
        case 10: _RP10R=3; break;
        case 11: _RP11R=3; break;
        case 12: _RP12R=3; break;
        case 13: _RP13R=3; break;
        case 14: _RP14R=3; break;
        case 15: _RP15R=3; break;
        case 16: _RP16R=3; break;
        case 17: _RP17R=3; break;
        case 18: _RP18R=3; break;
        case 19: _RP19R=3; break;
        case 20: _RP20R=3; break;
        case 21: _RP21R=3; break;
        case 22: _RP22R=3; break;
        case 23: _RP23R=3; break;
        case 24: _RP24R=3; break;
        case 25: _RP25R=3; break;
    }
    _U1RXR = U1RX; // RP9  = U1RX (p.165)
    LOCK_RP;

    ConfigIntUART1(UART_RX_INT_EN & UART_TX_INT_EN & UART_RX_INT_PR4 & UART_TX_INT_PR5);
}

void StartDebugTimer(void)
{
    T1CONbits.TON = 1;
}

void StopDebugTimer(void)
{
    T1CONbits.TON = 0;
}

uint16_t ReadDebugTimer(void)
{
    return TMR1;
}

uint16_t ReadStopDebugTimer(void)
{
    uint16_t time;

    T1CONbits.TON = 0;
    time = TMR1;
    TMR1 = 0;
    return time;
}

//TODO Ecrire un parseur pour établir une communication entre le pc et le PIC

/******************      RX Interrupt        *********************/
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    _U1RXIF = 0;      // On baisse le FLAG
}

/******************      TX Interrupt        ********************/
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
   _U1TXIF = 0; // clear TX interrupt flag
}