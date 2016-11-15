/*
* Project   : Template dsPIC33F
* File      : user.h
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
#ifndef USER_H
#define USER_H
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/
#include <p33Fxxxx.h>      /* Includes device header file                     */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <stdio.h>         /* Include printf                                  */
#include <xc.h>
#include "debug.h"
#include "timer.h"
#include "outcompare.h"

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        80000000 //7370000L
#define FCY             SYS_FREQ/2

#define UNLOCK_RP        __builtin_write_OSCCONL(OSCCON & 0xBF)
#define LOCK_RP          __builtin_write_OSCCONL(OSCCON | 0x40)

//Macro pour mettre le bit 'bit' du port 'port' à 'value'
//(ex: SET_BIT(LATA, 0, 1) == LATAbits.LATA0 = 1 == _LATA0 = 1)
#define SET_BIT(port, bit, value) value ? port | (1<<bit) : port & (~(1<<bit))
//si value=1 on met à 1 le bit dans le port (OR), si value=0 on le met à 0 (AND)

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define led _LATA0

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void ConfigureOscillator(void);    /* Handles clock switching/osc initialization */

void InitApp(void);             /* I/O and Peripheral Initialization          */

#endif  /* USER_H */