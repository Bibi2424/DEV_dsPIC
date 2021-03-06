/*
* Project   : Template dsPIC33F
* File      : user.h
* Compiler  : Microchip xC16
* �C        : 33FJ64MC802
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
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        80000000 //7370000L
#define FCY             SYS_FREQ/2

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <stdio.h>         /* Include printf                                  */
#include "debug.h"
#include "timer.h"
#include "outcompare.h"
#include <libpic30.h>

/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

#define UNLOCK_RP        __builtin_write_OSCCONL(OSCCON & 0xBF)
#define LOCK_RP          __builtin_write_OSCCONL(OSCCON | 0x40)

//Macro pour mettre le bit 'bit' du port 'port' � 'value'
//(ex: SET_BIT(LATA, 0, 1) == LATAbits.LATA0 = 1 == _LATA0 = 1)
#define SET_BIT(port, bit, value) value ? port | (1<<bit) : port & (~(1<<bit))
//si value=1 on met � 1 le bit dans le port (OR), si value=0 on le met � 0 (AND)

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define led _LATA0

//ADC
#define Dist_1  1
#define Dist_2  2
#define Dist_3  3
#define Current_G 4
#define Current_D 5

#define TEMPS_DIST 4000 //temps_Dist(instr) = temps_Dist(s)/temps_Current(s)

#define SEUIL_SICK 500  //TBC
#define MARGE_SICK 100  //TBC
#define SEUIL_HAUT SEUIL_SICK+MARGE_SICK
#define SEUIL_BAS SEUIL_SICK-MARGE_SICK

//#define ANGLE(angle) (int)((float)angle*0.8667+156)

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void ConfigureOscillator(void);    /* Handles clock switching/osc initialization */

void InitApp(void);             /* I/O and Peripheral Initialization          */

void InitADC(void);

int Angle(float angle);

#endif  /* USER_H */