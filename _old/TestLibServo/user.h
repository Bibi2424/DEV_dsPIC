/*
* Project   : Test Lib Servo
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
/* System Level #define Macros                                                */
/******************************************************************************/

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        80000000 //7370000L
#define FCY             SYS_FREQ/2

#define UNLOCK_RP        __builtin_write_OSCCONL(OSCCON & 0xBF)
#define LOCK_RP          __builtin_write_OSCCONL(OSCCON | 0x40)

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define led _LATA0
#define led1 _LATA1

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void ConfigureOscillator(void);    /* Handles clock switching/osc initialization */

void InitApp(void);             /* I/O and Peripheral Initialization          */

#endif  /* USER_H */