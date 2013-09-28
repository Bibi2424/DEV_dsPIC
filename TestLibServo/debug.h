/*
* Project   : Template dsPIC33F
* File      : debug.h
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
#ifndef DEBUG_H
#define	DEBUG_H
/******************************************************************************/
/* Debug #define Macros                                                       */
/******************************************************************************/

#define BAUDRATE_DEBUG 115200
#define BRGVAL_DEBUG ((FCY / BAUDRATE_DEBUG / 16) - 1)

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void InitDebug(int U1TX, int U1RX);

#endif	/* DEBUG_H */

