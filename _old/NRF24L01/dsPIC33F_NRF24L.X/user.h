/*
* dsPIC33F & module NRF24L
* Compiler : Microchip xC16
* µC : 33FJ64MC802
* File : user.h
* Juin 2013
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

/* Microcontroller MIPs (FCY), usefull for delay_ms */
#define SYS_FREQ        80000000 // 80Mips with the internal OSC and full PLL
#define FCY             SYS_FREQ/2

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define led1 _LATA0
#define led2 _LATA1
#define SPI_CE _LATB9
#define SPI_CSN _LATB8

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void Setup(void);           // I/O and Peripheral Initialization
void Loop(void);            // Main Loop


#endif