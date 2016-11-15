/*
* dsPIC33F & module NRF24L
* Compiler : Microchip xC16
* µC : 33FJ64MC802
* File : SPI_macro.h
* Juin 2013
*    ____________      _           _
*   |___  /| ___ \    | |         | |
*      / / | |_/ /___ | |__   ___ | |_
*     / /  |    // _ \| '_ \ / _ \| __|
*    / /   | |\ \ (_) | |_) | (_) | |_
*   /_/    |_| \_\___/|____/ \___/'\__|
*			      7robot.fr
*/

#ifndef SPI_MACRO_H
#define	SPI_MACRO_H

//Defines
#define SPI_SCK _RB5        // Clock outpout pin,
#define SPI_SO _RB6         // Serial output pin,
#define SPI_SI _RB7         // Serial input pin,
#define SPI_CSN _RB8        // CSN output pin,
#define SPI_CE _RB9         // CE output pin,
#define SPI_IRQ _LATB10       // IRQ input pin,

//Prototypes
void InitSPI(void);

#endif	/* SPI_H */

