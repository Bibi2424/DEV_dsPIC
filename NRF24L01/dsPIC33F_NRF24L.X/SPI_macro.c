/*
* dsPIC33F & module NRF24L
* Compiler : Microchip xC16
* µC : 33FJ64MC802
* File : SPI_macro.c
* Juin 2013
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

#include "SPI_macro.h"
#include "NRF24L01.h"
#include <spi.h>

/******************************************************************************/
/* SPI Macro                                                                  */
/******************************************************************************/

void InitSPI(void)
{
    //On map les pins du SPI
    _RP5R = 0b01000;    //SPI1 clock outpout sur RP5
    _RP6R = 0b00111;    //SPI1 data outpout sur RP6
    _SDI1R = 7;         //SPI1 data input sur RP7
    _RP8R = 0b01001;    //SPI1 slave select outpout sur RP8
    //_TRISB8 = 0;    si SSX ne marche pas
    _TRISB9 = 0;        //SPI CE outpout
    _TRISB10 = 1;       //SPI Interrupt input sur RB10

    //On initialise les I/O pins
    SPI_CE = 0;

    //Configuration de SPI1CON2
    _FRMEN = 1;     //Framed SPIx support enabled (SSx pin used as frame sync pulse input/output)
    _SPIFSD = 0;    //Frame sync pulse output (master)
    _FRMPOL = 0;    //Frame sync pulse is active-low                TODO : à vérifier
    _FRMDLY = 0;    //Frame sync pulse precedes first bit clock     TODO : à vérifier

    //Configuration de SPI1CON1
    _DISSCK = 0;                //Internal SPI clock is enabled
    _DISSDO = 0;                //SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 0;    //Communication is byte-wide (8 bits)
    _SMP = 0;                   //Input data sampled at middle of data output time  TODO : à vérifier
    _CKE = 0;                   //Serial output data changes on transition from Idle clock state to active clock state (see bit 6)
    _SSEN = 0;                  //Slave Select Enable bit (Slave mode)
    _CKP = 0;                   //Idle state for clock is a low level; active state is a high level
    _MSTEN = 1;                 //Master mode
    _SPRE = 0b011;              //Secondary prescale 5:1
    _PPRE = 0b10;               //Primary prescale 4:1


    //On configure les interruptions du module SPI, interruption de fin de transmission
    //ConfigIntSPI1(SPI_INT_PRI_3 & SPI_INT_EN);

    //On démarre le module SPI
    _SPIEN = 1;
}



