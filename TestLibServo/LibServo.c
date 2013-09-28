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

#include <p33Fxxxx.h>      /* Includes device header file                     */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <stdio.h>
#include <stdlib.h>
#include "timer.h"
#include "user.h"          /* Function / Parameters                           */
#include "debug.h"
#include "LibServo.h"
#include <libpic30.h>
#include <time.h>


/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

struct tableauServo tServo;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void InitLibServo()
{
    //initialize Timer1
    OpenTimer1(T1_OFF & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_256 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, PERIOD_TIMER);
    ConfigIntTimer1(T1_INT_PRIOR_3 & T1_INT_ON);

    //Preparing structure
    tServo.index=0;
    tServo.modif=0;
    tServo.taille=2;
    tServo.tab[0].tics=0;
    tServo.tab[0].port='x';
    tServo.tab[0].pin=-1;
    tServo.tab[1].tics=PeriodToTic(0.02);
    tServo.tab[1].port='x';
    tServo.tab[1].pin=-1;

    //Start Timer 1
    _TON=1;
}

uint16_t PeriodToTic(double period)
{
    return (uint16_t) (156250.00*period);
}

void ajouterServo(char port, int8_t pin)
{
    Servo temp = {port, pin, 0};
    tServo.tab[tServo.taille+tServo.modif] = temp;
    /*tServo.tab[tServo.taille+tServo.modif].port=port;
    tServo.tab[tServo.taille+tServo.modif].pin=pin;
    tServo.tab[tServo.taille+tServo.modif].tics=0;*/
    tServo.modif++;
}

void modifierServoAngle(char port, int8_t pin, int16_t angle)
{
    
}

void modifierServoPeriod(char port, int8_t pin, double period)
{
    Servo temp = {port, pin, PeriodToTic(period)};
    tServo.tab[tServo.taille+tServo.modif] = temp;
    tServo.modif++;
}

void trierTableau()
{
    bool isFound = false;
    uint8_t ancienneTaille = tServo.taille;
    Servo temp;
    uint8_t i,j;
    
    //
    for(i=0;i<tServo.modif;i++)
    {
        isFound = false;
        temp = tServo.tab[ancienneTaille+i];
        for(j=0; j<tServo.taille; j++)
        {
            if(isFound == false && tServo.tab[j].port == temp.port && tServo.tab[j].pin == temp.pin)
            {
                isFound = true;
                tServo.taille--;
                tServo.tab[j] = tServo.tab[j+1];
            }
            else if (isFound == true)
            {
                tServo.tab[j] = tServo.tab[j+1];
            }
        }
        for(j=tServo.taille-1; j>=0; j--)
        {
            tServo.tab[j+1] = tServo.tab[j];
            if(temp.tics >= tServo.tab[j-1].tics && temp.tics < tServo.tab[j].tics)
            {
                tServo.tab[j] = temp;
                tServo.taille++;
                break;
            }
        }
    }
    tServo.modif = 0;
}

void setPin(char port, int8_t pin, uint8_t value)
{
    if(pin != -1 && port != 'x')
    {
        switch(port)
        {
            case 'a':
            case 'A':
                LATA = SET_BIT(LATA, pin, value);
                TRISA = SET_BIT(LATA, pin, value);
                break;
            case 'b':
            case 'B':
                LATB = SET_BIT(LATB, pin, value);
                TRISB = SET_BIT(LATB, pin, value);
                break;
#if defined(__dsPIC33FJ64MC804__)
            case 'c':
            case 'C':
                LATC = SET_BIT(LATC, pin, value);
                TRISC = SET_BIT(LATB, pin, value);
                break;
#endif
        }
    }
}

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
{
    _TON=0;     //Desactive Timer 1
    uint8_t i;
    uint32_t nextInt;

    //At the end of 20ms
    if(tServo.index == tServo.taille-1)
    {
        tServo.index = 0;
        for(i=1; i<tServo.taille-1; i++)
            setPin(tServo.tab[i].port, tServo.tab[i].pin, 1);
        if(tServo.modif>0)
            trierTableau();
    }

    //Compute Next Interrupt
    do
    {
        nextInt = tServo.tab[tServo.index+1].tics - tServo.tab[tServo.index].tics;
        //Turn off the corresponding pin
        setPin(tServo.tab[tServo.index].port, tServo.tab[tServo.index].pin, 0);

        tServo.index++;
    }while(nextInt == 0);
    WriteTimer1(PERIOD_TIMER - nextInt);

    led1 = led1^1;    // On bascule l'état de la LED

    _TON=1;     //Active Timer 1
    _T1IF = 0;      // On baisse le FLAG
}