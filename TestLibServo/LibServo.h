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
#ifndef LIBSERVO_H
#define	LIBSERVO_H
/******************************************************************************/
/* Debug #define Macros and Structures                                        */
/******************************************************************************/

#define PRESCALE 256
#define TAILLE_MAX 10
#define PERIOD_TIMER 3125

#define SET_BIT(port, bit, value) value ? port | (1<<bit) : port & (~(1<<bit))

typedef struct Servo Servo;
struct Servo
{
    char port;
    int8_t pin;
    uint16_t tics;
};

typedef struct tableauServo tableauServo;
struct tableauServo
{
    int8_t taille;
    int8_t modif;
    int8_t index;
    Servo tab[TAILLE_MAX];
};

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void InitLibServo();

void ajouterServo(char port, int8_t pin);

void modifierServoAngle(char port, int8_t pin, int16_t angle);

void modifierServoPeriod(char port, int8_t pin, double period);

void trierTableau();

void setPin(char port, int8_t pin, uint8_t value);

uint16_t setBit(uint16_t port, uint8_t pin, uint8_t value);

uint16_t PeriodToTic(double period);

#endif  /* LIBSERVO_H */

