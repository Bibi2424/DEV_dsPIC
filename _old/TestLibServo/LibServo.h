/*
* Project   : Test Lib Servo
* File      : LibServo.h
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
#define TAILLE_MAX 10       // -> 8servos
#define PERIOD_TIMER 3125   // -> 20ms

//si value=1 on met à un le bit dans le port (OR), si value=0 on le met à 0 (AND)
#define SET_BIT(port, bit, value) value ? port | (1<<bit) : port & (~(1<<bit))

typedef struct Servo Servo;
struct Servo
{
    char port;      //A, B ou C
    int8_t pin;     // 0 -> 15
    uint16_t tics;  //Period en tics
};

typedef struct tableauServo tableauServo;
struct tableauServo
{
    int8_t taille;  //Taille du tableau (sans les modifs)
    int8_t modif;   //Nombre de modif
    int8_t index;   //index courant
    Servo tab[TAILLE_MAX];  //tableau de servo
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

