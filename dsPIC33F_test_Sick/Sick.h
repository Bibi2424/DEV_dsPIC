/*
* Sick
* Compiler : Microchip xC16
* µC : 33FJ64MC804
* Mai 2014
*    ____________      _           _
*   |___  /| ___ \    | |         | |
*      / / | |_/ /___ | |__   ___ | |_
*     / /  |    // _ \| '_ \ / _ \| __|
*    / /   | |\ \ (_) | |_) | (_) | |_
*   /_/    |_| \_\___/|____/ \___/'\__|
*			      7robot.fr
*/

#ifndef SICK_H
#define	SICK_H

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

#define NUMBER_OF_SICK 2

#define Dist_1  1
#define Dist_2  2

#define DEFAULT_THRESHOLD 80
#define MARGIN_SICK 50

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/

void InitADC();
void ChangeThreshold(int id, uint16_t NewThreshold);

#endif	/* SICK_H */

