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
#include "user.h"          /* Function / Parameters                           */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

extern volatile uint16_t i;
extern volatile uint16_t cpt;
extern volatile uint16_t channel;
extern volatile uint16_t channel_current;
extern volatile uint16_t channel_dist;
extern volatile uint16_t Buff_adc_current[2];
extern volatile uint16_t Buff_adc_dist[3];
extern volatile uint16_t Buff_adc_sector[3];

uint16_t time = 0;

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void ConfigureOscillator(void)
{
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBDbits.PLLDIV = 41; // M=43
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE  = 0; // N2=2
    // Fosc = M/(N1.N2)*Fin
}

void InitApp(void)
{
    // activation de la priorité des interruptions
    _NSTDIS = 0;

    //Init des E/S
    _TRISA0 = 0;
    led = 0;

    _TRISC4 = 0;
    _LATC4 = 0;

    //Init debug on UART, TX->RP8, RX->RP9
    InitDebug(8,9);

    //Configuration du Output Compare 1 en mode PWM
    OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE, 234, 0);   //1.5ms
    _TRISC5 = 0;        //RC5 en sortie
    _ODCC5 = 1;         //Open Drain
    UNLOCK_RP;
    _RP21R = 18;        //OC1(18) sur RP21;
    LOCK_RP;

    //Configuration du Timer 2, période 20ms pour l'OC1
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, 3000);
    ConfigIntTimer2(T2_INT_ON & T2_INT_PRIOR_4);

}

void InitADC()
{
   //Configuration du convertisseur Analog to Digital (ADC) du dspic33f
   //Cf page 286 dspic33f Data Sheet

   //AD1CON1 Confugration
   AD1CON1bits.ADON = 0;    //Eteindre A/D converter pour la configuration
   AD1CON1bits.FORM = 0;    //Configure le format de la sortie de l'ADC ( 3=signed float, 2=unsigned float, 1=signed integer, 0=unsigned integer
   AD1CON1bits.SSRC = 7;    //Config de l'échantillonnage : AutoConvert
   AD1CON1bits.SIMSAM = 0;  //Sample CH0
   AD1CON1bits.ASAM = 0;    //Début d'échantillonnage (1=tout de suite  0=dès que AD1CON1bits.SAMP est activé)
   AD1CON1bits.AD12B = 0;   //Choix du type de converter (10 ou 12 bits) 0 = 10 bits , 1 = 12bits

   //AD1CON2 Configuration
   AD1CON2bits.ALTS = 0;     //Always sampling on channel A
   AD1CON2bits.CHPS = 0;    //Select CH0

   //AD1CON3 Configuration
   AD1CON3bits.ADRC = 1;        //Choix du type de clock interne (=1) ou externe (=0)

   //Choix des références de tensions
   AD1CHS0bits.CH0SA = 1;	// Choix du (+) de la mesure pour le channel CH0 (0 = AN0) par défault
   AD1CHS0bits.CH0NA = 0;	// Choix du (-) de la mesure pour le channel CH0 (0 = Masse interne pic)

   //Configuration des pins analogiques
   AD1PCFGL = 0xFFFF;   //Met tous les ports AN en Digital Input
   //AD1PCFGLbits.PCFG0 = 0;
   AD1PCFGLbits.PCFG1 = 0;
   AD1PCFGLbits.PCFG2 = 0;
   AD1PCFGLbits.PCFG3 = 0;
   AD1PCFGLbits.PCFG4 = 0;
   AD1PCFGLbits.PCFG5 = 0;
   /* COM A ENLEVER SUR DSPIC AVEC 8 PINS ANALOGIQUES
   AD1PCFGLbits.PCFG6 = 0;
   AD1PCFGLbits.PCFG7 = 0;
    */

   //Configuration du Timer 5, pour l'ADC
    OpenTimer5(T5_ON & T5_GATE_OFF & T5_PS_1_1 & T5_SOURCE_INT, 1000);  //25µs
    ConfigIntTimer5(T5_INT_ON & T5_INT_PRIOR_5);

   //Configuration des interuption
   IFS0bits.AD1IF = 0;      //Mise à 0 du flag d'interrupt de ADC1
   IEC0bits.AD1IE = 1;      //Enable les interruptions d'ADC1
   IPC3bits.AD1IP = 3;      //Et les prioritées (ici prio = 3)
   AD1CON1bits.SAMP = 0;
   AD1CON1bits.ADON = 1;    // Turn on the A/D converter
}

int Angle(float angle)
{
   return (int)(angle*0.8667+156);
}

/******************************************************************************/
/*                                                                            */
/*                      Interrupt Routines                                    */
/*                                                                            */
/******************************************************************************/

void __attribute__((interrupt,auto_psv)) _T2Interrupt(void)
{
    printf("T2 interrupt\n");
    led = led^1;            // On bascule l'état de la LED
    _LATB7 = _LATB7^1;

    _T2IF = 0;              // On baisse le FLAG
}

void __attribute__((interrupt,auto_psv)) _T5Interrupt(void)
{
    if(channel == 0)    //Les deux capteurs de courants à la suite
    {
        _CH0SA = Current_G;
        _SAMP = 1; //Start sampling
    }
    else if(cpt == TEMPS_DIST-1)    //Un des capteurs de distance
    {
        cpt = 0;
        switch(channel_dist)
        {
            case 0:
                _CH0SA = Dist_1;
            break;
            case 1:
                _CH0SA = Dist_2;
            break;
            case 2:
                _CH0SA = Dist_3;
            break;
        }
        _SAMP = 1; //Start sampling
    }
    else
    {
        cpt++;
    }
    _T5IF = 0;
}

void __attribute__ ((interrupt, auto_psv)) _ADC1Interrupt(void)
 {

    if(channel == 0)
    {
        if(channel_current == 0)
        {
            Buff_adc_current[0] = ADC1BUF0;
            _CH0SA = Current_D;
            _SAMP = 1;
            channel_current = 1;
        }
        else
        {
            Buff_adc_current[1] = ADC1BUF0;
            channel_current = 0;
            channel = 1;
        }
    }
    else
    {
        Buff_adc_dist[channel_dist] = ADC1BUF0;
        if(Buff_adc_dist[channel_dist] > SEUIL_HAUT && Buff_adc_sector[channel_dist] == 0)
        {
            Buff_adc_sector[channel_dist] = 1;
            printf("Capteur %d: Détection OUI\n", channel_dist+1);
        }
        else if(Buff_adc_dist[channel_dist] < SEUIL_BAS && Buff_adc_sector[channel_dist] == 1)
        {
            Buff_adc_sector[channel_dist] = 0;
            printf("Capteur %d: Détection NON\n", channel_dist+1);
        }
        channel_dist = (channel_dist+1)%3;
        channel = 0;
    }

    _AD1IF = 0;        //Clear the interrupt flag
 }

/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs) User */
/* Guide for an up to date list of the available interrupt options.           */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* dsPIC33F Primary Interrupt Vector Names:                                   */
/*                                                                            */
/* _INT0Interrupt      _C1Interrupt                                           */
/* _IC1Interrupt       _DMA3Interrupt                                         */
/* _OC1Interrupt       _IC3Interrupt                                          */
/* _T1Interrupt        _IC4Interrupt                                          */
/* _DMA0Interrupt      _IC5Interrupt                                          */
/* _IC2Interrupt       _IC6Interrupt                                          */
/* _OC2Interrupt       _OC5Interrupt                                          */
/* _T2Interrupt        _OC6Interrupt                                          */
/* _T3Interrupt        _OC7Interrupt                                          */
/* _SPI1ErrInterrupt   _OC8Interrupt                                          */
/* _SPI1Interrupt      _DMA4Interrupt                                         */
/* _U1RXInterrupt      _T6Interrupt                                           */
/* _U1TXInterrupt      _T7Interrupt                                           */
/* _ADC1Interrupt      _SI2C2Interrupt                                        */
/* _DMA1Interrupt      _MI2C2Interrupt                                        */
/* _SI2C1Interrupt     _T8Interrupt                                           */
/* _MI2C1Interrupt     _T9Interrupt                                           */
/* _CNInterrupt        _INT3Interrupt                                         */
/* _INT1Interrupt      _INT4Interrupt                                         */
/* _ADC2Interrupt      _C2RxRdyInterrupt                                      */
/* _DMA2Interrupt      _C2Interrupt                                           */
/* _OC3Interrupt       _DCIErrInterrupt                                       */
/* _OC4Interrupt       _DCIInterrupt                                          */
/* _T4Interrupt        _DMA5Interrupt                                         */
/* _T5Interrupt        _U1ErrInterrupt                                        */
/* _INT2Interrupt      _U2ErrInterrupt                                        */
/* _U2RXInterrupt      _DMA6Interrupt                                         */
/* _U2TXInterrupt      _DMA7Interrupt                                         */
/* _SPI2ErrInterrupt   _C1TxReqInterrupt                                      */
/* _SPI2Interrupt      _C2TxReqInterrupt                                      */
/* _C1RxRdyInterrupt                                                          */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.  There is no Alternate Vector or 'AIVT' for the 33E family.   */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <compiler installation directory>/doc directory for the latest compiler    */
/* release.                                                                   */
/*                                                                            */
/******************************************************************************/