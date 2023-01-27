#ifndef GestPWM_H
#define GestPWM_H
/*--------------------------------------------------------*/
// GestPWM.h
/*--------------------------------------------------------*/
//	Description :	Gestion des PWM 
//			        pour TP1 2016-2017
//
//	Auteur 		: 	C. HUBER
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include <stdint.h>
#include "Mc32DriverAdc.h"
#include "app.h"


/*--------------------------------------------------------*/
// Définition des fonctions prototypes
/*--------------------------------------------------------*/
#define FILTER_SIZE 10                          // Taille du filtre coulissant
#define ADC_RES 1023.0                          // Resolution de l'ADC
#define ABS_MAX_SPEED 198.0                     // Vitesse abs. maximum (99*2)
#define SPEED_RATIO (ABS_MAX_SPEED / ADC_RES)   // Ratio vitesse/ADC

#define SERVO_MAX 0.0024                        // Temps max du servo [s]
#define SERVO_MIN 0.0006                        // Temps min du servo [s]



typedef struct {
    
    /* Variables consignes */
    float absSpeed;    // vitesse 0 à 99
    float absAngle;    // Angle  0 à 180
    float SpeedSetting; // consigne vitesse -99 à +99
    float AngleSetting; // consigne angle  -90 à +90
    
    /* Variables ADC pour PWM (Association) */
    S_ADCResults AdcResBuff[FILTER_SIZE]; // Données ADC
    float adcResFilt_Can0;
    float adcResFilt_Can1;
    uint8_t cntAdc;  
    
} S_pwmSettings;

extern S_pwmSettings PwmData;
extern S_pwmSettings PwmDataToSend;


/*typedef struct {
    
    // Variables consignes 

    float SpeedSetting; // consigne vitesse -99 à +99
    float AngleSetting; // consigne angle  -90 à +90
    
    
} S_pwmSetToSend;*/
//extern S_pwmSetToSend PwmDataToSend;



void GPWM_Initialize(S_pwmSettings *pData);

// Ces 3 fonctions ont pour paramètre un pointeur sur la structure S_pwmSettings.
void GPWM_ReadAdcFiltered(S_pwmSettings *pData);
void GPWM_GetSettings(S_pwmSettings *pData);	// Obtention vitesse et angle
void GPWM_DispSettings(S_pwmSettings *pData, int Remote);	// Affichage
void GPWM_ExecPWM(S_pwmSettings *pData);		// Execution PWM et gestion moteur.
void GPWM_ExecPWMSoft(S_pwmSettings *pData);		// Execution PWM software.

#endif
