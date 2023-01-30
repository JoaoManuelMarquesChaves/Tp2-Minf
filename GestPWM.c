/*--------------------------------------------------------*/
// GestPWM.c
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



#include "GestPWM.h"
#include "Mc32DriverAdc.h"
#include "Mc32DriverLcd.h"
#include "app.h"
#include "peripheral/oc/plib_oc.h"
#include "GesFifoTh32.h"

//S_pwmSettings PWMData;      // pour les settings
//S_ADCResults ReadAdc;

S_pwmSettings PwmData = {.cntAdc=0};
S_pwmSettings PwmDataToSend;
        
void GPWM_Initialize(S_pwmSettings *pData)
{
   // Init les données
    pData-> cntAdc = 0 ;
    
    // Init état du pont en H
    // initialiser le Hbrige
    BSP_EnableHbrige ();
    
    // lance les timers et OC
    /* initialisation des timers */
    // DRV_TMR0_Start();  
    DRV_TMR0_Start ();
    // DRV_TMR1_Start();  
    DRV_TMR1_Start ();           
    // DRV_TMR2_Start();  
    DRV_TMR2_Start ();            

    /* Initialisation des OC */
    // init OC0 activer OC2
    DRV_OC0_Start ();
    // init OC1 activer OC3
    DRV_OC1_Start ();
    
}

// Obtention vitesse et angle (mise a jour des 4 champs de la structure)
void GPWM_GetSettings(S_pwmSettings *pData)	
{
    /* Gestion de la lecture d'ADC avec filtrage */
    GPWM_ReadAdcFiltered(pData);
    
    /* Calcul de la consigne de vitesse */
    pData->absSpeed = pData->adcResFilt_Can0 * SPEED_RATIO;
    pData->SpeedSetting = pData->absSpeed - 99;
    pData->absSpeed = abs(pData->SpeedSetting);
    
    
    /* Calcul consigne de l'angle */
    pData->absAngle = pData->adcResFilt_Can1 * (180 / ADC_RES);
    pData->AngleSetting = pData->absAngle - 90;
    pData->absAngle = abs(pData->AngleSetting);
    
    
}


// Affichage des information en exploitant la structure
void GPWM_DispSettings(S_pwmSettings *pData, int Remote)
{   
    lcd_gotoxy(1,1);
    if (Remote == 1)
    {
        printf_lcd("** Remote settings");
    }
    else
    {
        printf_lcd("Local settings");
    }
           
    lcd_gotoxy(1,2);
    /* Affichage vitesse */
    printf_lcd("SpeedSetting = %3d",(int)pData->SpeedSetting);
    /* Affichage vitesse absolue */
    lcd_gotoxy(1,3);
    printf_lcd("absSpeed = %2d",(int)pData->absSpeed);
    /* Afficahge de l'angle */
    lcd_gotoxy(1,4);
    printf_lcd("Angle = %3d",(int)pData->AngleSetting);                     
}

// Execution PWM et gestion moteur à partir des info dans structure
void GPWM_ExecPWM(S_pwmSettings *pData)
{
  uint16_t t2_Width = 0;
    uint16_t t3_Width = 0;
    float ratioAngle = 0;
 
    //tourner le moteur de gauche à droite.............................
    if(pData->SpeedSetting < 0)
    {
        AIN1_HBRIDGE_W = 1; //AIN1 High
        AIN2_HBRIDGE_W = 0; //AIN2 LOW
        STBY_HBRIDGE_W = 1; // STBY High
    }
    //tourner le moteur de droite à gauche............................
    if (pData->SpeedSetting > 0)
    {
        AIN1_HBRIDGE_W = 0; //AIN1 LOW
        AIN2_HBRIDGE_W = 1; //AIN2 High
        STBY_HBRIDGE_W = 1; // STBY High
    }
    if (pData->SpeedSetting == 0)
    {
        //moteur ne tourne plus, il passe en stanby
        STBY_HBRIDGE_W = 0; // STBY LOW       
    }
    
    /* Obtention de la periode du timer */
    t2_Width = DRV_TMR1_PeriodValueGet();  
    /* Calcul du rapport de la pulse avec la vitesse */
    t2_Width = t2_Width * (float)(pData->absSpeed/99.0);
    /* MAJ de la pulse du PWM */
    PLIB_OC_PulseWidth16BitSet(_OCMP2_BASE_ADDRESS, t2_Width);
    
    /* Calcul du ratio de l'angle */
    ratioAngle = (float)((pData->AngleSetting+90)/180.0);
    /* Calcul de la largeur de la pulse en prenant directement en compte la frequence du timer */
    t3_Width = (DRV_TMR2_CounterFrequencyGet()/(1/(SERVO_MAX-SERVO_MIN)))*ratioAngle;
    /* Ajouter le minimum pour le servo moteur */
    t3_Width += DRV_TMR2_CounterFrequencyGet()/(1/(SERVO_MIN));    
    //génération d'une impulsion dont la largeur est proportionnelle à l'angle
    PLIB_OC_PulseWidth16BitSet(_OCMP3_BASE_ADDRESS, t3_Width);
    
 }
    
   
  
    



void GPWM_ReadAdcFiltered(S_pwmSettings *pData)
{
    /* Variable comptage */
    uint8_t i = 0; 
    
    //lecture nouvelles valeurs ch0 et ch1
    pData->AdcResBuff[pData->cntAdc] = BSP_ReadAllADC();
    
    //incrémentation compteur
    if(pData->cntAdc < FILTER_SIZE-1)
    {
        /* Incremente le timer de filtrage */
        pData->cntAdc = pData->cntAdc +1;
    }
    else
    {
        /* Relance le filtrage */
        pData->cntAdc = 0;
    }
    
    /* Somme du buffer pour le filtrage */
    pData->adcResFilt_Can0 = 0;
    pData->adcResFilt_Can1 = 0;
    for(i = 0; i<FILTER_SIZE; i++)
    {
        /* Somme des deux cannaux */
        pData->adcResFilt_Can0 += pData->AdcResBuff[i].Chan0;
        pData->adcResFilt_Can1 += pData->AdcResBuff[i].Chan1;
    }  
    /* Moyenne de la somme du filtre */
    pData->adcResFilt_Can0 /= (float)FILTER_SIZE;
    pData->adcResFilt_Can1 /= (float)FILTER_SIZE;
}