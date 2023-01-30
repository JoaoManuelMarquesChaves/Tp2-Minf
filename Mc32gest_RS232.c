// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'�mission et de r�ception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoy� r�ponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

// Structure d�crivant le message
typedef struct {
    int8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    int8_t MsbCrc;
    int8_t LsbCrc;
} StruMess;


// Struct pour �mission des messages
StruMess TxMess;
// Struct pour r�ception des messages
StruMess RxMess;

// Declaration des FIFO pour r�ception et �mission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de r�ception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'�mission
S_fifo descrFifoTX;


// Initialisation de la communication s�rielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de r�ception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'�mission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit �mission par l'autre
   
} // InitComm

int GetMessage(S_pwmSettings *pData)
{
    static uint8_t GetSituation =  0 ;
    static uint16_t ValCrc;       // en cas d'abandon en cours
    uint8_t  EndMess;
    uint8_t  NbCharToRead = 0;
    static int MessReady;     // indique validit� message re�u
    static uint8_t IcycleRx;
    // D�termine le nombre de caract�res � lire
    NbCharToRead = GetReadSize( &descrFifoRX);
    // Si >= taille message alors traite
    if(NbCharToRead >= MESS_SIZE)  
    {
        EndMess = 0;
        while( (NbCharToRead >= 1) && ( EndMess == 0) ) 
        {
            // Traitement selon �tat du message
            switch(GetSituation)  
            {
                case 0: // control et recuperation start
                    GetCharFromFifo( &descrFifoRX, &RxMess.Start );
                    NbCharToRead--;
                    if( RxMess.Start == STX_code) 
                    {
                        ValCrc = 0xFFFF;
                        ValCrc = updateCRC16(ValCrc, RxMess.Start);
                        GetSituation = 1;
                    } 
                    break;
                case 1: // recuperation speed
                    GetCharFromFifo( &descrFifoRX, &RxMess.Speed );
                    NbCharToRead--;
                    ValCrc = updateCRC16(ValCrc, RxMess.Speed);
                    GetSituation = 2;
                    break;
                case 2: // recuperation Angle
                    GetCharFromFifo( &descrFifoRX, &RxMess.Angle );
                    NbCharToRead--;
                    ValCrc = updateCRC16(ValCrc, RxMess.Angle);
                    GetSituation = 3;
                    break;
                case 3: // recuperation Crc Msb
                    GetCharFromFifo( &descrFifoRX, &RxMess.MsbCrc );
                    NbCharToRead--;
                    ValCrc = updateCRC16(ValCrc, RxMess.MsbCrc);
                    GetSituation = 4;
                    break;
                case 4: // recuperation Crc Lsb
                    GetCharFromFifo( &descrFifoRX, &RxMess.LsbCrc );
                    NbCharToRead--;
                    ValCrc = updateCRC16(ValCrc, RxMess.LsbCrc);
                    GetSituation = 5;
                    break;                   
                case 5: // control Crc
                    if(ValCrc == 0 ) 
                    {
                        // ok -> envoie data
                        pData->SpeedSetting = RxMess.Speed;
                        pData->AngleSetting = RxMess.Angle;
                        MessReady = 1;
                    }
                    else 
                    {
                            BSP_LEDToggle(BSP_LED_6); // test Crc nop
                    }
                    GetSituation = 0;
                    EndMess = 1;   // pour ne traiter qu'un seul message � la fois
                    break;
            } // end switch
        } // end while
    }
    else
    {   //remet un 0 seulment apres 10 mesage non lu
        if(IcycleRx >=10)
        {
            MessReady=0;
        }
        IcycleRx ++;
    }
    // Gestion control de flux de la r�ception
    // 12 correspond � 2 paquets de 6 (3/4 RxBuffer de 8)
    if(GetWriteSpace ( &descrFifoRX) >= 12) 
    {
        // Autorise �mission par l'autre
        RS232_RTS = 0;
    }
    return MessReady;
} // End GetMessage

void SendMessage(S_pwmSettings *pData)
{
    int32_t freeSize;
    uint16_t ValCrc16 = 0XFFFF; 
    U_manip16 tmpCrc;
    // Test si place Pour �crire 1 message
    freeSize = GetWriteSpace ( &descrFifoTX); 
    // Traitement �mission � introduire ICI
    if(freeSize >= (MESS_SIZE))
    {       
        //compose le message
        TxMess.Start = 0XAA; // start
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Start );
        // Traitement data
        TxMess.Speed = pData->SpeedSetting ;
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Speed );
        // Traitement data
        TxMess.Angle = pData->AngleSetting;
        ValCrc16 = updateCRC16(ValCrc16, TxMess.Angle );
        // Traitement CRC
        tmpCrc.val = ValCrc16;
        TxMess.MsbCrc = tmpCrc.shl.msb;
        TxMess.LsbCrc = tmpCrc.shl.lsb;
        // D�pose le message dans le fifo
        PutCharInFifo (&descrFifoTX, TxMess.Start);
        PutCharInFifo (&descrFifoTX, TxMess.Speed);
        PutCharInFifo (&descrFifoTX, TxMess.Angle);
        PutCharInFifo (&descrFifoTX, TxMess.MsbCrc);
        PutCharInFifo (&descrFifoTX, TxMess.LsbCrc);
    }
    
    // Gestion du controle de flux
    // si on a un caract�re � envoyer et que CTS = 0
    freeSize = GetReadSize(&descrFifoTX);
    if ((RS232_CTS == 0) && (freeSize >0))
    {
        // Autorise int �mission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}

// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la r�ponse g�n�r�e dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;    
    int8_t i_cts;
    uint8_t TXsize,freeSize;
    BOOL TxBuffFull;
    int8_t c;

    // Marque d�but interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur � la r�ception.
    }
   
    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {

        // Oui Test si erreur parit� ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            
            // Lecture des caract�res depuis le buffer HW -> fifo SW
			// pour savoir s'il y a une data dans le buffer HW RX 
            while (PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                // Traitement RX � faire ICI
                // Lecture des caract�res depuis le buffer HW -> fifo SW
                c = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                PutCharInFifo ( &descrFifoRX, c);
            }
            
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        
        // Traitement controle de flux reception � faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        freeSize = GetWriteSpace ( &descrFifoRX);
            if (freeSize <= 6) // a cause d'un int pour 6 char
            {
            // Demande de ne plus �mettre
                RS232_RTS = 1;              
            }

        
    } // end if RX

    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) {

        

            
        // Avant d'�mettre, on v�rifie 3 conditions :
        //  Si CTS = 0 autorisation d'�mettre (entr�e RS232_CTS)
        //  S'il y a un carat�res � �mettre dans le fifo
        //  S'il y a de la place dans le buffer d'�mission
       i_cts = RS232_CTS;
       TXsize = GetReadSize (&descrFifoTX);
       TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
       
       if ( (i_cts == 0) && ( TXsize > 0 ) && TxBuffFull==false )
       {
           do{
                // Traitement TX � faire ICI
                // Envoi des caract�res depuis le fifo SW -> buffer HW
                GetCharFromFifo(&descrFifoTX, &c);
                PLIB_USART_TransmitterByteSend(USART_ID_1, c);
                LED5_W = !LED5_R; // Toggle Led5
                i_cts = RS232_CTS;
                TXsize = GetReadSize (&descrFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
            } while ( (i_cts == 0) && ( TXsize > 0 ) && TxBuffFull==false );
                
		
        // disable TX interrupt (pour �viter une interrupt. inutile si plus rien � transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
       }
        // Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
 }