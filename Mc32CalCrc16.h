#ifndef MC32CALCCRC16_H
#define MC32CALCCRC16_H

/*--------------------------------------------------------*/
//	Mc32CalcCrc16.h
/*--------------------------------------------------------*/

// Fonction pour calcul d'un CRC16-CCITT
// Adaptation PIC du programme de Michael Neumann, 14.06.1998
// Migration sur PIC32MX 26.03.2014 C. Huber
// ATTENTION : Correction de la formule 06.02.2015

#include <stdint.h>

// Important : selon spec. CCITT il faut initialiser la valeur du
// Crc16 à 0xFFFF


// Fonction pour calcul du CRC16 byte à byte
// -----------------------------------------

// Il faut fournir la valeur précédante du CRC ainsi qu'un 
// des caractère du message
// La fonction retourne la nouvelle valeur du CRC16

uint16_t updateCRC16(uint16_t crc, uint8_t data);

#endif