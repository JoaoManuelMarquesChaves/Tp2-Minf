/*--------------------------------------------------------*/
// ETML Ecole Technique
// GesFifoTh32.h
/*--------------------------------------------------------*/
//	Description :
//	 Gestion d'un fifo de caractère, utilisation de pointeur et
//   d'un descripteur de fifo
//
//	Auteur 		: 	C. Huber
//	Version		:	V1.6
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
//  Modifications :
//   CHR 19.12.2014  remplacement typedef32 par stdint
//   CHR 20.12.2016  fifosize en int32_t pour permettre des
//                    fifo de grande taille
//   SCA 06.09.2022  v1.7 MPLABX 5.45/xc32 2.50/Harmony 2.06
//                   Enlevé bug dans GetCharFromFifo qui
//                   empêchait un buffer > 256 éléments
//
/*--------------------------------------------------------*/

#ifndef GesFifoTh32_H
#define GesFifoTh32_H

#include <stdint.h>


// structure décrivant un FIFO
typedef struct fifo {
   int32_t fifoSize;   // taille du fifo
   int8_t *pDebFifo;   // pointeur sur début du fifo
   int8_t *pFinFifo;   // pointeur sur fin du fifo
   int8_t *pWrite;      // pointeur d'écriture
   int8_t *pRead;      // pointeur de lecture
} S_fifo;

/*--------------------------------------------------------*/
/* Définition des fonctions de gestion du fifo            */
/*--------------------------------------------------------*/

/*---------------*/
/* InitFifo      */
/*===============*/

// Initialisation du descripteur de FIFO
void InitFifo ( S_fifo *pDescrFifo, int32_t FifoSize, int8_t *pDebFifo, int8_t InitVal );

/*---------------*/
/* GetWriteSpace */
/*===============*/

// Retourne la place disponible en écriture

int32_t GetWriteSpace ( S_fifo *pDescrFifo);

/*-------------*/
/* GetReadSize */
/*=============*/

// Retourne le nombre de caractères à lire

int32_t GetReadSize ( S_fifo *pDescrFifo);

/*---------------*/
/* PutCharInFifo */
/*===============*/

// Dépose un caractère dans le FIFO
// Retourne 0 si OK, 1 si FIFO full

uint8_t PutCharInFifo ( S_fifo *pDescrFifo, int8_t charToPut );

/*-----------------*/
/* GetCharFromFifo */
/*=================*/

// Obtient (lecture) un caractère du fifo
// retourne 0 si OK, 1 si empty
// le caractère lu est retourné par réference

uint8_t GetCharFromFifo ( S_fifo *pDescrFifo, int8_t *carLu );

#endif
