/*--------------------------------------------------------*/
// ETML Ecole Technique
// GesFifoTh32.c
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

#include "GesFifoTh32.h"

/*---------------*/
/* InitFifo      */
/*===============*/

// Init avec possibilité de fournir une valeur de remplissage
// Initialisation du descripteur de FIFO

void InitFifo ( S_fifo *pDescrFifo, int32_t FifoSize, int8_t *pDebFifo, int8_t InitVal )
{
   int32_t i;
   int8_t *pFif;
   pDescrFifo->fifoSize =   FifoSize;
   pDescrFifo->pDebFifo =   pDebFifo; // début du fifo
   // fin du fifo
   pDescrFifo->pFinFifo =   pDebFifo + (FifoSize - 1);
   pDescrFifo->pWrite   =   pDebFifo;  // début du fifo
   pDescrFifo->pRead     =   pDebFifo;  // début du fifo
   pFif = pDebFifo;
   for (i=0; i < FifoSize; i++) {
      *pFif  = InitVal;
      pFif++;
   }
} /* InitFifo */


/*---------------*/
/* GetWriteSpace */
/*===============*/

// Retourne la place disponible en écriture

int32_t GetWriteSpace ( S_fifo *pDescrFifo)
{
   int32_t writeSize;

   // Détermine le nb de car.que l'on peut déposer
   writeSize = pDescrFifo->pRead - pDescrFifo->pWrite -1;
   if (writeSize < 0) {
      writeSize = writeSize + pDescrFifo->fifoSize;
    }
   return (writeSize);
} /* GetWriteSpace */


/*-------------*/
/* GetReadSize */
/*=============*/

// Retourne le nombre de caractères à lire

int32_t GetReadSize ( S_fifo *pDescrFifo)
{
   int32_t readSize;

   readSize = pDescrFifo->pWrite - pDescrFifo->pRead;
   if (readSize < 0) {
       readSize = readSize +  pDescrFifo->fifoSize;
   }

   return (readSize);
} /* GetReadSize */

/*---------------*/
/* PutCharInFifo */
/*===============*/

// Dépose un caractère dans le FIFO
// Retourne 0 si OK, 1 si FIFO full

uint8_t PutCharInFifo ( S_fifo *pDescrFifo, int8_t charToPut )
{
   uint8_t writeStatus;

   // test si fifo est FULL
   if (GetWriteSpace(pDescrFifo) == 0) {
      writeStatus = 1; // fifo FULL
   }
   else {
      // écrit le caractère dans le FIFO
      *(pDescrFifo->pWrite) = charToPut;

      // incrément le pointeur d'écriture
      pDescrFifo->pWrite++;
      // gestion du rebouclement
      if (pDescrFifo->pWrite > pDescrFifo->pFinFifo) {
          pDescrFifo->pWrite = pDescrFifo->pDebFifo;
      }

      writeStatus = 0; // OK
   }
   return (writeStatus);
} // PutCharInFifo 


/*-----------------*/
/* GetCharFromFifo */
/*=================*/

// Obtient (lecture) un caractère du fifo 
// retourne 0 si OK, 1 si empty
// le caractère lu est retourné par réference

uint8_t GetCharFromFifo ( S_fifo *pDescrFifo, int8_t *carLu )
{
   int32_t readSize;
   uint8_t readStatus;

   // détermine le nb de car. que l'on peut lire
   readSize = GetReadSize(pDescrFifo);

   // test si fifo est vide
   if (readSize == 0) {
      readStatus = 1; // fifo EMPTY
      *carLu = 0;     // carLu = NULL
   }
   else {
      // lis le caractère dans le FIFO
      *carLu = *(pDescrFifo->pRead);

      // incrément du pointeur de lecture
      pDescrFifo->pRead++;
      // gestion du rebouclement
      if (pDescrFifo->pRead > pDescrFifo->pFinFifo) 
      {
          pDescrFifo->pRead = pDescrFifo->pDebFifo;
      }
      readStatus = 0; // OK
   }
   return (readStatus);
} // GetCharFromFifo 


