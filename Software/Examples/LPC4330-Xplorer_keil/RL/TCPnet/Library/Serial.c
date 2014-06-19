/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low Level Serial Routines
 * Note(s): possible defines select the used communication interface:
 *            __DBG_ITM   - ITM SWO interface
 *                        - disabled  (default)
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2012 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <LPC43xx.H>
#include "Serial.h"

#ifdef __DBG_ITM
volatile int ITM_RxBuffer = ITM_RXBUFFER_EMPTY;  /*  CMSIS Debug Input        */
#endif


/*----------------------------------------------------------------------------
  SER_Init:  Initialize Serial Interface
 *----------------------------------------------------------------------------*/
void SER_Init (void) {

 ;                                               /* not used   for __DBG_ITM  */
}


/*----------------------------------------------------------------------------
  SER_PutChar:  Write a character to the Serial Port
 *----------------------------------------------------------------------------*/
int32_t SER_PutChar (int32_t ch) {

#if   defined __DBG_ITM
  int i;
  ITM_SendChar (ch & 0xFF);
  for (i = 5000; i; i--) __NOP();
#endif

  return (ch);
}


/*----------------------------------------------------------------------------
  SER_GetChar:  Read a character from the Serial Port
 *----------------------------------------------------------------------------*/
int SER_GetChar (void) {

#if   defined __DBG_ITM
  if (ITM_CheckChar())
    return ITM_ReceiveChar();
#endif

  return (-1);
}
