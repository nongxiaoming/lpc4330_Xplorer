/*-----------------------------------------------------------------------------
 * Name:    KBD.c
 * Purpose: Low level keyboard functions
 * Note(s):
 *-----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2012 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <LPC43xx.h>                    /* LPC43xx Definitions                */
#include "KBD.h"

#define NUM_KEYS  1                     /* Number of available keys           */

/* Keys for NGX LPC4330 Xplorer                                               */
#define P2_7       0x01


/* Clock Control Unit register bits */
#define CCU_CLK_CFG_RUN   (1 << 0)
#define CCU_CLK_CFG_AUTO  (1 << 1)
#define CCU_CLK_STAT_RUN  (1 << 0)

/*----------------------------------------------------------------------------
  KBD_Init:  Initialize keyboard/buttons
 *----------------------------------------------------------------------------*/
void KBD_Init (void) {

  /* Enable clock and init GPIO inputs */
  LPC_CCU1->CLK_M4_GPIO_CFG  = CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_GPIO_STAT & CCU_CLK_STAT_RUN)); 

  LPC_SCU->SFSP2_7  = (1 << 6) | 0;

  LPC_GPIO_PORT->DIR[0]  &= ~(1 << 7);
}


/*----------------------------------------------------------------------------
  KBD_UnInit:  Uninitialize keyboard/buttons
 *----------------------------------------------------------------------------*/
void KBD_UnInit (void) {
  LPC_SCU->SFSP2_7 = 0;
}


/*----------------------------------------------------------------------------
  KBD_GetKeys:  Get keyboard state
 *----------------------------------------------------------------------------*/
uint32_t KBD_GetKeys (void) {
  /* Read board keyboard inputs */
  uint32_t val = 0;

  if ((LPC_GPIO_PORT->PIN[0] & (1 << 7)) == 0) {
    /* P2_7 button */
    val |= P2_7;
  }
  return (val);
}


/*----------------------------------------------------------------------------
  KBD_Num:  Get number of available keys
 *----------------------------------------------------------------------------*/
uint32_t KBD_Num (void) {
  return (NUM_KEYS);
}
