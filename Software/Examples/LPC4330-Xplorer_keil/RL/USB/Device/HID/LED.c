/*-----------------------------------------------------------------------------
 * Name:    LED.c
 * Purpose: Low level LED functions
 * Note(s):
 *-----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <LPC43xx.h>
#include "LED.h"

#define NUM_LEDS  2                                     /* Number of LEDs     */

const uint32_t led_mask[] = { 1UL << 11,                /* GPIO1.11           */
                              1UL << 12 };              /* GPIO1.12           */

/* Clock Control Unit register bits */
#define CCU_CLK_CFG_RUN   (1 << 0)
#define CCU_CLK_CFG_AUTO  (1 << 1)
#define CCU_CLK_STAT_RUN  (1 << 0)

/*----------------------------------------------------------------------------
  LED_Init:  Initialize LEDs
 *----------------------------------------------------------------------------*/
void LED_Init (void) {

  /* Enable clock and init GPIO outputs */
  LPC_CCU1->CLK_M4_GPIO_CFG  = CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_GPIO_STAT & CCU_CLK_STAT_RUN));

  LPC_SCU->SFSP2_11  =  0;                              /* GPIO1[11]          */
  LPC_SCU->SFSP2_12  =  0;                              /* GPIO1[12]          */

  LPC_GPIO_PORT->DIR[1] |= (led_mask[0] | led_mask[1]);
  LPC_GPIO_PORT->SET[1]  = (led_mask[0] | led_mask[1]); /* switch LEDs off    */
}


/*----------------------------------------------------------------------------
  LED_UnInit:  Uninitialize LEDs; pins are set to reset state
 *----------------------------------------------------------------------------*/
void LED_UnInit(void) {

  LPC_GPIO_PORT->DIR[1] &= ~(led_mask[0] | led_mask[1]);
}


/*----------------------------------------------------------------------------
  LED_On: Turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (uint32_t num) {

  if (num < NUM_LEDS) {
    LPC_GPIO_PORT->CLR[1] = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  LED_Off: Turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (uint32_t num) {

  if (num < NUM_LEDS) {
    LPC_GPIO_PORT->SET[1] = led_mask[num];
  }
}

/*----------------------------------------------------------------------------
  LED_Val: Write value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Val (uint32_t val) {
  int i;

  for (i = 0; i < NUM_LEDS; i++) {
    if (val & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}


/*----------------------------------------------------------------------------
  LED_Num: Return number of available LEDs
 *----------------------------------------------------------------------------*/
uint32_t LED_Num (void) {
  return (NUM_LEDS);
}
