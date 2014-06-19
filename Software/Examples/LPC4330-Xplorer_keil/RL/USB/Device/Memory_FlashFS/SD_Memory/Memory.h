/*-----------------------------------------------------------------------------
 *      RL-ARM - USB
 *-----------------------------------------------------------------------------
 *      Name:    Memory.h
 *      Purpose: USB Memory Storage Demo Definitions
 *      Rev.:    V4.70
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

/* LED Mask Definitions */
#define LED_MSK_OFF         0x00000000  /* LEDs OFF */
#define LED_MSK_ALL         0x0000000F  /* LED0..3  */
#define LED_MSK_RD          0x00000001  /* LED0 */
#define LED_MSK_WR          0x00000002  /* LED1 */
#define LED_MSK_CFG         0x00000004  /* LED2 */
#define LED_MSK_SUSP        0x00000008  /* LED3 */

/* LED Number Definitions */
#define LED_NUM_RD          0           /* LED0 */
#define LED_NUM_WR          1           /* LED1 */
#define LED_NUM_CFG         2           /* LED2 */
#define LED_NUM_SUSP        3           /* LED3 */

/* Turn LEDs ON or OFF */
extern void LED_On  (unsigned int led_num);
extern void LED_Off (unsigned int led_num);

/* Output value to LEDs */
extern void LED_Val (unsigned int led_val);

extern volatile BOOL WakeUp;
