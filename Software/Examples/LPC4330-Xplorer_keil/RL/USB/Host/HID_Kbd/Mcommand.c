/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    Mcommand.c
 *      Purpose: Time Set Commands for the Remote Measurement Recorder
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>                       /* standard ANSI I/O .h-file         */
#include <LPC43xx.h>

#include "Measure.h"                     /* global project definition file    */


/******************************************************************************/
/*                           Display measurements                             */
/******************************************************************************/
void measure_display (struct mrec display)  {

  printf ("\rTime: %2d:%02d:%02d.%03d SW2:%02X",
           display.time.hour,            /* display hour                      */
           display.time.min,             /* display minute                    */
           display.time.sec,             /* display second                    */
           display.time.msec,            /* display millisecond               */
           display.gpio);                /* display gpio port G value         */

}


/******************************************************************************/
/*                           Set Current Time                                 */
/******************************************************************************/
void set_time (char * buffer)  {
  int hour, min, sec;                    /* temporary time values             */
  int args;                              /* number of arguments               */

  args = sscanf (buffer, "%d:%d:%d",     /* scan input line for               */
                 &hour,                  /* hour, minute and second           */
                 &min,
                 &sec);
  if (hour > 23  ||  min > 59  ||        /* check for valid inputs            */
      sec > 59   ||  args < 2  ||  args == EOF)  {
    printf (ERR, "INVALID TIME FORMAT");
  }
  else  {                                /* if inputs valid then              */
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk |   /* Disable SysTick        */
                       SysTick_CTRL_ENABLE_Msk);
    SysTick->VAL   = 0;                             /* Zero the SysTick Value */
    current.time.hour = hour;            /* setting the new time: hours       */
    current.time.min  = min;             /* setting the new time: minutes     */
    current.time.sec  = sec;             /* setting the new time: seconds     */
    current.time.msec = 0;               /* setting the new time: miliseconds */
    interval.min = 0;                    /* force new interval                */
    interval.sec = 0;
    interval.msec = 0;
    SysTick->CTRL |=  (SysTick_CTRL_TICKINT_Msk |   /* Enable SysTick         */
                       SysTick_CTRL_ENABLE_Msk);
  }
}


/******************************************************************************/
/*                            Set Interval Time                               */
/******************************************************************************/
void set_interval (char * buffer) {
  struct interval itime = {0,0,0};       /* temporary interval record         */
  int min, sec, msec;
  int args;                              /* number of arguments               */

  args = sscanf (buffer, "%d:%d.%d",     /* scan input line for               */
                 &min,                   /* minute, second and milliseconds   */
                 &sec,
                 &msec);
                                         /* check valid inputs                */
  if (sec > 59  ||  msec > 999  ||  args < 2  || args == EOF)  {
    printf (ERR, "INVALID INTERVAL FORMAT");
  }
  else  {                                /* if inputs are valid then          */
    itime.min  = min;
    itime.sec  = sec;
    itime.msec = msec;
    if (itime.min != 0 || itime.sec != 0 || itime.msec != 0)  {
      if (itime.msec-- == 0)  {          /* correct interval time for         */
        itime.msec = 999;                /* interrupt routine                 */
        if (itime.sec-- == 0)  {
          itime.sec = 59;
          itime.min--;
        }
      }
    }
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk |   /* Disable SysTick        */
                       SysTick_CTRL_ENABLE_Msk);
    SysTick->VAL   = 0;                             /* Zero the SysTick Value */

    setinterval = itime;                 /* of the new setinterval time       */
    interval.min = 0;                    /* force new interval                */
    interval.sec = 0;
    interval.msec = 0;
    SysTick->CTRL |=  (SysTick_CTRL_TICKINT_Msk |   /* Enable SysTick         */
                       SysTick_CTRL_ENABLE_Msk);
  }
}
