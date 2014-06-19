/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    Measure.c
 *      Purpose: Measure Recorder
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <stdio.h>                       /* standard I/O .h-file              */
#include <ctype.h>                       /* character functions               */
#include <LPC43xx.h>
#include "Terminal.h"
#include "Serial.h"
#include "KBD.h"
#include "LED.h"

#include "Measure.h"                     /* global project definition file    */

const char menu[] =
   "\n"
   "+***** REMOTE MEASUREMENT RECORDER *******+\n"
   "| This program is a simple Measurement    |\n"
   "| Recorder. It is based on the LPC4330    |\n"
   "| and records the state of PushButton SW2 |\n"
   "+ command ----+ function -----------------+\n"
   "| R [n]       | read <n> records          |\n"
   "| D           | display measurement       |\n"
   "| T hh:mm:ss  | set time                  |\n"
   "| I mm:ss.ttt | set interval time         |\n"
   "| C           | clear records             |\n"
   "| Q           | quit recording            |\n"
   "| S           | start recording           |\n"
   "+-------------+---------------------------+\n";

struct interval setinterval;                /* interval setting values        */
struct interval interval;                   /* interval counter               */

volatile int measurement_interval = 0;      /* measurement interval over      */
volatile int mdisplay = 0;                  /* measurement display requested  */
volatile int startflag = 0;                 /* start measurement recording    */

struct mrec current;                        /* current measurements           */

#define SCNT 20                             /* number of records              */

struct mrec save_record[SCNT];              /* buffer for measurements        */
int sindex;                                 /* save index                     */
int savefirst;                              /* save first index               */

char ERR [] = "\n*** ERROR: %s\n";          /* ERROR message string in code   */

#define WRONGINDEX 0xffff                   /* error signal for wrong index   */


/******************************************************************************/
/*               Save current measurements in save_record                     */
/******************************************************************************/
static void save_current_measurements (void) {
  save_record[sindex++] = current;             /* copy current measurements   */
  if (sindex == SCNT) sindex = 0;              /* check bounds of sindex      */
  if (sindex == savefirst)  {                  /* check circular buffer limits*/
    if (++savefirst == SCNT)  savefirst = 0;   /* check bounds of savefirst   */
  }
}


/******************************************************************************/
/*                Timer interrupt service function                            */
/*                executes each 1ms                                           */
/******************************************************************************/
void SysTick_Handler (void) {

  if (measurement_interval)  {                 /* measurement done ?          */
    save_current_measurements ();              /* yes -> save measurements    */
    measurement_interval = 0;                  /* Save measurements done      */
  }
                                               /* check if interval is over   */
  if (interval.min  == 0 &&
      interval.sec  == 0 &&
      interval.msec == 0     )  {
    interval = setinterval;                    /* set interval time again     */
    measurement_interval = startflag;          /* determine measurement flag  */
  }
  else  {                                      /* update interval time        */
    if (interval.msec-- == 0)  {               /* calculate millisecond       */
      interval.msec = 999;
      if (interval.sec-- == 0)  {              /* calculate second            */
        interval.sec = 59;
        interval.min--;                        /* calculate minute            */
      }
    }
  }
                                               /* update current time         */
  if (++current.time.msec == 1000)  {          /* update millisecond cnt      */
    current.time.msec = 0;

    if (++current.time.sec == 60)  {           /* update second counter       */
      current.time.sec = 0;

      if (++current.time.min == 60)  {         /* update minute counter       */
        current.time.min = 0;

        if (++current.time.hour == 24)  {      /* update hour counter         */
          current.time.hour = 0;
        }
      }
    }
  } /* end of if( ++current.time.msec... */

  if (current.time.msec == 0) {
    LED_On(1);                                 /* Turn LED#1 On               */
  }
  if (current.time.msec == 100) {
    LED_Off(1);                                /* Turn LED#1 Off              */
  }

  if (measurement_interval || mdisplay)  {     /* process measurement         */
      current.gpio = KBD_GetKeys();            /* Push Button SW2             */
    mdisplay = 0;                              /* mdisplay = 0 for ready sig. */
  }
}


/******************************************************************************/
/*                       Calculate first Read Index                           */
/******************************************************************************/
static int read_index (char *buffer) {
  int index = 0;
  int args;

  if (setinterval.min == 0     &&              /* check if setinterval is     */
      setinterval.sec == 0     &&              /* below 1 second and          */
      setinterval.msec < 999   &&              /* measurements are collected  */
      startflag                   )  {
    printf (ERR, "QUIT MEASUREMENTS BEFORE READ");
    return (WRONGINDEX);                       /* no display on the fly if    */
  }                                            /* interval time < 1 second    */
  args = sscanf (buffer, "%d", &index);        /* scan input for read count   */
  if (args == 0  ||  index == 0  || args == EOF)  index = SCNT;
  index = sindex - index;                      /* calculate first read index  */
  if (index < 0) index += SCNT;                /* from read count             */
  return (index);
}


/******************************************************************************/
/*                         Clear Measurement Records                          */
/******************************************************************************/
static void clear_records (void) {
  int idx;                                     /* index for loop              */

  startflag = 0;                               /* stop measurement collecting */
  sindex = savefirst = 0;                      /* reset circular buffer index */
  for (idx = 0; idx != SCNT; idx++)  {         /* mark all records unused     */
    save_record[idx].time.hour = 0xff;         /* unused flag: hour = 0xff    */
  }
}


/******************************************************************************/
/***************************      MAIN PROGRAM      ***************************/
/******************************************************************************/
int main (void)  {                             /* main entry for program      */
  char cmdbuf [15];                            /* command input buffer        */
  char cmd    =  0;                            /* active command              */
  char con    =  0;                            /* Connection status of kbd    */
  char con_ex =  0x80;                         /* Previous connection status  */
  char out    =  1;                            /* Output to keyboard LEDs     */
  int  i;                                      /* index for command buffer    */
  int  idx = 0;                                /* index for circular buffer   */

  SystemCoreClockUpdate();                     /* Update System Core Clock    */
  SysTick_Config(SystemCoreClock/1000);        /* Generate interrupt each 1 ms*/
  LED_Init();                                  /* Initialize LEDs             */
  KBD_Init();                                  /* Initialize Push Button      */
  SER_Init ();                                 /* initialize serial interface */
  usbh_init_all();                             /* Initialize all USB Hosts    */

  clear_records ();                            /* initialize circular buffer  */
  printf ( menu );                             /* display command menu        */
  for (i = 0; i < 1000000; i++) {               /* allow kbd connect detect    */
    usbh_engine_all();
                                               /* if connected, break         */
    if ((usbh_hid_status(1, 0) << 1) | usbh_hid_status(0, 0))
      break;
  }

  while (1)  {                                 /* loop forever                */
    usbh_engine_all();
    con = (usbh_hid_status(1, 0) << 1) |       /* If kbd connection status    */
           usbh_hid_status(0, 0);
    if (con ^ con_ex) {
      if (con & (con ^ con_ex) & 2) {
        usbh_hid_data_out (1, 0, (U8 *)&out,1);/* Turn on NUM LED             */
        printf ("\nCommand: ");                /* display prompt              */
      } else if (con & (con ^ con_ex) & 1) {
        usbh_hid_data_out (0, 0, (U8 *)&out,1);/* Turn on NUM LED             */
        printf ("\nCommand: ");                /* display prompt              */
      } else {
        printf ("\nConnect the keyboard ...\n");
      }
      con_ex = con;
    }

    if (cmd) {                                 /* if command active           */
COMMAND:
      if (getline (&cmdbuf[0], sizeof (cmdbuf))) {      /* input command line */
        if (cmdbuf[0] == 0x1B) {               /* if ESCAPE key abort command */
          printf ("\n");
          printf ("\nCommand: ");
          cmd = 0;
        }
      }
      if (cmd == 'R') {                        /* cmd: Read circular Buffer   */
        if (idx != sindex)  {                  /* check end of table          */
          if (save_record[idx].time.hour != 0xff)  {
            printf ("\n");
            measure_display (save_record[idx]);/* display record              */
          }
          if (++idx == SCNT) idx = 0;          /* next circular buffer entry  */
        } else {
          printf ("\n");
          printf ("\nCommand: ");
          cmd = 0;
        }
      } else if (cmd == 'D') {                 /* cmd: Display Command        */
        mdisplay = 1;                          /* request measurement         */
        while (mdisplay);                      /* wait for measurement        */
        measure_display (current);             /* display values              */
      }
    } else if (getline (&cmdbuf[0], sizeof (cmdbuf))) { /* input command line */

      for (i = 0; cmdbuf[i] != 0; i++)  {      /* convert to upper characters */
        cmdbuf[i] = toupper(cmdbuf[i]);
      }

      for (i = 0; cmdbuf[i] == ' '; i++);      /* skip blanks                 */

      switch (cmdbuf[i])  {                    /* proceed to command function */

        case 'R':                              /* Read circular Buffer        */
          if ((idx = read_index (&cmdbuf[i+1])) == WRONGINDEX)  break;
          if (save_record[idx].time.hour != 0xff)  {
            printf ("\n");
            measure_display (save_record[idx]);/* display 1st record          */
            if (++idx == SCNT) idx = 0;        /* next circular buffer entry  */
          }
          cmd = 'R';
          goto COMMAND;

        case 'T':                              /* Enter Current Time          */
          set_time (&cmdbuf[i+1]);
          break;

        case 'I':                              /* Enter Interval Time         */
          set_interval (&cmdbuf[i+1]);
          break;

        case 'D':                              /* Display Command             */
          printf ("\nDisplay Measurements: (ESC aborts)\n");
          cmd = 'D';
          goto COMMAND;

        case 'S':                              /* Start Command               */
          printf ("\nStart Measurement Recording\n");
          startflag = 1;
          break;

        case 'Q':                              /* Quit Command                */
          printf ("\nQuit Measurement Recording\n");
          startflag = 0;
          break;

        case 'C':                              /* Clear Command               */
          printf ("\nClear Measurement Records\n");
          clear_records ();
          break;

        default:                               /* Error Handling              */
          printf (ERR, "UNKNOWN COMMAND");
          printf (menu);                       /* display command menu        */
          break;
      }
      printf ("\nCommand: ");
    }
  }
}
