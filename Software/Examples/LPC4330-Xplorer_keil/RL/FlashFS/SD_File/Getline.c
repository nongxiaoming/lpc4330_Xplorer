/*------------------------------------------------------------------------------
 *      RL-ARM
 *------------------------------------------------------------------------------
 *      Name:    Getline.c
 *      Purpose: Line Edited Character Input
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2012 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <stdio.h>
#include "Terminal.h"


/*------------------------------------------------------------------------------
 *      Line Editor
 *----------------------------------------------------------------------------*/

int getline (char *line, int n)  {
  static int  cnt  = 0;
         int  cnt1;
         int  key;
         char c;

  line += cnt;
  key = getkey ();                          /* Read key from Serial port      */
  if (key >= 0) {
    c = (char)(key);
    if (c == CR)  c = LF;                   /* CR = LF                        */
    if (c == BACKSPACE  ||  c == DEL)  {    /* process backspace              */
      if (cnt != 0)  {
        cnt--;                              /* decrement count                */
        line--;                             /* and line pointer               */
        sendchar (BACKSPACE);               /* echo backspace                 */
        sendchar (' ');
        sendchar (BACKSPACE);
      }
    }
    else if (c == ESC) {                    /* process escape                 */
      *line++ = c;                          /* store character and increment  */
      cnt++;                                /* and count                      */
    }
    else if (c != CNTLQ &&                  /* ignore: Control Q = XON        */
             c != CNTLS ) {                 /*         Control S = XOFF       */
      sendchar (*line = c);                 /* echo and store character       */
      line++;                               /* increment line pointer         */
      cnt++;                                /* and count                      */
    }
    if ((cnt >= n - 1) || (c == LF) || (c == ESC)) {
      *line = 0;                            /* mark end of string             */
      cnt1  = cnt;
      cnt   = 0;
      return (cnt1);
    }
  }

  return (0);
}

/*------------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
