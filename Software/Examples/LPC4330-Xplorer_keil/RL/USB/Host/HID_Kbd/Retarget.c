/*-----------------------------------------------------------------------------
 *      RL-ARM
 *-----------------------------------------------------------------------------
 *      Name:    Retarget.c
 *      Purpose: Retarget input to the USB keyboard and 
 *               output to Serial IO
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <stdio.h>
#include <string.h>
#include <rt_sys.h>
#include <rl_usb.h>


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;

#ifdef STDIO
extern int  SER_PutChar   (int ch);

/*-----------------------------------------------------------------------------
  Write character to the Serial Port
 *----------------------------------------------------------------------------*/
int sendchar (int ch) {

  if (ch == '\n')  {
    SER_PutChar ('\r');
  }
  SER_PutChar (ch);

  return (ch);
}


/*-----------------------------------------------------------------------------
  Read character from the USB keyboard
 *----------------------------------------------------------------------------*/
int getkey (void) {

  return (usbh_hid_kbd_getkey (1, 0));
}
#endif

/*--------------------------- fputc ------------------------------------------*/

int fputc (int ch, FILE *f) {
#ifdef STDIO
  return (sendchar (ch));
#endif
}

/*--------------------------- fgetc ------------------------------------------*/

int fgetc (FILE *f) {
#ifdef STDIO
  return (getkey ());
#endif
}

/*--------------------------- ferror -----------------------------------------*/

int ferror (FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

/*--------------------------- _ttywrch ---------------------------------------*/

void _ttywrch (int ch) {
#ifdef STDIO
  sendchar (ch);
#endif
}

/*--------------------------- _sys_exit --------------------------------------*/

void _sys_exit (int return_code) {
  /* Endless loop. */
  while (1);
}

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
