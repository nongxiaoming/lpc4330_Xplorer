/*-----------------------------------------------------------------------------
 *      RL-ARM
 *-----------------------------------------------------------------------------
 *      Name:    Terminal.h
 *      Purpose: File manipulation example terminal definitions
 *-----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

enum {BACKSPACE = 0x08, 
      LF        = 0x0A, 
      CR        = 0x0D, 
      CNTLQ     = 0x11, 
      CNTLS     = 0x13, 
      ESC       = 0x1B, 
      DEL       = 0x7F };

/* External functions */
extern int getline  (char *buf, int bufsz);
extern int sendchar (int ch);
extern int getkey   (void);

/*-----------------------------------------------------------------------------
 * end of file
 *----------------------------------------------------------------------------*/
