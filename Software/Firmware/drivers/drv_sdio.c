/*
 * File      : drv_sdio.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009-2014 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-06-10     xiaonong      The first version for LPC43xx
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#ifdef RT_USING_SDIO
#include "drv_sdio.h"



#define SDIOCLK 180000000
#define CPUCLK  180000000


static rt_bool_t sdio_bus_mode (rt_uint8_t mode);
static rt_bool_t sdio_bus_width (rt_uint32_t width);
static rt_bool_t sdio_bus_speed (rt_uint32_t kbaud);
static rt_bool_t sdio_command (rt_uint8_t cmd, rt_uint32_t arg, rt_uint32_t resp_type, rt_uint32_t *rp);
static rt_bool_t sdio_read_block (rt_uint32_t bl, rt_uint8_t *buf, rt_uint32_t cnt);
static rt_bool_t sdio_write_block (rt_uint32_t bl, rt_uint8_t *buf, rt_uint32_t cnt);
static rt_bool_t sdio_set_dma (U32 mode, U8 *buf, U32 cnt);
static rt_bool_t  sdio_check_media (void);         /* Optional function for SD card check */



/* Wait time in for loop cycles */
#define DMA_TOUT  10000000

static __align(4) SDIO_DMA_DESC DMADesc[4];

/* Clock Control Unit register bits */
#define CCU_CLK_CFG_RUN   (1 << 0)
#define CCU_CLK_CFG_AUTO  (1 << 1)
#define CCU_CLK_STAT_RUN  (1 << 0)


static rt_bool_t sdio_bus_mode (rt_uint8_t mode)
{
  /* Set SDIO Bus mode to Open Drain or Push Pull. */

  switch (mode) {
    case BUS_OPEN_DRAIN:
    case BUS_PUSH_PULL:
      /* Not configurable. */
      return RT_TRUE;

    default:
      return RT_FALSE;
  }
}



static rt_bool_t sdio_bus_width (rt_uint32_t width)
{
  /* Set SDIO Bus width. */
  switch (width) {
    case 1:
      LPC_SDMMC->CTYPE &= ~1;
      return RT_TRUE;

    case 4:
      LPC_SDMMC->CTYPE |=  1;
      return RT_TRUE;

    default:
      return RT_FALSE;
  }
}


static rt_bool_t sdio_bus_speed (rt_uint32_t kbaud)
{
  /* Set SDIO clock speed to desired value. */
  U32 div;

  /* baud = SDIOCLK / (div + 2) */
  div = (SDIOCLK/1000 + kbaud - 1) / kbaud;
  div >>= 1;
  if (div > 0xFF) div  = 0xFF;
  LPC_SDMMC->CLKENA = 0;                /* Disable clock                      */
  LPC_SDMMC->CLKSRC = 0;                /* Clock source is clock divider 0    */
  LPC_SDMMC->CLKDIV = div;              /* Set clock divider                  */
  LPC_SDMMC->CLKENA = 1;                /* Don't stop clock when card in idle */

  /* Send "update clock registers" command and wait until finished */
  LPC_SDMMC->CMD = SDIO_CMD_CLK_UPD | SDIO_CMD_WAIT_PRV | SDIO_CMD_START;
  while (LPC_SDMMC->CMD & SDIO_CMD_START);
  return RT_TRUE;
}



static rt_bool_t sdio_command (rt_uint8_t cmd, rt_uint32_t arg, rt_uint32_t resp_type, rt_uint32_t *rp)
{
  /* Send a Command to Flash card and get a Response. */
  U32 cmdval, ints;

  /* Clear interrupt status */
  LPC_SDMMC->RINTSTS = SDIO_RINTSTS_MSK;

  /* Set command register value */
  cmdval = (cmd & 0x3F) | SDIO_CMD_RESP_CRC | SDIO_CMD_WAIT_PRV | SDIO_CMD_START;
  switch (resp_type) {
    case RESP_SHORT:
      cmdval |= SDIO_CMD_RESP_EXP;
      break;
    case RESP_LONG:
      cmdval |= SDIO_CMD_RESP_EXP | SDIO_CMD_RESP_LEN;
      break;
  }

  if (cmd == READ_BLOCK  || cmd == READ_MULT_BLOCK ||
      cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK) {
    /* Set data expected and read/write bits */
    cmdval |= SDIO_CMD_DATA_EXP | SDIO_CMD_WAIT_PRV;
    if (cmd == WRITE_BLOCK || cmd == WRITE_MULT_BLOCK) {
      cmdval |= SDIO_CMD_READ_WRITE;
    }
  }

  if ((cmd == SEND_OP_COND) || (cmd == SEND_APP_OP_COND)  || (cmd == STOP_TRANS)) {
    /* Disable response CRC check */
    cmdval &= ~SDIO_CMD_RESP_CRC;
  }

  /* Send the command */
  LPC_SDMMC->CMDARG = arg;
  LPC_SDMMC->CMD    = cmdval;

  if (resp_type == RESP_NONE) {
    /* Wait until command finished */
    while (!(LPC_SDMMC->RINTSTS & SDIO_RINTSTS_CDONE));
    return RT_TRUE;
  }

  for (;;) {
    ints = LPC_SDMMC->RINTSTS;

    if (ints & (SDIO_RINTSTS_RE | SDIO_RINTSTS_RCRC | SDIO_RINTSTS_RTO)) {
      /* Response error, CRC error, response timeout */
      return RT_FALSE;
    }

    if (ints & SDIO_RINTSTS_HLE) {
      /* Hardware locked write */
      LPC_SDMMC->CMD = cmdval;
    }

    if (ints & SDIO_RINTSTS_CDONE) {
      /* Command done */
      if (cmdval & SDIO_CMD_DATA_EXP) {
        if (ints & (1 << 3)) {
          break;
        }
      }
      else break;
    }
  }
  /* Read MCI response registers */
  rp[0] = LPC_SDMMC->RESP0;
  if (resp_type == RESP_LONG) {
    rp[0] = LPC_SDMMC->RESP3;
    rp[1] = LPC_SDMMC->RESP2;
    rp[2] = LPC_SDMMC->RESP1;
    rp[3] = LPC_SDMMC->RESP0;
  }
  return RT_TRUE;
}



static rt_bool_t sdio_read_block (uint32_t bl, uint8_t *buf, uint32_t cnt)
{
  /* Read one or more 512 byte blocks from Flash Card. */
  U32 i, idsts;

  /* Wait until DMA completes the operation */
  for (i = DMA_TOUT; i; i--) {
    idsts = LPC_SDMMC->IDSTS;
    if (idsts & SDIO_IDSTS_FBE) {
      /* Fatal Bus Error */
      break;
    }
    if (idsts & SDIO_IDSTS_CES) {
      /* Card error summary */
      break;
    }
    if (idsts & SDIO_IDSTS_RI) {
      /* Data reception finished */
      LPC_SDMMC->IDSTS |= SDIO_IDSTS_MSK;
      return (__TRUE);
    }
  }
  /* Clear Interrupt flags */
  LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
  /* DMA Data Transfer timeout. */
  return (__FALSE);
}


static rt_bool_t sdio_write_block (rt_uint32_t bl, rt_uint8_t *buf, rt_uint32_t cnt)
{
  /* Write a cnt number of 512 byte blocks to Flash Card. */
  U32 i, idsts;

  /* Wait until DMA completes the operation */
  for (i = DMA_TOUT; i; i--) {
    idsts = LPC_SDMMC->IDSTS;
    if (idsts & SDIO_IDSTS_FBE) {
      /* Fatal Bus Error */
      break;
    }
    if (idsts & SDIO_IDSTS_CES) {
      /* Card error summary */
      break;
    }
    if (idsts & SDIO_IDSTS_TI) {
      /* Data transmission finished */
      LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
      return RT_TRUE;
    }
  }
  /* Clear Interrupt flags */
  LPC_SDMMC->IDSTS = SDIO_IDSTS_MSK;
  /* DMA Data Transfer timeout. */
  return RT_FALSE;
}

/*--------------------------- DmaStart ---------------------------------------*/

static BOOL SetDma (U32 mode, U8 *buf, U32 cnt) {
  /* Configure DMA Descriptor for read or write */
  U32 i, n;

  LPC_SDMMC->BYTCNT = cnt * 512;
  
  for (i = 0; cnt; i++) {
    n = (cnt > 8) ? (8) : (cnt);

    DMADesc[i].BufSize  = n * 512;
    DMADesc[i].BufAddr1 = (U32)buf;
    DMADesc[i].BufAddr2 = (U32)buf + n * 512;

    buf += n * 512;
    cnt -= n;

    n = (cnt > 8) ? (8) : (cnt);
    if (n) {
      DMADesc[i].BufSize |= (n * 512) << 13;

      buf += n * 512;
      cnt -= n;
    }
    DMADesc[i].CtrlStat = DESC_OWN;
  }
  DMADesc[i-1].CtrlStat |= DESC_LD | DESC_ER;
  DMADesc[0].CtrlStat |= DESC_FS;

  return RT_TRUE;
}


/*--------------------------- CheckMedia -------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;

  if (!(LPC_SDMMC->CDETECT & 1)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  }


  return (stat);
}

int rt_hw_sdio_init(void)
{
     /* Initialize SD/MMC interface. */
  
  /* Connect SDIO base clock to PLL1                                          */
  LPC_CGU->BASE_SDIO_CLK  = (0x01 << 11) | (0x09 << 24);

  /* Enable GPIO register interface clock                                     */
  LPC_CCU1->CLK_M4_GPIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_GPIO_STAT & CCU_CLK_STAT_RUN));

  /* Enable SDIO clocks                                                       */
  LPC_CCU1->CLK_M4_SDIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU1->CLK_M4_SDIO_CFG & CCU_CLK_STAT_RUN));

  LPC_CCU2->CLK_SDIO_CFG |= CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
  while (!(LPC_CCU2->CLK_SDIO_CFG & CCU_CLK_STAT_RUN));

  /* Configure SDIO port pins */
  LPC_SCU->SFSPC_0  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.0  = SD_CLK            */
  LPC_SCU->SFSPC_4  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.4  = SD_DAT0           */
  LPC_SCU->SFSPC_5  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.5  = SD_DAT1           */
  LPC_SCU->SFSPC_6  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.6  = SD_DAT2           */
  LPC_SCU->SFSPC_7  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.7  = SD_DAT3           */
  LPC_SCU->SFSPC_8  = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.8  = SD_CD             */
  LPC_SCU->SFSPC_10 = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | 0x7; /* PC.10 = SD_CMD            */

  /* Reset SDMMC interface and wait until reset complete */
  LPC_SDMMC->CTRL |= SDIO_CTRL_DMA_RST | SDIO_CTRL_FIFO_RST |SDIO_CTRL_CTRL_RST;
  while (LPC_SDMMC->CTRL & (SDIO_CTRL_DMA_RST | SDIO_CTRL_FIFO_RST |SDIO_CTRL_CTRL_RST));

  /* Power enable */
  LPC_SDMMC->PWREN |= SDIO_PWREN_EN;

  /* Set FIFO Threshold watermark */
  LPC_SDMMC->FIFOTH = (1 << 28) | (15 << 16) | 16;

  /* Set Bus Mode */
  LPC_SDMMC->BMOD = (1 << 8) | SDIO_BMOD_DE;

  /* Set descriptor address */
  LPC_SDMMC->DBADDR = (U32)&DMADesc;

  /* Clear Interrupt Flags */
  LPC_SDMMC->RINTSTS  = SDIO_RINTSTS_MSK;
  LPC_SDMMC->IDSTS   |= SDIO_IDSTS_MSK;

  /* Enable DMA interrupts */
  LPC_SDMMC->IDINTEN |= 0x37;

  /* Enable internal DMA */
  LPC_SDMMC->CTRL |= SDIO_CTRL_USE_IDMA | SDIO_CTRL_DMA_EN;


    return 0;
}

#endif
