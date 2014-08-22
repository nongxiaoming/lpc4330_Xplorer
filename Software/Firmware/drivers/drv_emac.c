/*
 * File      : drv_emac.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009-2013 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-05-19     Bernard      porting from LPC17xx drivers.
 */

#include <rtthread.h>
#include "lwipopts.h"
#include <netif/ethernetif.h>

#include "drv_emac.h"

#define EMAC_PHY_AUTO       0
#define EMAC_PHY_10MBIT     1
#define EMAC_PHY_100MBIT    2

#define MAX_ADDR_LEN 6

/* Local variables */
static rt_uint8_t TxBufIndex;
static rt_uint8_t RxBufIndex;

/* ENET local DMA Descriptors. */
static RX_Desc Rx_Desc[NUM_RX_BUF];
static TX_Desc Tx_Desc[NUM_TX_BUF];

/* ENET local DMA buffers. */
static rt_uint32_t rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2] SECTION("ETH_RAM");
static rt_uint32_t tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2] SECTION("ETH_RAM");

struct lpc_emac
{
    /* inherit from ethernet device */
    struct eth_device parent;

    rt_uint8_t phy_mode;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];     /* hw address   */
};
static struct lpc_emac lpc_emac_device;
static struct rt_semaphore sem_lock;
static struct rt_event tx_event;

/* Local Function Prototypes */
static void write_PHY(rt_uint32_t PhyReg, rt_uint32_t Value);
static rt_uint16_t read_PHY(rt_uint8_t PhyReg) ;

void ENET_IRQHandler(void)
{
    rt_uint32_t status;

    /* enter interrupt */
    rt_interrupt_enter();

    status = LPC_EMAC->IntStatus;

    if (status & INT_RX_DONE)
    {
        /* Disable EMAC RxDone interrupts. */
        LPC_EMAC->IntEnable = INT_TX_DONE;

        /* a frame has been received */
        eth_device_ready(&(lpc_emac_device.parent));
    }
    else if (status & INT_TX_DONE)
    {
        /* set event */
        rt_event_send(&tx_event, 0x01);
    }

    if (status & INT_RX_OVERRUN)
    {
        rt_kprintf("rx overrun\n");
    }

    if (status & INT_TX_UNDERRUN)
    {
        rt_kprintf("tx underrun\n");
    }

    /* Clear the interrupt. */
    LPC_EMAC->IntClear = status;

    /* leave interrupt */
    rt_interrupt_leave();
}
void ETHERNET_IRQHandler (void) {
  OS_FRAME *frame;
  U32 i, RxLen;
  U32 *sp,*dp;

  i = RxBufIndex;
  do {
    if (Rx_Desc[i].Stat & DMA_RX_ERROR_MASK) {
      goto rel;
    }
    if ((Rx_Desc[i].Stat & DMA_RX_SEG_MASK) != DMA_RX_SEG_MASK) {
      goto rel;
    }
    RxLen = ((Rx_Desc[i].Stat >> 16) & 0x3FFF) - 4;
    if (RxLen > ETH_MTU) {
      /* Packet too big, ignore it and free buffer. */
      goto rel;
    }
    /* Flag 0x80000000 to skip sys_error() call when out of memory. */
    frame = alloc_mem (RxLen | 0x80000000);
    /* if 'alloc_mem()' has failed, ignore this packet. */
    if (frame != NULL) {
      sp = (U32 *)(Rx_Desc[i].Addr & ~3);
      dp = (U32 *)&frame->data[0];
      for (RxLen = (RxLen + 3) >> 2; RxLen; RxLen--) {
        *dp++ = *sp++;
      }
      put_in_queue (frame);
    }
    /* Release this frame from ETH IO buffer. */
rel:Rx_Desc[i].Stat = DMA_RX_OWN;

    if (++i == NUM_RX_BUF) i = 0;
    RxBufIndex = i;  
  }
  while (!(Rx_Desc[i].Stat & DMA_RX_OWN));
  RxBufIndex = i;

  if (LPC_ETHERNET->DMA_STAT & INT_RBUIE) {
    /* Receive buffer unavailable, resume DMA */
    LPC_ETHERNET->DMA_STAT = INT_RBUIE;
    LPC_ETHERNET->DMA_REC_POLL_DEMAND = 0;       
  }
  /* Clear pending interrupt bits */
  LPC_ETHERNET->DMA_STAT = INT_NISE | INT_AISE | INT_RBUIE | INT_RIE;
}

/* phy write */
static void write_PHY(rt_uint32_t PhyReg, rt_uint32_t Value)
{
   rt_uint32_t tout;

  /* Write a data 'Value' to PHY register 'PhyReg'. */
  while (LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB);
  LPC_ETHERNET->MAC_MII_DATA  = Value;
  LPC_ETHERNET->MAC_MII_ADDR  = MMAR_GB               |
                                MMAR_MW               |
                                CSR_CLK_RANGE    << 2 |
                                PhyReg           << 6 |
                                DP83848C_DEF_ADR << 11;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_WR_TOUT; tout++) {
    if ((LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB) == 0) {
      break;
    }
  }
}

/* phy read */
static rt_uint16_t read_PHY(rt_uint8_t PhyReg)
{
    rt_uint32_t tout;

/* Read a PHY register 'PhyReg'. */
  while(LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB);
  LPC_ETHERNET->MAC_MII_ADDR =  MMAR_GB               |
                                CSR_CLK_RANGE    << 2 |
                                PhyReg           << 6 |
                                DP83848C_DEF_ADR << 11;

  /* Wait until operation completed */
  for (tout = 0; tout < MII_RD_TOUT; tout++) {
    if ((LPC_ETHERNET->MAC_MII_ADDR & MMAR_GB) == 0) {
      break;
    }
  }
  return (LPC_ETHERNET->MAC_MII_DATA & MMDR_GD);
}

/* init rx descriptor */
rt_inline void rx_descr_init(void)
{
/* Initialize Receive DMA Descriptor array. */
  rt_uint32_t i, next;

  RxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_RX_BUF; i++) {
    if (++next == NUM_RX_BUF) next = 0;
    Rx_Desc[i].Stat = DMA_RX_OWN;
    Rx_Desc[i].Ctrl = DMA_RX_RCH | ETH_BUF_SIZE;
    Rx_Desc[i].Addr = (rt_uint32_t)&rx_buf[i];
    Rx_Desc[i].Next = (rt_uint32_t)&Rx_Desc[next];
  }
  LPC_ETHERNET->DMA_REC_DES_ADDR = (uint32_t)&Rx_Desc[0];
}

/* init tx descriptor */
rt_inline void tx_descr_init(void)
{
   /* Initialize Transmit DMA Descriptor array. */
  rt_uint32_t i,next;

  TxBufIndex = 0;
  for (i = 0, next = 0; i < NUM_TX_BUF; i++) {
    if (++next == NUM_TX_BUF) next = 0;
    Tx_Desc[i].CtrlStat = DMA_TX_TCH | DMA_TX_LS | DMA_TX_FS;
    Tx_Desc[i].Addr     = (rt_uint32_t)&tx_buf[i];
    Tx_Desc[i].Next     = (rt_uint32_t)&Tx_Desc[next];
  }
  LPC_ETHERNET->DMA_TRANS_DES_ADDR = (rt_uint32_t)&Tx_Desc[0];
}


static rt_err_t lpc_emac_init(rt_device_t dev)
{
  /* Initialize the EMAC ethernet controller. */
  int tout, regv;

  /* Enable GPIO register interface clock */
  LPC_CCU1->CLK_M4_GPIO_CFG     |= 1;
  while (!(LPC_CCU1->CLK_M4_GPIO_STAT   & 1));

  /* Enable ethernet branch clock */
  LPC_CCU1->CLK_M4_ETHERNET_CFG |= 1;
  while (!(LPC_CCU1->CLK_M4_ETHERNET_STAT & 1));

  LPC_RGU->RESET_EXT_STAT19 |= (1 << 2);
  LPC_RGU->RESET_EXT_STAT22 |= (1 << 2);

  LPC_RGU->RESET_EXT_STAT19 &= ~(1 << 2);
  LPC_RGU->RESET_EXT_STAT22 &= ~(1 << 2);

  LPC_CREG->CREG6 = (LPC_CREG->CREG6 & ~0x7) | 4;

  /* Ethernet pins configuration */
  LPC_SCU->SFSP0_0  = (1 << 6) | (1 << 5) | 0x2; /* P0.0  = ENET_RXD1         */
  LPC_SCU->SFSP0_1  = (1 << 6) | (1 << 5) | 0x6; /* P0.1  = ENET_TX_EN        */

  LPC_SCU->SFSP1_15 = (1 << 6) | (1 << 5) | 0x3; /* P1.15 = ENET_RXD0         */
  LPC_SCU->SFSP1_16 = (1 << 6) | (1 << 5) | 0x7; /* P1.16 = ENET_RX_DV        */
  LPC_SCU->SFSP1_17 = (1 << 6) |            0x3; /* P1.17 = ENET_MDIO         */
  LPC_SCU->SFSP1_18 = (1 << 6) | (1 << 5) | 0x3; /* P1.18 = ENET_TXD0         */
  LPC_SCU->SFSP1_19 = (1 << 6) | (1 << 5) | 0x0; /* P1.19 = ENET_TX_CLK       */
  LPC_SCU->SFSP1_20 = (1 << 6) | (1 << 5) | 0x3; /* P1.20 = ENET_TXD1         */

  LPC_SCU->SFSPC_1  = (1 << 6) | (1 << 5) | 0x3; /* PC.1  = ENET_MDC          */

#ifdef _MII_
  LPC_CREG->CREG6 &= ~0x7;
  
  LPC_SCU->SFSP0_1 = (1 << 6) | (1 << 5) | 0x2;  /* P0.1  = ENET_COL          */
  
  LPC_SCU->SFSPC_0 = (1 << 6) | (1 << 5) | 0x0;  /* PC.0  = ENET_RX_CLK       */
  LPC_SCU->SFSPC_2 = (1 << 6) | (1 << 5) | 0x3;  /* PC.2  = ENET_TXD2         */
  LPC_SCU->SFSPC_3 = (1 << 6) | (1 << 5) | 0x3;  /* PC.3  = ENET_TXD3         */
  LPC_SCU->SFSPC_5 = (1 << 6) | (1 << 5) | 0x3;  /* PC.5  = ENET_TX_ER        */
  LPC_SCU->SFSPC_6 = (1 << 6) | (1 << 5) | 0x3;  /* PC.6  = ENET_RXD2         */
  LPC_SCU->SFSPC_7 = (1 << 6) | (1 << 5) | 0x3;  /* PC.7  = ENET_RXD3         */
  LPC_SCU->SFSPC_8 = (1 << 6) | (1 << 5) | 0x3;  /* PC.8  = ENET_RX_DV        */
#endif

  /* Reset Ethernet Controller peripheral */
  LPC_RGU->RESET_CTRL0 = ETHERNET_RST;
  while (!(LPC_RGU->RESET_ACTIVE_STATUS0 & ETHERNET_RST));

  /* Reset MAC Subsystem internal registers */
  LPC_ETHERNET->DMA_BUS_MODE |= DBMR_SWR;
  while (LPC_ETHERNET->DMA_BUS_MODE & DBMR_SWR);

  /* Put the DP83848C in reset mode */
  write_PHY (PHY_REG_BMCR, PHY_BMCR_RESET);

  /* Wait for hardware reset to end. */
  for (tout = 0; tout < TIMEOUT; tout++) {
    regv = read_PHY (PHY_REG_BMCR);
    if (!(regv & PHY_BMCR_RESET)) {
      /* Reset complete */
      break;
    }
  }

    /* Configure the PHY device */
    switch (lpc_emac_device.phy_mode)
    {
    case EMAC_PHY_AUTO:
        /* Use autonegotiation about the link speed. */
        write_PHY(PHY_REG_BMCR, PHY_AUTO_NEG);
        break;
    case EMAC_PHY_10MBIT:
        /* Connect at 10MBit */
        write_PHY(PHY_REG_BMCR, PHY_FULLD_10M);
        break;
    case EMAC_PHY_100MBIT:
        /* Connect at 100MBit */
        write_PHY(PHY_REG_BMCR, PHY_FULLD_100M);
        break;
    }

  /* Check the link status. */
  for (tout = 0; tout < TIMEOUT; tout++) {
    regv = read_PHY (PHY_REG_STS);
    if (regv & LINK_VALID_STS) {
      /* Link is on. */
      break;
    }
  }

  /* Initialize MAC control register */
  LPC_ETHERNET->MAC_CONFIG = MCR_DO;

  /* Configure Full/Half Duplex mode. */
  if (regv & FULL_DUP_STS) {
    /* Full duplex is enabled. */
    LPC_ETHERNET->MAC_CONFIG |= MCR_DM;
  }

  /* Configure 100MBit/10MBit mode. */
  if (~(regv & SPEED_10M_STS)) {
    /* 100MBit mode. */
    LPC_ETHERNET->MAC_CONFIG |= MCR_FES;  
  }

    /* Set the Ethernet MAC Address registers */
  LPC_ETHERNET->MAC_ADDR0_HIGH = ((rt_uint32_t)lpc_emac_device.dev_addr[5] << 8)  
		                             | ((rt_uint32_t)lpc_emac_device.dev_addr[4]);
		
  LPC_ETHERNET->MAC_ADDR0_LOW  = ((rt_uint32_t)lpc_emac_device.dev_addr[3] << 24) 
		                              |((rt_uint32_t)lpc_emac_device.dev_addr[2] << 16) 
		                              |((rt_uint32_t)lpc_emac_device.dev_addr[1] <<  8) 
		                              |((rt_uint32_t)lpc_emac_device.dev_addr[3]);
		
  /* Reset all interrupts */
  LPC_ETHERNET->DMA_STAT = 0x0001FFFF;

  /* Enable Rx interrupts */
  LPC_ETHERNET->DMA_INT_EN =  INT_NISE | INT_AISE | INT_RBUIE | INT_RIE;

  /* Initialize Descriptor Lists    */
  rx_descr_init();
  tx_descr_init();

  /* Configure Frame Filtering */
  LPC_ETHERNET->MAC_FRAME_FILTER = MFFR_PM | MFFR_PAM | MFFR_RA;
  LPC_ETHERNET->MAC_FLOW_CTRL    = MFCR_DZQP;

  /* Start Transmission & Receive processes */
  LPC_ETHERNET->DMA_OP_MODE = DOMR_FTF | DOMR_ST | DOMR_SR;
  
  /* Enable Receiver and Transmitter */  
  LPC_ETHERNET->MAC_CONFIG |= MCR_TE | MCR_RE;

  /* Ethernet Interrupt Disable function. */
  NVIC_DisableIRQ (ETHERNET_IRQn);

    return RT_EOK;
}

static rt_err_t lpc_emac_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t lpc_emac_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t lpc_emac_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t lpc_emac_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t lpc_emac_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, lpc_emac_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* EtherNet Device Interface */
/* transmit packet. */
rt_err_t lpc_emac_tx(rt_device_t dev, struct pbuf *p)
{
    rt_uint32_t Index, IndexNext;
    rt_uint8_t *ptr;

  Index = TxBufIndex;
  /* Wait until previous packet transmitted. */
  while (Tx_Desc[Index].CtrlStat & DMA_TX_OWN)
{
    rt_err_t result;
        rt_uint32_t recved;

        /* there is no block yet, wait a flag */
        result = rt_event_recv(&tx_event, 0x01,
                               RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved);

        RT_ASSERT(result == RT_EOK);
}
    /* lock EMAC device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

   /* copy data to tx buffer */
    ptr = (rt_uint8_t *)(Tx_Desc[Index].Addr & ~3);
    pbuf_copy_partial(p, ptr, p->tot_len, 0);

  Tx_Desc[Index].Size      = p->tot_len;
  Tx_Desc[Index].CtrlStat |= DMA_TX_OWN;
  if (++Index == NUM_TX_BUF) Index = 0;
  TxBufIndex = Index;
  /* Start frame transmission. */
  LPC_ETHERNET->DMA_STAT = DSR_TPSS;
  LPC_ETHERNET->DMA_TRANS_POLL_DEMAND = 0;
 /* unlock EMAC device */
  rt_sem_release(&sem_lock);

    return RT_EOK;
}

/* reception packet. */
struct pbuf *lpc_emac_rx(rt_device_t dev)
{
    struct pbuf *p;
    rt_uint32_t size;
    rt_uint32_t Index;

    /* init p pointer */
    p = RT_NULL;

    /* lock EMAC device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    Index = LPC_EMAC->RxConsumeIndex;
    if (Index != LPC_EMAC->RxProduceIndex)
    {
        size = (RX_STAT_INFO(Index) & 0x7ff) + 1;
        if (size > ETH_FRAG_SIZE) size = ETH_FRAG_SIZE;

        /* allocate buffer */
        p = pbuf_alloc(PBUF_LINK, size, PBUF_RAM);
        if (p != RT_NULL)
        {
            pbuf_take(p, (rt_uint8_t *)RX_BUF(Index), size);
        }

        /* move Index to the next */
        if (++Index > LPC_EMAC->RxDescriptorNumber)
            Index = 0;

        /* set consume index */
        LPC_EMAC->RxConsumeIndex = Index;
    }
    else
    {
        /* Enable RxDone interrupt */
        LPC_EMAC->IntEnable = INT_RX_DONE | INT_TX_DONE;
    }

    /* unlock EMAC device */
    rt_sem_release(&sem_lock);

    return p;
}

int lpc_emac_hw_init(void)
{
    uint32_t result[4];

    rt_event_init(&tx_event, "tx_event", RT_IPC_FLAG_FIFO);
    rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO);

    /* set autonegotiation mode */
    lpc_emac_device.phy_mode = EMAC_PHY_AUTO;

    // OUI 00-60-37 NXP Semiconductors
    lpc_emac_device.dev_addr[0] = 0x00;
    lpc_emac_device.dev_addr[1] = 0x60;
    lpc_emac_device.dev_addr[2] = 0x37;
    /* set mac address: (only for test) */
    ReadDeviceSerialNum(result);
    lpc_emac_device.dev_addr[3] = result[0] ^ result[1];
    lpc_emac_device.dev_addr[4] = result[1] ^ result[2];
    lpc_emac_device.dev_addr[5] = result[2] ^ result[3];

    lpc_emac_device.parent.parent.init      = lpc_emac_init;
    lpc_emac_device.parent.parent.open      = lpc_emac_open;
    lpc_emac_device.parent.parent.close     = lpc_emac_close;
    lpc_emac_device.parent.parent.read      = lpc_emac_read;
    lpc_emac_device.parent.parent.write     = lpc_emac_write;
    lpc_emac_device.parent.parent.control   = lpc_emac_control;
    lpc_emac_device.parent.parent.user_data = RT_NULL;

    lpc_emac_device.parent.eth_rx           = lpc_emac_rx;
    lpc_emac_device.parent.eth_tx           = lpc_emac_tx;

    eth_device_init(&(lpc_emac_device.parent), "e0");
    return 0;
}
INIT_DEVICE_EXPORT(lpc_emac_hw_init);

#ifdef RT_USING_FINSH
#include <finsh.h>
void emac_dump()
{
    rt_kprintf("Command  : %08x\n", LPC_EMAC->Command);
    rt_kprintf("Status   : %08x\n", LPC_EMAC->Status);
    rt_kprintf("RxStatus : %08x\n", LPC_EMAC->RxStatus);
    rt_kprintf("TxStatus : %08x\n", LPC_EMAC->TxStatus);
    rt_kprintf("IntEnable: %08x\n", LPC_EMAC->IntEnable);
    rt_kprintf("IntStatus: %08x\n", LPC_EMAC->IntStatus);
}
FINSH_FUNCTION_EXPORT(emac_dump, dump emac register);
#endif

