/*
 * File      : drv_spi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2014 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-10    xiaonong       The first version for LPC43xx
 */

#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#include <stdint.h>
#include <rtthread.h>
#include <drivers/i2c.h>

#include "LPC43xx.h"

#define I2C_PERIPH_CLK  180000000
#define I2C_TOUT           100000       /* Approx. delay for CPU @ 120MHz     */

#define A_WR      0                     /* Master will write to the I2C       */
#define A_RD      1                     /* Master will read from the I2C      */

/* Clock Control Unit register bits */
#define CCU_CLK_CFG_RUN   (1 << 0)
#define CCU_CLK_CFG_AUTO  (1 << 1)
#define CCU_CLK_STAT_RUN  (1 << 0)

/* LPC43xx Register Bits Definitions */
#define SFSI2C0_SCL_EFP   (1 <<  0)
#define SFSI2C0_SCL_EHD   (1 <<  2)
#define SFSI2C0_SCL_EZI   (1 <<  3)
#define SFSI2C0_SCL_ZIF   (1 <<  7)
#define SFSI2C0_SDA_EFP   (1 <<  8)
#define SFSI2C0_SDA_EHD   (1 << 10)
#define SFSI2C0_SDA_EZI   (1 << 11)
#define SFSI2C0_SDA_ZIF   (1 << 15)

#define I2C_CONCLR_AAC    (1 <<  2)     /* CONCLR:Assert Acknowledge Clear    */
#define I2C_CONCLR_SIC    (1 <<  3)     /* CONCLR:I2C Interrupt Clear Bit     */
#define I2C_CONCLR_STAC   (1 <<  5)     /* CONCLR:Start Flag Clear            */
#define I2C_CONCLR_I2ENC  (1 <<  6)     /* CONCLR:I2C Interface Disable       */

#define I2C_CONSET_AA     (1 <<  2)     /* CONSET:Assert acknowledge flag     */
#define I2C_CONSET_SI     (1 <<  3)     /* CONSET:I2C interrupt flag          */
#define I2C_CONSET_STO    (1 <<  4)     /* CONSET:STOP flag                   */
#define I2C_CONSET_STA    (1 <<  5)     /* CONSET:START flag                  */
#define I2C_CONSET_I2EN   (1 <<  6)     /* CONSET:I2C interface enable        */

#define I2C_STAT_ST       0x08          /* STAT:START has been transmitted    */
#define I2C_STAT_RPTST    0x10          /* STAT:Repeated START transmitted    */
#define I2C_STAT_SLAWA    0x18          /* STAT:SLA+W transmitted and ACKed   */
#define I2C_STAT_SLAWNA   0x20          /* STAT:SLA+W transmitted, no ACK     */
#define I2C_STAT_DTA      0x28          /* Data transmitted, ACK received     */
#define I2C_STAT_DTNA     0x30          /* Data transmitted, no ACK           */
#define I2C_STAT_ALOST    0x38          /* Arbitration lost                   */

#define I2C_STAT_SLARA    0x40          /* STAT:SLA+R transmitted and ACKed   */
#define I2C_STAT_SLARNA   0x48          /* STAT:SLA+R transmitted, no ACK     */
#define I2C_STAT_DRA      0x50          /* Data received, ack returned        */
#define I2C_STAT_DRNA     0x58          /* Data received, not ack returned    */

struct lpc_i2c_bus
{
    struct rt_i2c_bus_device parent;
    LPC_I2Cn_Type *I2C;
};



int rt_hw_i2c_init(void);

#endif // __DRV_SPI_H
