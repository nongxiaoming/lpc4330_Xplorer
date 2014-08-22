/*
* File      : drv_uart.c
* This file is part of RT-Thread RTOS
* COPYRIGHT (C) 2009-2013 RT-Thread Develop Team
*
* The license and distribution terms for this file may be
* found in the file LICENSE in this distribution or at
* http://www.rt-thread.org/license/LICENSE
*
* Change Logs:
* Date           Author       Notes
* 2013-06-10     xiaonong      The first version for LPC40xx
*/

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "drv_i2c.h"

#ifdef RT_USING_I2C


static rt_err_t lpc_i2c_start(LPC_I2Cn_Type *I2Cx)
{
    rt_uint32_t i, state;

    I2Cx->CONCLR = I2C_CONCLR_SIC;

    I2Cx->CONSET = I2C_CONSET_STA;

    // Wait for complete
    for (i = I2C_TOUT; i; i--)

        state = LPC_I2C0->STAT;

    if (state == I2C_STAT_ST || state == I2C_STAT_RPTST)
    {

        I2Cx->CONCLR = I2C_CONCLR_STAC;

        return RT_EOK;
    }

    return RT_ETIMEOUT;
}

static rt_err_t lpc_i2c_stop(LPC_I2Cn_Type *I2Cx)
{
    /* Make sure start bit is not active */
    if (I2Cx->CONSET & I2C_CONSET_STA)
    {
        I2Cx->CONCLR = I2C_CONCLR_STAC;
    }

    I2Cx->CONSET = I2C_CONSET_STO;

    I2Cx->CONCLR = I2C_CONCLR_SIC;

    return RT_EOK;
}


static rt_size_t lpc_i2c_recv_bytes(LPC_I2Cn_Type *I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;
    rt_uint32_t state = 0;
    while (len--)
    {
        I2Cx->CONCLR = I2C_CONCLR_SIC;
        if (len == 0)
        {
            I2Cx->CONCLR = I2C_CONCLR_AAC;
        }
        else
        {
            I2Cx->CONSET = I2C_CONSET_AA;
        }
        while (!(I2Cx->CONSET & I2C_CONSET_SI));

        msg->buf[bytes++]  = (uint8_t)(I2Cx->DAT);
        state = I2Cx->STAT ;
        if (len && (I2C_STAT_DRA != state))
        {
            i2c_dbg("i2c recv error on the byte of %d,send ack error!\n", bytes);
            return bytes;
        }
        else if (I2C_STAT_DRNA != state)
        {
            i2c_dbg("i2c recv error on the byte of %d,send nack error!\n", bytes);
            return bytes;
        }
    }

    return bytes;
}


static rt_size_t lpc_i2c_send_bytes(LPC_I2Cn_Type *I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;
    rt_uint32_t stat = 0;
    /* Make sure start bit is not active */
    if (I2Cx->CONSET & I2C_CONSET_STA)
    {
        I2Cx->CONCLR = I2C_CONCLR_STAC;
    }
    while (len--)
    {
        I2Cx->CONCLR = I2C_CONCLR_SIC;
        I2Cx->DAT = msg->buf[bytes++] ;

        while (!(I2Cx->CONSET & I2C_CONSET_SI));

        stat = I2Cx->STAT;
        if (I2C_STAT_DTA != stat)
        {
            i2c_dbg("send data error ,i2c is not ack!\n");
            return bytes;
        }
    }

    return bytes;
}
static void i2c_set_clock(LPC_I2Cn_Type *I2Cx, uint32_t clock)
{
    /* Clock rate 400kHz @ APB1 Clock */
    I2Cx->SCLH = I2C_PERIPH_CLK / clock;
    I2Cx->SCLL = I2C_PERIPH_CLK / clock;
}

static rt_uint32_t i2c_send_addr(LPC_I2Cn_Type *I2Cx, struct rt_i2c_msg *msg)
{
    rt_uint16_t addr;
    rt_uint16_t flags = msg->flags;
    /* Make sure start bit is not active */
    if (I2Cx->CONSET & I2C_CONSET_STA)
    {
        I2Cx->CONCLR = I2C_CONCLR_STAC;
    }
    /* Test on the direction to set/reset the read/write bit */
    addr = msg->addr << 1;
    if (flags & RT_I2C_RD)
    {
        /* Set the address bit0 for read */
        addr |= 1;
    }
    I2Cx->CONCLR = I2C_CONCLR_SIC;
    /* Send the address */
    I2Cx->DAT = addr ;

    while (!(I2Cx->CONSET & I2C_CONSET_SI));

    return (I2Cx->STAT);
}


static rt_size_t lpc_i2c_xfer(struct rt_i2c_bus_device *bus,
                              struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    rt_uint32_t stat = 0;
    struct lpc_i2c_bus *lpc_i2c = (struct lpc_i2c_bus *)bus;
    /*start the i2c bus*/
    stat = lpc_i2c_start(lpc_i2c->I2C);
    if ((I2C_STAT_SLAWA != stat) && (I2C_STAT_SLARA != stat))
    {
        i2c_dbg("start the i2c bus failed,i2c bus stop!\n");
        goto out;
    }
    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                stat = lpc_i2c_start(lpc_i2c->I2C);
                if ((I2C_STAT_ST != stat) && (I2C_STAT_RPTST != stat))
                {
                    i2c_dbg("restart the i2c bus failed,i2c bus stop!\n");
                    goto out;
                }
            }
            stat = i2c_send_addr(lpc_i2c->I2C, msg);
            if (I2C_STAT_SLAWA  != stat && I2C_STAT_SLARA != stat)
            {
                i2c_dbg("send i2c address but no ack,i2c stop!");
                goto out;
            }
        }
        if (msg->flags & RT_I2C_RD)
        {
            ret = lpc_i2c_recv_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = lpc_i2c_send_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    lpc_i2c_stop(lpc_i2c->I2C);

    return ret;
}


static const struct rt_i2c_bus_device_ops i2c_ops =
{
    lpc_i2c_xfer,
    RT_NULL,
    RT_NULL
};



/** \brief init and register lpc spi bus.
*
* \param SPI: lpc SPI, e.g: LPC_SSP0,LPC_SSP1,LPC_SSP2.
* \param lpc_spi: lpc spi bus struct.
* \param spi_bus_name: spi bus name, e.g: "spi1"
* \return
*
*/
rt_err_t lpc_i2c_register(LPC_I2Cn_Type *I2Cx,
                          struct lpc_i2c_bus *lpc_i2c,
                          const char *spi_bus_name)
{
    if (I2Cx == LPC_I2C0)
    {
        lpc_i2c->I2C = LPC_I2C0;
        /* Connect base clock */
        LPC_CGU->BASE_APB1_CLK = (1    << 11) |
                                 (0x09 << 24) ; /* PLL1 is APB1 clock source        */

        /* Enable I2C0 peripheral clock */
        LPC_CCU1->CLK_APB1_I2C0_CFG = CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
        while (!(LPC_CCU1->CLK_APB1_I2C0_STAT & CCU_CLK_STAT_RUN));

    }
    else if (I2Cx == LPC_I2C1)
    {
        lpc_i2c->I2C = LPC_I2C1;
        /* Connect base clock */
        LPC_CGU->BASE_APB3_CLK = (1    << 11) |
                                 (0x09 << 24) ; /* PLL1 is APB3 clock source        */

        /* Enable I2C1 peripheral clock */
        LPC_CCU1->CLK_APB3_I2C1_CFG = CCU_CLK_CFG_AUTO | CCU_CLK_CFG_RUN;
        while (!(LPC_CCU1->CLK_APB3_I2C1_STAT & CCU_CLK_STAT_RUN));
    }
    else
    {
        return RT_ENOSYS;
    }

    return  rt_i2c_bus_device_register(&lpc_i2c->parent, spi_bus_name);
}

int rt_hw_i2c_init(void)
{
    static struct lpc_i2c_bus lpc_i2c0;

    rt_memset((void *)&lpc_i2c0, 0, sizeof(struct lpc_i2c_bus));
    lpc_i2c0.parent.ops = &i2c_ops;
    lpc_i2c_register(LPC_I2C0, &lpc_i2c0, "i2c0");
    i2c_set_clock(LPC_I2C0, 400000);
    /* Configure I2C Pins */
    LPC_SCU->SFSI2C0 = SFSI2C0_SDA_EZI | SFSI2C0_SCL_EZI;
    /* Set I2C Operation */
    LPC_I2C0->CONCLR = I2C_CONCLR_I2ENC | I2C_CONCLR_STAC | I2C_CONCLR_SIC | I2C_CONCLR_AAC;
    LPC_I2C0->CONSET = I2C_CONSET_I2EN;


    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_i2c_init);

#endif
