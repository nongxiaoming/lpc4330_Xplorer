/*
 * File      : stm32f20x_40x_spi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-10    xiaonong       The first version for LPC40xx
 */

#ifndef __DRV_SDIO_H
#define __DRV_SDIO_H

#include <stdint.h>
#include <rtthread.h>
#include <drivers/spi.h>

#include "LPC43xx.h"




int rt_hw_sdio_init(void);

#endif // __DRV_SDIO_H
