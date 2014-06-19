/*
 * @brief Spansion S25 driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */
#include <string.h>

#include "spifilib_dev.h"
#include "spifilib_chiphw.h"

/** @defgroup LPCSPIFILIB_CONFIG_SA25FLP LPCSPIFILIB Spansion S25 family device support
 * @ingroup LPCSPIFILIB_DRIVERS
 * This driver includes support for the following devices.<br>
 * S25FL032P<br>
 * S25FL129P<br>
 *
 * @{
 */

/**
 * @}
 */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
/* Generic device family name */
static const char pDevNameString[] = "Spansion S25FL-P FLASH";

/* Declare the base node */
static uint32_t devCount;
static SPIFI_DEV_DATA_T devHead = {0};

/* Required private data size for this driver */
#define PRVDATASIZE 0

/* Command definitions. Only used commands are defined. */
#define S25_CMD_READ_DATA               0x03		/**< Read Data bytes */
#define S25_CMD_FAST_READ               0x0B		/**< Read Data bytes at Fast Speed */
#define S25_CMD_QOR                     0x6B		/**< Quad Output Read */
#define S25_CMD_RDID                    0x9F		/**< Read Identification */
#define S25_CMD_WREN                    0x06		/**< Write Enable */
#define S25_CMD_P4E                     0x20		/**< 4 KB Parameter Sector Erase */
#define S25_CMD_BULK_ERS                0xC7		/**< Bulk Erase */
#define S25_CMD_SE                      0xD8		/**< 64KB Sector Erase */
#define S25_CMD_PP                      0x02		/**< Page Programming */
#define S25_CMD_QPP                     0x32		/**< Quad Page Programming */
#define S25_CMD_RDSR                    0x05		/**< Read Status Register */
#define S25_CMD_WRR                     0x01		/**< Write (Status & Configuration) Register */
#define S25_CMD_RCR                     0x35		/**< Read Configuration Register (CFG) */
#define S25_CMD_CLSR                    0x30		/**< Reset the Erase and Program Fail Flag (SR5 and SR6) and restore normal operation) */

/* Status register definitions */
/* Status Register Write Disable,
   1 = Protects when W# is low,
   0 = No protection, even when W# is low */
#define S25_STAT_SRWD                   (1 << 7)
/* Programming Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define S25_STAT_P_ERR                  (1 << 6)
/* Erase Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define S25_STAT_W_ERR                  (1 << 5)
/* Block protect bits,
   Protects upper half of address range in 5 sizes */
#define S25_STAT_BPMASK                 (7 << 2)
/* Write Enable Latch,
   1 = Device accepts Write Status Register, program, or erase commands,
   0 = Ignores Write Status Register, program, or erase commands */
#define S25_STAT_WEL                    (1 << 1)
/* Write in Progress,
   1 = Device Busy. A Write Status Register, program, or erase,
   0 = Ready. Device is in standby mode and can accept commands. */
#define S25_STAT_WIP                    (1 << 0)

/* Sets Quad mode in configuration register
     1 = enable Quad mode
     0 = disable Quad mode */
#define S25_CFG_QUAD                                        (1 << 1)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Software write Enable */
static void s25WriteEN(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_WREN) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

/* Software write Enable */
static void s25ClearStatus(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_CLSR) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

static uint8_t s25ReadStatusRegister(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint8_t statRegister;

	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_RDSR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	statRegister = spifi_HW_GetData8(pSpifiCtrlAddr);

	/* Wait for command to complete */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	return statRegister;
}

/* Wait for device to complete operation (go unbusy) */
static void s25WaitUnBusy(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	/* Device wait for device to go unbusy */
	while ((s25ReadStatusRegister(pSpifiCtrlAddr) & S25_STAT_WIP) != 0) {}
}

/* Write Status / Config Register */
static void s25WriteStatusRegister(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, uint8_t status)
{
	s25WriteEN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_WRR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Write the data out */
	spifi_HW_SetData8(pSpifiCtrlAddr, status);

	/* Wait for Controler to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	s25WaitUnBusy(pSpifiCtrlAddr);
}

/* Read Status Register*/
static uint16_t s25ReadStatusConfigRegister(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint16_t statConfigRegisters;
	
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_RDSR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	statConfigRegisters = (((uint16_t) spifi_HW_GetData8(pSpifiCtrlAddr)) << 8);

	/* Wait for command to complete */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_RCR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	statConfigRegisters |= spifi_HW_GetData8(pSpifiCtrlAddr);

	/* Wait for command to complete */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	return statConfigRegisters;
}

/* Write Status / Config Register */
static void s25WriteStatusConfigRegisters(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, uint16_t statusConfig)
{
	s25WriteEN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(S25_CMD_WRR) |
					 SPIFI_CMD_DATALEN(2) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_SetData8(pSpifiCtrlAddr, (uint8_t) (statusConfig >> 8));
	spifi_HW_SetData8(pSpifiCtrlAddr, (uint8_t) statusConfig);

	/* Wait for LPC controller to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	s25WaitUnBusy(pSpifiCtrlAddr);
}

/* Set / Clear Quad mode */
static void s25SetQuadMode(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, bool enQuadMode)
{
	uint16_t statusConfig;

	statusConfig = s25ReadStatusConfigRegister(pSpifiCtrlAddr);

	/* Set or clear the quad enable bit */
	if (enQuadMode) {
		statusConfig |= S25_CFG_QUAD;
	}
	else {
		statusConfig &= ~S25_CFG_QUAD;
	}

	/* write status / config */
	s25WriteStatusConfigRegisters(pSpifiCtrlAddr, statusConfig);
}

/* Read Identification */
static void s25GetID(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, uint8_t *pID)
{
	/* Read ID command, plus read 3 bytes on data */
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_DATALEN(3) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP) |
					 SPIFI_CMD_OPCODE(S25_CMD_RDID)));

	/* Get info from the device */
	pID[0] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Manufacturers ID */
	pID[1] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Memory Type */
	pID[2] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Memmory Capacity */

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

/* Converts a device status to an OR'ed API status */
static uint32_t s25GetDevStatus(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint16_t devStat;
	uint32_t status = 0;

	/* Read device status word */
	devStat = s25ReadStatusRegister(pSpifiCtrlAddr);

	/* Convert to standard status values */
	if ((devStat & (S25_STAT_P_ERR | S25_STAT_W_ERR)) != 0) {
		if ((devStat & S25_STAT_P_ERR) != 0) {
			status |= SPIFI_STAT_PROGERR;
		}
		if ((devStat & S25_STAT_W_ERR) != 0) {
			status |= SPIFI_STAT_ERASEERR;
		}

		/* Clear latched status */
		s25ClearStatus(pSpifiCtrlAddr);
	}
	if ((devStat & S25_STAT_BPMASK) != 0) {
		if ((devStat & S25_STAT_BPMASK) == S25_STAT_BPMASK) {
			status |= SPIFI_STAT_FULLLOCK;
		}
		else {
			status |= SPIFI_STAT_PARTLOCK;
		}
	}
	if ((devStat & S25_STAT_WIP) != 0) {
		status |= SPIFI_STAT_BUSY;
	}

	return status;
}

/* Checks to see if the device is writable and not busy */
static SPIFI_ERR_T s25CheckWriteState(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint16_t stat;

	/* Get status */
	stat = s25ReadStatusRegister(pSpifiCtrlAddr);

	/* Exit if blocks are locked or WIP in progress */
	if ((stat & S25_STAT_BPMASK) != 0) {
		return SPIFI_ERR_LOCKED;
	}
	else if ((stat & S25_STAT_WIP) != 0) {
		return SPIFI_ERR_BUSY;
	}

	return SPIFI_ERR_NONE;
}

/* Find a matching entry in the device table */
static SPIFI_DEV_DATA_T *s25FindMatch(uint8_t *pID)
{
	SPIFI_DEV_DATA_T *pNode;

	/* search the list skipping over head node since it is a dummy node
	     and is NEVER used. */
	for (pNode = devHead.pNext; pNode != NULL; pNode = pNode->pNext) {
		/* Manufacturer match? */
		if (pID[0] == pNode->pDevData->id[0]) {
			/* Part match, check memory type and size fields */
			if ((pID[1] == pNode->pDevData->id[1]) && (pID[2] == pNode->pDevData->id[2])) {
				/* Match, time to exit */
				return pNode;
			}
		}
	}

	return NULL;
}

/* Enter memory mode */
static uint32_t deviceGetMemoryModeCmd(SPIFI_HANDLE_T *pHandle, bool enMMode)
{
	uint32_t cmdValue;
	
		if (pHandle->pInfoData->opts & SPIFI_OPT_USE_QUAD) {
			cmdValue =
				(SPIFI_CMD_OPCODE(S25_CMD_QOR) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}
		else {
			cmdValue =
				(SPIFI_CMD_OPCODE(S25_CMD_FAST_READ) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}
	
	return cmdValue;
}

/* Full device unlock */
static SPIFI_ERR_T deviceUnlockDevice(SPIFI_HANDLE_T *pHandle)
{
	uint8_t stat;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	/* Get current status */
	stat = s25ReadStatusRegister(pSpifiCtrlAddr);

	/* Clear lock bits only if they are locked */
	if ((stat & S25_STAT_BPMASK) != 0) {
		stat &= ~S25_STAT_BPMASK;

		s25WriteStatusRegister(pSpifiCtrlAddr, stat);
	}

	return SPIFI_ERR_NONE;
}

/* Full device lock */
static SPIFI_ERR_T deviceLockDevice(SPIFI_HANDLE_T *pHandle)
{
	uint8_t stat;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	/* Get current status */
	stat = s25ReadStatusRegister(pSpifiCtrlAddr);

	/* Clear lock bits only if they are locked */
	if ((stat & S25_STAT_BPMASK) != S25_STAT_BPMASK) {
		stat |= S25_STAT_BPMASK;

		s25WriteStatusRegister(pSpifiCtrlAddr, stat);
	}

	return SPIFI_ERR_NONE;
}

/* Unlock a block, not supported */
static SPIFI_ERR_T deviceUnlockBlock(SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	return SPIFI_ERR_NOTSUPPORTED;
}

/* Lock a block, not supported */
static SPIFI_ERR_T deviceLockBlock(SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	return SPIFI_ERR_NOTSUPPORTED;
}

/* Bulk Erase*/
static SPIFI_ERR_T deviceEraseAll(SPIFI_HANDLE_T *pHandle)
{
	SPIFI_ERR_T status;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	status = s25CheckWriteState(pSpifiCtrlAddr);
	if (status == SPIFI_ERR_NONE) {
		s25WriteEN(pSpifiCtrlAddr);
		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(S25_CMD_BULK_ERS) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		spifi_HW_WaitCMD(pSpifiCtrlAddr);

		/* Device wait for device to go unbusy */
		s25WaitUnBusy(pSpifiCtrlAddr);
	}

	return status;
}

/* Erase a block by block number */
static SPIFI_ERR_T deviceEraseBlock(SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	uint16_t stat;
	uint32_t addr;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	if (blockNum < pHandle->pInfoData->numBlocks) {
		status = s25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			addr = blockNum * pHandle->pInfoData->blockSize;
			s25ClearStatus(pSpifiCtrlAddr);
			s25WriteEN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr,
							(SPIFI_CMD_OPCODE(S25_CMD_SE) |
							 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));

			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to go unbusy */
				s25WaitUnBusy(pSpifiCtrlAddr);

				/* Read status and check error bits */
				stat = s25GetDevStatus(pSpifiCtrlAddr);
				if ((stat & SPIFI_STAT_ERASEERR) != 0) {
					status = SPIFI_ERR_ERASEERR;
				}
			}
		}
	}

	return status;
}

/* Erase a block by sub-block number */
static SPIFI_ERR_T deviceEraseSubBlock(SPIFI_HANDLE_T *pHandle, uint32_t subBlockNum)
{
	uint16_t stat;
	uint32_t addr;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	if (subBlockNum < pHandle->pInfoData->numSubBlocks) {
		status = s25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			addr = subBlockNum * pHandle->pInfoData->subBlockSize;
			s25ClearStatus(pSpifiCtrlAddr);
			s25WriteEN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr,
							(SPIFI_CMD_OPCODE(S25_CMD_P4E) |
							 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));

			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to go unbusy */
				s25WaitUnBusy(pSpifiCtrlAddr);

				/* Read status and check error bits */
				stat = s25GetDevStatus(pSpifiCtrlAddr);
				if ((stat & SPIFI_STAT_ERASEERR) != 0) {
					status = SPIFI_ERR_ERASEERR;
				}
			}
		}
	}

	return status;
}

/* Program a region */
static SPIFI_ERR_T devicePageProgram(SPIFI_HANDLE_T *pHandle, uint32_t addr, const uint32_t *writeBuff, uint32_t bytes)
{
	uint16_t stat;
	uint32_t cmdValue;
	uint8_t *writeBuff8 = (uint8_t *) writeBuff;
	SPIFI_ERR_T status = SPIFI_ERR_PAGESIZE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	if (bytes <= pHandle->pInfoData->pageSize) {
		status = s25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			s25ClearStatus(pSpifiCtrlAddr);
			s25WriteEN(pSpifiCtrlAddr);

			if (pHandle->pInfoData->opts & SPIFI_OPT_USE_QUAD) {
				cmdValue = (SPIFI_CMD_OPCODE(S25_CMD_QPP) |
							SPIFI_CMD_DATALEN(bytes) |
							SPIFI_CMD_DOUT(1) |
							SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
							SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
			}
			else {
				cmdValue = (SPIFI_CMD_OPCODE(S25_CMD_PP) |
							SPIFI_CMD_DATALEN(bytes) |
							SPIFI_CMD_DOUT(1) |
							SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
			}

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);

			/* Write data */
			while (bytes) {
				spifi_HW_SetData8(pSpifiCtrlAddr, *writeBuff8);
				++writeBuff8;
				--bytes;

			}
		}

		spifi_HW_WaitCMD(pSpifiCtrlAddr);

		/* If block is disabled, exit now */
		if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
			/* Device wait for device to go unbusy */
			s25WaitUnBusy(pSpifiCtrlAddr);

			/* Read status and check error bits */
			stat = s25GetDevStatus(pSpifiCtrlAddr);
			if ((stat & SPIFI_STAT_PROGERR) != 0) {
				status = SPIFI_ERR_PROGERR;
			}
		}
	}

	return status;
}

/* Read a region */
static SPIFI_ERR_T deviceRead(SPIFI_HANDLE_T *pHandle, uint32_t addr, uint32_t *readBuff, uint32_t bytes)
{
	uint8_t *readBuff8 = (uint8_t *) readBuff;
	SPIFI_ERR_T status = SPIFI_ERR_RANGE;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	uint32_t cmdValue;

	/* Limit read to controller data limit in bytes */
	if (bytes < pHandle->pInfoData->maxReadSize) {

		/* Choose how to program the read command */
		if (pHandle->pInfoData->opts & SPIFI_OPT_USE_QUAD) {
			cmdValue =
				(SPIFI_CMD_OPCODE(S25_CMD_QOR) |
				 SPIFI_CMD_DATALEN(bytes) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}
		else {
			cmdValue =
				(SPIFI_CMD_OPCODE(S25_CMD_FAST_READ) |
				 SPIFI_CMD_DATALEN(bytes) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}

		spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
		spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);

		/* Read data */
		while (bytes) {
			*readBuff8 = spifi_HW_GetData8(pSpifiCtrlAddr);
			++readBuff8;
			--bytes;
		}

		spifi_HW_WaitCMD(pSpifiCtrlAddr);
		status = SPIFI_ERR_NONE;
	}

	return status;
}

/* Set or unset driver options. Caller already updated driver options. */
static SPIFI_ERR_T deviceSetUnsetOptions(SPIFI_HANDLE_T *pHandle, uint32_t options, bool set)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	/* Enable / Disable quad mode */
	if (options & SPIFI_OPT_USE_QUAD) {
		s25SetQuadMode(pSpifiCtrlAddr, set);
	}

	return SPIFI_ERR_NONE;
}

/* Enable or disable software write protect state */
static SPIFI_ERR_T deviceReset(SPIFI_HANDLE_T *pHandle)
{
	return SPIFI_ERR_NOTSUPPORTED;
}

/* Returns information on the device */
static uint32_t deviceGetInfo(SPIFI_HANDLE_T *pHandle, SPIFI_INFO_ID_T infoId)
{
	uint32_t val;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	switch (infoId) {
	case SPIFI_INFO_DEVSIZE:
		val = pHandle->pInfoData->numBlocks * pHandle->pInfoData->blockSize;
		break;

	case SPIFI_INFO_ERASE_BLOCKS:
		val = pHandle->pInfoData->numBlocks;
		break;

	case SPIFI_INFO_ERASE_BLOCKSIZE:
		val = pHandle->pInfoData->blockSize;
		break;

	case SPIFI_INFO_ERASE_SUBBLOCKS:
		val = pHandle->pInfoData->numSubBlocks;
		break;

	case SPIFI_INFO_ERASE_SUBBLOCKSIZE:
		val = pHandle->pInfoData->subBlockSize;
		break;

	case SPIFI_INFO_PAGESIZE:
		val = pHandle->pInfoData->pageSize;
		break;

	case SPIFI_INFO_MAXREADSIZE:
		val = pHandle->pInfoData->maxReadSize;
		break;

	case SPIFI_INFO_MAXCLOCK:
		val = pHandle->pInfoData->maxClkRate;
		break;

	case SPIFI_INFO_CAPS:
		val = pHandle->pInfoData->caps;
		break;

	case SPIFI_INFO_STATUS:
		val = s25GetDevStatus(pSpifiCtrlAddr);
		break;

	case SPIFI_INFO_OPTIONS:
		val = pHandle->pInfoData->opts;
		break;

	default:
		val = 0;
		break;
	}

	return val;
}

/* Returns a string pointer to the generic device name */
static const char *deviceGetName(SPIFI_HANDLE_T *pHandle)
{
	return pDevNameString;
}

/* Detect if this device exists at the passed base address, returns 0 if the
   device doesn't exist of the required memory allocation size for the device
   context if the device exists. */
static uint32_t devDetect(uint32_t spifiCtrlAddr)
{
	SPIFI_DEV_DATA_T *devNode;
	uint32_t idx;
	uint8_t id[3];
	uint8_t idVerify[3];
	uint32_t detDevSize = 0;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)spifiCtrlAddr;
	
	
	/* Read device ID three times to validate. First read on a hard reset isn't reliable*/
	s25GetID(pSpifiCtrlAddr, id);
	s25GetID(pSpifiCtrlAddr, id);
	s25GetID(pSpifiCtrlAddr, idVerify);

	/* Compare both reads to make sure they match. If any byte doesn't compare, abort. */
	for (idx = 0; idx < sizeof(id); ++idx) {
		if (id[idx] != idVerify[idx]) {
			return detDevSize;
		}
	}

	/* Find a matching part */
	devNode = s25FindMatch(id);
	if (devNode) {
		/* Return size of device from blocks and size */
		detDevSize = devNode->pDevData->blks * devNode->pDevData->blkSize;
	}

	return detDevSize;
}

/* Setup a device */
static SPIFI_ERR_T devSetup(SPIFI_HANDLE_T *pHandle, uint32_t spifiCtrlAddr, uint32_t baseAddr)
{
	/* Spansion S25 device function table */
	static  SPIFI_DEV_T fxTable;
	SPIFI_DEV_DATA_T *devNode;
	SPIFI_ERR_T err = SPIFI_ERR_GEN;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)spifiCtrlAddr;
	
	/* Init the private device functions */
	fxTable.init = NULL;		/* No specific init required for this family */
	fxTable.deInit = NULL;	/* No specific deInit required for this family */
	fxTable.unlockDevice = deviceUnlockDevice;
	fxTable.lockDevice = deviceLockDevice;
	fxTable.unlockBlock = deviceUnlockBlock;
	fxTable.lockBlock = deviceLockBlock;
	fxTable.eraseAll = deviceEraseAll;
	fxTable.eraseBlock = deviceEraseBlock;
	fxTable.eraseSubBlock = deviceEraseSubBlock;
	fxTable.pageProgram = devicePageProgram;
	fxTable.read = deviceRead;
	fxTable.setOpts = deviceSetUnsetOptions;
	fxTable.reset = deviceReset;
	fxTable.getMemModeCmd = deviceGetMemoryModeCmd;
	fxTable.getDevName = deviceGetName;
	fxTable.getInfo = deviceGetInfo;

	/* Read JEDEC ID */
	s25GetID(pSpifiCtrlAddr, pHandle->pInfoData->id);

	/* Find a matching part */
	devNode = s25FindMatch(pHandle->pInfoData->id);
	if (devNode) {
		pHandle->pInfoData->spifiCtrlAddr = spifiCtrlAddr;
		pHandle->pInfoData->baseAddr = baseAddr;
		pHandle->pInfoData->numBlocks = devNode->pDevData->blks;
		pHandle->pInfoData->blockSize = devNode->pDevData->blkSize;
		pHandle->pInfoData->numSubBlocks = devNode->pDevData->subBlks;
		pHandle->pInfoData->subBlockSize = devNode->pDevData->subBlkSize;
		pHandle->pInfoData->pageSize = devNode->pDevData->pageSize;
		pHandle->pInfoData->maxReadSize = devNode->pDevData->maxReadSize;
		pHandle->pInfoData->maxClkRate = devNode->pDevData->maxClkRate;
		pHandle->pInfoData->caps = devNode->pDevData->caps;
		pHandle->pInfoData->pDevName = (char *) pDevNameString;

		err = pHandle->pInfoData->lastErr = SPIFI_ERR_NONE;
	}

	/* save pointer to device function table */
	pHandle->pDev = &fxTable;

	return err;
}

static SPIFI_ERR_T devRegister(void *pDevFamily, SPIFI_DEV_DATA_T *pDevData)
{
	/* insert into the beginning of the list */
	pDevData->pNext = devHead.pNext;
	devHead.pNext = pDevData;

	/* update the count of devices */
	++devCount;

	return SPIFI_ERR_NONE;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
SPIFI_DEV_FAMILY_T *SPIFI_REG_FAMILY_SpansionS25FLP(void)
{
	static SPIFI_DEV_FAMILY_T handle;
	static SPIFI_DEVDESC_T desc;

	/* Store the device specific info so it can be returned */
	desc.pDevName = pDevNameString;
	desc.prvDataSize = PRVDATASIZE;
	desc.pPrvDevDetect = devDetect;
	desc.pPrvDevSetup = devSetup;
	desc.pPrvDevRegister = devRegister;

	/* Save the descriptor in the handle */
	handle.pDesc = &desc;

	/* Make sure that the base list is empty */
	devHead.pNext = NULL;
	devCount = 0;

	{
		/* ID Spansion S25FL032P */
		static const SPIFI_DEV_PDATA_T pData = {
			{0x01, 0x02, 0x15, 0},	/* ID bytes returned from read id (4th byte not used) */
			64,         			/* # of blocks */
			0x10000,        		/* block size */
			1024,           		/* # of sub-blocks */
			0x1000,             	/* sub-block size */
			0x100,      			/* page size */
			32768,              	/* max single read bytes */
			40000000,				/* max clock rate in Hz */
			(SPIFI_CAP_QUAD | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK) /* capabilitites */
		};
		static SPIFI_DEV_DATA_T data;

		data.pDevData = &pData;
		devRegister(&handle, &data);
	}

	{
		/* Spansion S25FL129P */
		static const SPIFI_DEV_PDATA_T pData = {
			{0x01, 0x20, 0x18, 0},	/* ID bytes returned from read id (4th byte not used) */
			256,            		/* # of blocks */
			0x10000,    			/* block size */
			4096,           		/* # of sub-blocks */
			0x1000,             	/* sub-block size */
			0x100,      			/* page size */
			32768,          		/* max single read bytes */
			40000000,				/* max clock rate in Hz */
			(SPIFI_CAP_QUAD | SPIFI_CAP_FULLLOCK | SPIFI_CAP_NOBLOCK)
		};
		static SPIFI_DEV_DATA_T data;

		data.pDevData = &pData;
		devRegister(&handle, &data);
	}

	/* finally return the handle */
	return &handle;
}
