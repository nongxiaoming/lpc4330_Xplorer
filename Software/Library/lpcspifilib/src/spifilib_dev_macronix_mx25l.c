/*
 * @brief Macronix MX25 driver
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

#include "spifilib_dev.h"
#include "spifilib_chiphw.h"

/** @defgroup LPCSPIFILIB_CONFIG_MX25 LPCSPIFILIB Macronix MX25xxx5 family device support
 * @ingroup LPCSPIFILIB_DRIVERS
 * This driver includes support for the following devices.<br>
 * MX25L3235E<br>
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
static const char pDevNameString[] = "Macronix MX25 FLASH";

/* Declare the base node */
static uint32_t devCount;
static SPIFI_DEV_DATA_T devHead = {0};

/* Required private data size for this driver */
#define PRVDATASIZE 0

/* Command definitions */
#define MX25_CMD_READ_DATA               0x03		/**< Read Data bytes */
#define MX25_CMD_FAST_READ               0x0B		/**< Read Data bytes at Fast Speed */
#define MX25_CMD_QOR                     0x6B		/**< Quad Output Read */
#define MX25_CMD_RDID                    0x9F		/**< Read Identification */
#define MX25_CMD_WREN                    0x06		/**< Write Enable */
#define MX25_CMD_P4E                     0x20		/**< 4 KB Parameter Sector Erase */
#define MX25_CMD_BULK_ERS                0xC7		/**< Bulk Erase */
#define MX25_CMD_SE                      0xD8		/**< 64KB Sector Erase */
#define MX25_CMD_PP                      0x02		/**< Page Programming */
#define MX25_CMD_QPP                     0x32		/**< Quad Page Programming */
#define MX25_CMD_RDSR                    0x05		/**< Read Status Register */
#define MX25_CMD_WRR                     0x01		/**< Write (Status & Configuration) Register */
#define MX25_CMD_RCR                     0x35		/**< Read Configuration Register (CFG) */
#define MX25_CMD_CLSR                    0x30		/**< Reset the Erase and Program Fail Flag (SR5 and SR6) and restore normal operation) */

/* Status register definitions */
/* Status Register Write Disable,
   1 = Protects when W# is low,
   0 = No protection, even when W# is low */
#define MX25_STAT_SRWD                   (1 << 7)
/* Programming Error Occurred,
   0 = No Error,
   1 = Error occurred */
#define MX25_STAT_QUAD_ENABLE            (1 << 6)
/* Block protect bits,
   Protects upper half of address range in 5 sizes */
#define MX25_STAT_BPMASK                 (0xf << 2)
/* Write Enable Latch,
   1 = Device accepts Write Status Register, program, or erase commands,
   0 = Ignores Write Status Register, program, or erase commands */
#define MX25_STAT_WEL                    (1 << 1)
/* Write in Progress,
   1 = Device Busy. A Write Status Register, program, or erase,
   0 = Ready. Device is in standby mode and can accept commands. */
#define MX25_STAT_WIP                    (1 << 0)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Software write Enable */
static void mx25WriteEN(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(MX25_CMD_WREN) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

static uint8_t mx25ReadStatusRegister(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint8_t statRegister;
	
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(MX25_CMD_RDSR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	statRegister = spifi_HW_GetData8(pSpifiCtrlAddr);

	/* Wait for command to complete */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	return statRegister;
}

/* Wait for device to complete operation (go unbusy) */
static void mx25WaitUnBusy(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	
	/* Device wait for device to go unbusy  I.e wip and wel are both 0 */
	while ((mx25ReadStatusRegister(pSpifiCtrlAddr) & (MX25_STAT_WIP | MX25_STAT_WEL)) != 0) {}
}

/* Write Status / Config Register */
static void mx25WriteStatusRegister(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, uint8_t status)
{
	mx25WriteEN(pSpifiCtrlAddr);
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(MX25_CMD_WRR) |
					 SPIFI_CMD_DATALEN(1) |
					 SPIFI_CMD_DOUT(1) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Write the data out */
	spifi_HW_SetData8(pSpifiCtrlAddr, status);

	/* Wait for Controler to finish command */
	spifi_HW_WaitCMD(pSpifiCtrlAddr);

	/* Wait for flash controller to finish command */
	mx25WaitUnBusy(pSpifiCtrlAddr);
}

/* Set / Clear Quad mode */
static void mx25SetQuadMode(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, bool enQuadMode)
{
	uint8_t status;

	status = mx25ReadStatusRegister(pSpifiCtrlAddr);

	/* Set or clear the quad enable bit */
	if (enQuadMode) {
		status |= MX25_STAT_QUAD_ENABLE;
	}
	else {
		status &= ~MX25_STAT_QUAD_ENABLE;
	}

	/* write status / config */
	mx25WriteStatusRegister(pSpifiCtrlAddr, status);
}

/* Read Identification */
static void mx25GetID(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr, uint8_t *pID)
{
	/* Read ID command, plus read 3 bytes on data */
	spifi_HW_SetCmd(pSpifiCtrlAddr,
					(SPIFI_CMD_OPCODE(MX25_CMD_RDID) |
					 SPIFI_CMD_DATALEN(3) |
					 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
					 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

	/* Get info from the device */
	pID[0] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Manufacturers ID */
	pID[1] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Memory Type */
	pID[2] = spifi_HW_GetData8(pSpifiCtrlAddr);	/* Memmory Capacity */

	spifi_HW_WaitCMD(pSpifiCtrlAddr);
}

/* Converts a device status to an OR'ed API status */
static uint32_t mx25GetDevStatus(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint16_t devStat;
	uint32_t status = 0;

	/* Read device status word */
	devStat = mx25ReadStatusRegister(pSpifiCtrlAddr);

	if ((devStat & MX25_STAT_BPMASK) != 0) {
		if ((devStat & MX25_STAT_BPMASK) == MX25_STAT_BPMASK) {
			status |= SPIFI_STAT_FULLLOCK;
		}
		else {
			status |= SPIFI_STAT_PARTLOCK;
		}
	}

	if ((devStat & MX25_STAT_WIP) != 0) {
		status |= SPIFI_STAT_BUSY;
	}

	return status;
}

/* Checks to see if the device is writable and not busy */
static SPIFI_ERR_T mx25CheckWriteState(LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr)
{
	uint16_t stat;

	/* Get status */
	stat = mx25ReadStatusRegister(pSpifiCtrlAddr);

	/* Exit if blocks are locked or WIP in progress */
	if ((stat & MX25_STAT_BPMASK) != 0) {
		return SPIFI_ERR_LOCKED;
	}
	else if ((stat & MX25_STAT_WIP) != 0) {
		return SPIFI_ERR_BUSY;
	}

	return SPIFI_ERR_NONE;
}

/* Find a matching entry in the device table */
static SPIFI_DEV_DATA_T *mx25FindMatch(uint8_t *pID)
{
	SPIFI_DEV_DATA_T *pNode;

	/* search the list skipping over head node since it is NEVER used */
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
				(SPIFI_CMD_OPCODE(MX25_CMD_QOR) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}
		else {
			cmdValue =
				(SPIFI_CMD_OPCODE(MX25_CMD_FAST_READ) |
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
	stat = mx25ReadStatusRegister(pSpifiCtrlAddr);

	/* Clear lock bits only if they are locked */
	if ((stat & MX25_STAT_BPMASK) != 0) {
		stat &= ~MX25_STAT_BPMASK;

		mx25WriteStatusRegister(pSpifiCtrlAddr, stat);
	}

	return SPIFI_ERR_NONE;
}

/* Full device lock */
static SPIFI_ERR_T deviceLockDevice(SPIFI_HANDLE_T *pHandle)
{
	uint8_t stat;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	/* Get current status */
	stat = mx25ReadStatusRegister(pSpifiCtrlAddr);

	/* Clear lock bits only if they are locked */
	if ((stat & MX25_STAT_BPMASK) != MX25_STAT_BPMASK) {
		stat |= MX25_STAT_BPMASK;

		mx25WriteStatusRegister(pSpifiCtrlAddr, stat);
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
	
	status = mx25CheckWriteState(pSpifiCtrlAddr);
	if (status == SPIFI_ERR_NONE) {
		mx25WriteEN(pSpifiCtrlAddr);
		spifi_HW_SetCmd(pSpifiCtrlAddr,
						(SPIFI_CMD_OPCODE(MX25_CMD_BULK_ERS) |
						 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
						 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP)));

		spifi_HW_WaitCMD(pSpifiCtrlAddr);

		/* Device wait for device to go unbusy */
		mx25WaitUnBusy(pSpifiCtrlAddr);
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
		status = mx25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			addr = blockNum * pHandle->pInfoData->blockSize;
			mx25WriteEN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr,
							(SPIFI_CMD_OPCODE(MX25_CMD_SE) |
							 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));

			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to go unbusy */
				mx25WaitUnBusy(pSpifiCtrlAddr);

				/* Read status and check error bits */
				stat = mx25GetDevStatus(pSpifiCtrlAddr);
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
		status = mx25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			addr = subBlockNum * pHandle->pInfoData->subBlockSize;
			mx25WriteEN(pSpifiCtrlAddr);

			spifi_HW_SetAddr(pSpifiCtrlAddr, addr);
			spifi_HW_SetCmd(pSpifiCtrlAddr,
							(SPIFI_CMD_OPCODE(MX25_CMD_P4E) |
							 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_ALL_SERIAL) |
							 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS)));

			spifi_HW_WaitCMD(pSpifiCtrlAddr);

			/* If blocking is disabled, exit now */
			if ((pHandle->pInfoData->opts & SPIFI_OPT_NOBLOCK) == 0) {
				/* Device wait for device to go unbusy */
				mx25WaitUnBusy(pSpifiCtrlAddr);

				/* Read status and check error bits */
				stat = mx25GetDevStatus(pSpifiCtrlAddr);
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
		status = mx25CheckWriteState(pSpifiCtrlAddr);
		if (status == SPIFI_ERR_NONE) {
			mx25WriteEN(pSpifiCtrlAddr);

			if (pHandle->pInfoData->opts & SPIFI_OPT_USE_QUAD) {
				cmdValue = (SPIFI_CMD_OPCODE(MX25_CMD_QPP) |
							SPIFI_CMD_DATALEN(bytes) |
							SPIFI_CMD_DOUT(1) |
							SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
							SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
			}
			else {
				cmdValue = (SPIFI_CMD_OPCODE(MX25_CMD_PP) |
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
			mx25WaitUnBusy(pSpifiCtrlAddr);

			/* Read status and check error bits */
			stat = mx25GetDevStatus(pSpifiCtrlAddr);
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

		/* Setup to send the command */
		if (pHandle->pInfoData->opts & SPIFI_OPT_USE_QUAD) {
			cmdValue =
				(SPIFI_CMD_OPCODE(MX25_CMD_QOR) |
				 SPIFI_CMD_DATALEN(bytes) |
				 SPIFI_CMD_DOUT(0) |
				 SPIFI_CMD_INTER(1) |
				 SPIFI_CMD_FIELDFORM(SPIFI_FIELDFORM_SERIAL_OPCODE_ADDRESS) |
				 SPIFI_CMD_FRAMEFORM(SPIFI_FRAMEFORM_OP_3ADDRESS));
		}
		else {
			cmdValue =
				(SPIFI_CMD_OPCODE(MX25_CMD_FAST_READ) |
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
		mx25SetQuadMode(pSpifiCtrlAddr, set);
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
		val = mx25GetDevStatus(pSpifiCtrlAddr);
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
	int idx;
	uint8_t id[3];
	uint8_t idVerify[3];
	uint32_t detDevSize = 0;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)spifiCtrlAddr;
	
	/* Read device ID three times to validate. First read on a hard reset isn't reliable*/
	mx25GetID(pSpifiCtrlAddr, id);
	mx25GetID(pSpifiCtrlAddr, id);
	mx25GetID(pSpifiCtrlAddr, idVerify);

	/* Compare both reads to make sure they match. If any byte doesn't compare, abort. */
	for (idx = 0; idx < sizeof(id); ++idx) {
		if (id[idx] != idVerify[idx]) {
			return detDevSize;
		}
	}

	/* Find a matching part */
	devNode = mx25FindMatch(id);
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
	fxTable.init = NULL;			/* no device specific init required */
	fxTable.deInit = NULL;		/* no device specific deInit required */
	fxTable.getMemModeCmd = deviceGetMemoryModeCmd;
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
	fxTable.getDevName = deviceGetName;
	fxTable.getInfo = deviceGetInfo;

	/* Read JEDEC ID */
	mx25GetID(pSpifiCtrlAddr, pHandle->pInfoData->id);

	/* Find a matching part */
	devNode = mx25FindMatch(pHandle->pInfoData->id);
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

	/* update the number of devices in the list */
	++devCount;

	/* Nothing to do here yet */
	return SPIFI_ERR_NONE;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
SPIFI_DEV_FAMILY_T *SPIFI_REG_FAMILY_MacronixMX25L(void)
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

	/* Make sure that the base list is empty and the count reflects 0 */
	devHead.pNext = NULL;
	devCount = 0;

	/* Add support for MX25L3235E */
	{
		/* Macronix MX25L3235E */
		static const SPIFI_DEV_PDATA_T pData = {
			{0xC2, 0x20, 0x16, 0},	/* ID bytes returned from read id (4th byte not used) */
			64,						/* # of blocks */
            0x10000,        		/* block size */
			1024,               	/* # of sub-blocks */
			0x1000,         		/* sub-block size */
			0x100,          		/* page size */
			32768,					/* max single read bytes */
			80000000,				/* max clock rate in Hz */
			(SPIFI_CAP_QUAD | SPIFI_CAP_NOBLOCK)	/* capabilities */
		};
		static SPIFI_DEV_DATA_T data;

		data.pDevData = &pData;
		devRegister(&handle, &data);
	}

	/* finally return the handle */
	return &handle;
}
