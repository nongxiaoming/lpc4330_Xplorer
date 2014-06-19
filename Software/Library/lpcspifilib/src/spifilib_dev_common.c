/*
 * @brief LPCSPIFILIB driver functions and structures that are not visible
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

#include "spifilib_api.h"
#include "spifilib_chiphw.h"
#include <string.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Declare the version numbers */
#define LIBRARY_VERSION_MAJOR (0)
#define LIBRARY_VERSION_MINOR (5)

/* device node count and linked list header */
static uint32_t famCount = 0;
static SPIFI_DEV_FAMILY_T famHead = {0};

/* Number of supported devices */
#define NUMSUPPDEVS (sizeof(pPrvDevs) / sizeof(SPIFI_DEVDESC_T *))

/* Mapped error strings to error codes */
static const char *spifiErrStrings[SPIFI_ERR_LASTINDEX] = {
	"No error",
	"Device is busy",
	"General error",
	"Capability not supported",
	"Alignment error",
	"Device is locked",
	"Program error",
	"Erase error",
	"Program region not blank",
	"Page size exceeded",
	"Validation error",
	"Range exceeded",
	"Not Allowed in Memory Mode"
};

static const char noName[] = "Invalid index";

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Detect first SPIFI FLASH device at the passed base address */
static SPIFI_DEV_FAMILY_T *spifiPartDetect(uint32_t spifiCtrlAddr)
{
	SPIFI_DEV_FAMILY_T *pNode;

	/* Loop through the library and check for detected devices.
	     skip over head node because it is NEVER used. */
	for (pNode = famHead.pNext; pNode != NULL; pNode = pNode->pNext) {
		if (pNode->pDesc->pPrvDevDetect(spifiCtrlAddr) > 0) {
			/* Match at this index */
			return pNode;
		}
	}

	return NULL;
}

static uint32_t spifiCalculateHandleSize(SPIFI_DEV_FAMILY_T *devData)
{
	/* This is the size needed for the device context instance by the driver */
	return sizeof(SPIFI_HANDLE_T) + sizeof(SPIFI_INFODATA_T) +
		   devData->pDesc->prvDataSize;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

SPIFI_DEV_FAMILY_T *spifiRegisterFamily(SPIFI_DEV_FAMILY_T *(*regFx)(void))
{
	SPIFI_DEV_FAMILY_T *pFam;

	/* Get the family node from the user */
	pFam = regFx();

	/* Insert the node into the beginning of the list */
	pFam->pNext = famHead.pNext;
	famHead.pNext = pFam;

	/* update the count of known families */
	++famCount;

	/* Nothing to do here yet */
	return SPIFI_ERR_NONE;
}

/* Report the library version number */
uint16_t spifiGetLibVersion(void)
{
	return (LIBRARY_VERSION_MAJOR << 8) | LIBRARY_VERSION_MINOR;
}

/* Initialize the SPIFILIB driver */
SPIFI_ERR_T spifiInit(uint32_t spifiCtrlAddr, bool reset)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)spifiCtrlAddr;
	
	if (reset) {
		/* Reset controller */
		spifi_HW_ResetController(pSpifiCtrlAddr);
		
		/* Set intermediate data and memcmd registers. */
		spifi_HW_SetIDATA(pSpifiCtrlAddr, 0x0);
		spifi_HW_SetMEMCMD(pSpifiCtrlAddr, 0);

		spifi_HW_ResetController(pSpifiCtrlAddr);
		
		/* Setup SPIFI controller */
		spifi_HW_SetCtrl(pSpifiCtrlAddr,
						 (SPIFI_CTRL_TO(1000) |
						  SPIFI_CTRL_CSHI(15) |
						  SPIFI_CTRL_RFCLK(1) |
						  SPIFI_CTRL_FBCLK(1)));
	}

	/* Nothing to do here yet */
	return SPIFI_ERR_NONE;
}

/* Shutdown the SPIFILIB driver */
void spifiDeInit(void)
{
	/* Nothing to do here yet */
}

/* performs device specific initialization */
SPIFI_ERR_T spifiDevInit(SPIFI_HANDLE_T *pHandle)
{
	SPIFI_ERR_T retValue = SPIFI_ERR_NONE;
	
	/* call device specific initializtion if provided */
	if (pHandle->pDev->init) {
		retValue = pHandle->pDev->init(pHandle);
	}
	
	/* make sure the controller is not in memMode */
	spifiDevSetMemMode(pHandle, false);
	
	return retValue;
}

/* performs device specific de-initialization */
SPIFI_ERR_T spifiDevDeInit(SPIFI_HANDLE_T *pHandle)
{
	SPIFI_ERR_T retValue = SPIFI_ERR_NONE;
	
	/* call device specific de-init if provided */
	if (pHandle->pDev->deInit) {
		retValue = pHandle->pDev->deInit(pHandle);
	}
	
	/* make sure the controller is in memMode */
	spifiDevSetMemMode(pHandle, true);
	
	return retValue;
}

/* Converts a SPIFILIB error code into a meaningful string */
const char *spifiReturnErrString(SPIFI_ERR_T errCode)
{
	if (((unsigned int) errCode) < SPIFI_ERR_LASTINDEX) {
		return spifiErrStrings[errCode];
	}

	return noName;
}

/* Returns status of memory mode */
bool spifiDevGetMemoryMode(SPIFI_HANDLE_T *pHandle)
{
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	return (spifi_HW_GetStat(pSpifiCtrlAddr) & SPIFI_STAT_MCINIT) != 0;
}

SPIFI_ERR_T spifiDevSetMemMode(SPIFI_HANDLE_T *pHandle, bool enMMode)
{
	uint32_t cmdValue;
	LPC_SPIFI_CHIPHW_T *pSpifiCtrlAddr = (LPC_SPIFI_CHIPHW_T *)pHandle->pInfoData->spifiCtrlAddr;
	
	/* RESET the memMode controller */
	spifi_HW_ResetController(pSpifiCtrlAddr);

	/* Wait for HW to acknowledge the reset. */
	spifi_HW_WaitRESET(pSpifiCtrlAddr);
	
	if (enMMode) {
		/* Get the device specific memory mode command */
		cmdValue = pHandle->pDev->getMemModeCmd(pHandle, enMMode);
	
		/* Specify the intermediate data byte. */
		spifi_HW_SetIDATA(pSpifiCtrlAddr, 0xA0);

		/* Set the appropriate values in the command reg. */
		spifi_HW_SetCmd(pSpifiCtrlAddr, cmdValue);
		spifi_HW_WaitCMD(pSpifiCtrlAddr);
		spifi_HW_SetMEMCMD(pSpifiCtrlAddr, cmdValue);
	}
	else {
		spifi_HW_SetIDATA(pSpifiCtrlAddr, 0x0);
		spifi_HW_SetMEMCMD(pSpifiCtrlAddr, 0);

		/* RESET the memMode controller */
		spifi_HW_ResetController(pSpifiCtrlAddr);
		
		/* Wait for HW to acknowledge the reset. */
		spifi_HW_WaitRESET(pSpifiCtrlAddr);
	}
	return SPIFI_ERR_NONE;
}

/* Return the number of supported device families in this driver */
uint32_t spifiGetSuppFamilyCount(void)
{
	/* return number of registered devices */
	return famCount;
}

/* Return the driver family name for a specific index */
const char *spifiGetSuppFamilyName(uint32_t index)
{
	uint32_t idx;
	SPIFI_DEV_FAMILY_T *pNode;

	if (index >= famCount) {
		return noName;
	}

	/* cycle through the list of families skipping over head node since it
	   is NEVER used. Once we break out of this loop pNode should be
	   pointing at the correct node.  */
	pNode = famHead.pNext;
	for (idx = 0; idx < index; ++idx) {
		pNode = pNode->pNext;
	}

	return pNode->pDesc->pDevName;
}

/* Detect and return memory needed for device handle at passed address */
uint32_t spifiGetHandleMemSize(uint32_t spifiCtrlAddr)
{
	uint32_t bytesNeeded = 0;
	SPIFI_DEV_FAMILY_T *detectedPart;

	/* Find first device at the base address */
	detectedPart = spifiPartDetect(spifiCtrlAddr);
	if (detectedPart) {
		/* This is the size needed for the device context instance by the driver */
		bytesNeeded = spifiCalculateHandleSize(detectedPart);
	}

	return bytesNeeded;
}

/* Initialize driver and hardware for a specific device */
SPIFI_HANDLE_T *spifiInitDevice(void *pMem, uint32_t sizePMem, uint32_t spifiCtrlAddr, uint32_t baseAddr)
{
	SPIFI_DEV_FAMILY_T *detectedPart;
	SPIFI_HANDLE_T *pSpifiHandle;
	uint32_t *pMem32 = (uint32_t *) pMem;

	/* Is the passed buffer size aligned on a 32-bit boundary? */
	if (((uint32_t) pMem32 & 0x3) != 0) {
		return NULL;
	}

	/* Detect the device at at the base address and abort on error. */
	detectedPart = spifiPartDetect(spifiCtrlAddr);
	if (!detectedPart) {
		return NULL;
	}

	/* Is passed memory space big enough? */
	if (spifiCalculateHandleSize(detectedPart) > sizePMem) {
		return NULL;
	}

	/* Setup handle */
	pSpifiHandle = (SPIFI_HANDLE_T *) pMem;

	/* Clear entire device context areas */
	memset(pMem, 0, sizePMem);

	/* Setup device info region */
	pMem32 += (sizeof(SPIFI_HANDLE_T) / sizeof(uint32_t));
	pSpifiHandle->pInfoData = (SPIFI_INFODATA_T *) pMem32;

	/* Setup device private data region */
	pMem32 += (sizeof(SPIFI_INFODATA_T) / sizeof(uint32_t));
	pSpifiHandle->pDevData = (void *) pMem32;

	/* Setup device specific data, then call device init */
	if (detectedPart->pDesc->pPrvDevSetup(pSpifiHandle, spifiCtrlAddr, baseAddr) != SPIFI_ERR_NONE) {
		return NULL;
	}
	
	if ((pSpifiHandle->pDev->init) && (pSpifiHandle->pDev->init(pSpifiHandle) != SPIFI_ERR_NONE)) {
		/* Some kind of error, stop */
		return NULL;
	}
	
	return pSpifiHandle;
}

SPIFI_ERR_T spifiDevSetOpts(SPIFI_HANDLE_T *pHandle, uint32_t options, bool set)
{
	/* default to not supported */
	SPIFI_ERR_T retValue = SPIFI_ERR_NOTSUPPORTED;
	bool memMode;

	if ((options & pHandle->pInfoData->caps) == options) {

		/* first get the current memory mode */
		memMode = spifiDevGetMemoryMode(pHandle);

		/* Set the option in the driver so other routines will act accordingly */
		if (set) {
			pHandle->pInfoData->opts |= options;
		}
		else {
			pHandle->pInfoData->opts &= ~options;
		}

		/* Perform device specific setup for the option */
		retValue = pHandle->pDev->setOpts(pHandle, options, set);

		/* If changing the state of Quad support update memory mode */
		if (options & SPIFI_OPT_USE_QUAD) {
			spifiDevSetMemMode(pHandle, memMode);
		}
	}
	return retValue;
}

/* Returns the address mapped to an block number */
uint32_t spifiGetAddrFromBlock(SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	uint32_t baseAddr = 0xFFFFFFFF;

	if (blockNum < pHandle->pInfoData->numBlocks) {
		baseAddr = pHandle->pInfoData->baseAddr + (blockNum * pHandle->pInfoData->blockSize);
	}

	return baseAddr;
}

/* Returns the starting address of a sub-block number */
uint32_t spifiGetAddrFromSubBlock(SPIFI_HANDLE_T *pHandle, uint32_t subBlockNum)
{
	uint32_t baseAddr = 0xFFFFFFFF;

	if (subBlockNum < pHandle->pInfoData->numSubBlocks) {
		baseAddr = pHandle->pInfoData->baseAddr + (subBlockNum * pHandle->pInfoData->subBlockSize);
	}

	return baseAddr;
}

/* Returns the block number the passedd= address is located in */
uint32_t spifiGetBlockFromAddr(SPIFI_HANDLE_T *pHandle, uint32_t addr)
{
	uint32_t block;
	block = (addr - pHandle->pInfoData->baseAddr) / pHandle->pInfoData->blockSize;

	if (block >= pHandle->pInfoData->numBlocks) {
		return ~0UL;
	}

	return block;
}

/* Returns the sub-block number the passed address is located in */
uint32_t spifiGetSubBlockFromAddr(SPIFI_HANDLE_T *pHandle, uint32_t addr)
{
	uint32_t subBlock;

	subBlock = (addr - pHandle->pInfoData->baseAddr) / pHandle->pInfoData->subBlockSize;

	if (subBlock >= pHandle->pInfoData->numSubBlocks) {
		return ~0UL;
	}

	return subBlock;
}

/* Returns the first sub-block in hte passed block */
uint32_t spifiGetSubBlockFromBlock(SPIFI_HANDLE_T *pHandle, uint32_t blockNum)
{
	uint32_t subBlock = 0xffffffff;

	/* If the blockNum passed is larger than this device, report error */
	if (blockNum >= pHandle->pInfoData->numBlocks) {
		return subBlock;
	}

	/* Calculate the sub-block number based on detected params */
	subBlock = (blockNum * (pHandle->pInfoData->blockSize / pHandle->pInfoData->subBlockSize));

	return subBlock;
}

/* Program the device with the passed buffer */
SPIFI_ERR_T spifiProgram(SPIFI_HANDLE_T *pHandle, uint32_t addr, const uint32_t *writeBuff, uint32_t bytes)
{
	uint32_t sendBytes;
	SPIFI_ERR_T err = SPIFI_ERR_NONE;

	/* Program using up to page size */
	while ((bytes > 0) && (err == SPIFI_ERR_NONE)) {
		sendBytes = bytes;
		if (sendBytes > pHandle->pInfoData->pageSize) {
			sendBytes = pHandle->pInfoData->pageSize;
		}

		err = pHandle->pDev->pageProgram(pHandle, addr, writeBuff, sendBytes);
		addr += sendBytes;
		writeBuff += (sendBytes / sizeof(uint32_t));
		bytes -= sendBytes;
	}

	return err;
}

/* Read the device into the passed buffer */
SPIFI_ERR_T spifiRead(SPIFI_HANDLE_T *pHandle, uint32_t addr, uint32_t *readBuff, uint32_t bytes)
{
	uint32_t readBytes;
	SPIFI_ERR_T err = SPIFI_ERR_NONE;

	/* Read using up to the maximum read size */
	while ((bytes > 0) && (err == SPIFI_ERR_NONE)) {
		readBytes = bytes;
		if (readBytes > pHandle->pInfoData->maxReadSize) {
			readBytes = pHandle->pInfoData->maxReadSize;
		}

		err = pHandle->pDev->read(pHandle, addr, readBuff, readBytes);
		addr += readBytes;
		readBuff += (readBytes / sizeof(uint32_t));
		bytes -= readBytes;
	}

	return err;
}

/* Erase multiple blocks */
SPIFI_ERR_T spifiErase(SPIFI_HANDLE_T *pHandle, uint32_t firstBlock, uint32_t numBlocks)
{
	SPIFI_ERR_T err = SPIFI_ERR_NONE;

	if ((firstBlock + numBlocks) > pHandle->pInfoData->numBlocks) {
		return SPIFI_ERR_RANGE;
	}

	/* Only perform erase if numBlocks is != 0 */
	for (; (numBlocks); ++firstBlock, --numBlocks) {
		err = pHandle->pDev->eraseBlock(pHandle, firstBlock);
		if (err != SPIFI_ERR_NONE) {
			break;
		}
	}

	return err;
}

/* Erase multiple blocks by address range */
SPIFI_ERR_T spifiEraseByAddr(SPIFI_HANDLE_T *pHandle, uint32_t firstAddr, uint32_t lastAddr)
{
	uint32_t firstBlock, lastBlock;
	SPIFI_ERR_T err = SPIFI_ERR_RANGE;

	/* Get block numbers for addresses */
	firstBlock = spifiGetBlockFromAddr(pHandle, firstAddr);
	lastBlock = spifiGetBlockFromAddr(pHandle, lastAddr);

	/* Limit to legal address range */
	if ((firstBlock != 0xFFFFFFFF) && (lastBlock != 0xFFFFFFFF)) {
		err = spifiErase(pHandle, firstBlock, ((lastBlock - firstBlock) + 1));
	}

	return err;
}
