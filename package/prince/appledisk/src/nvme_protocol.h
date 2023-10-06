/*
 * Copyright (C) 2012 Apple Inc. All rights reserved.
 *
 * This document is the property of Apple Inc.
 * It is considered confidential and proprietary.
 *
 * This document may not be reproduced or transmitted in any form,
 * in whole or in part, without the express written permission of
 * Apple Inc.
 */
#ifndef NVME_PROTOCOL_H
#define NVME_PROTOCOL_H

#include <sys.h>

#if( !defined( OSCompileAssert ) )
#	define OSCompileAssert( TEST )     \
	extern int OSCompileAssertFailed[ ( TEST ) ? 1 : -1 ] __unused;
#endif

typedef union nvme_command {
	struct {
		uint8_t		opc;		// CDW[0] bits 0-7
		uint8_t		fuse:2;		// CDW[0] bits 8-9
		uint8_t		reserved0:3;	// CDW[0] bits 10-12
		uint8_t         lprp:1;		// CDW[0] bit 13
		uint8_t		reserved1:2;	// CDW[0] bits 14-15
		uint16_t	cid;		// CDW[0] bits 16-31
		uint32_t	nsid;		// CDW[1]
		uint64_t	reserved2;	// CDW[3:2]
		uint64_t	mptr;		// CDW[5:4]
		uint64_t	prp1;		// CDW[7:6]
		uint64_t	prp2;		// CDW[9:8]
	};
	uint32_t cdw[16];
} nvme_command_t;

typedef union nvme_completion {
	struct {
		uint32_t	cs;
		uint32_t	reserved0;
		uint16_t	sqhd;
		uint16_t	sqid;
		uint16_t	cid;
		uint16_t	p:1;
		uint16_t	sc:8;
		uint16_t	sct:3;
		uint16_t	reserved1:2;
		uint16_t	m:1;
		uint16_t	dnr:1;
	};
	uint32_t cdw[4];
} nvme_completion_t;

typedef struct nvme_identify_namespace {
	uint64_t	nsze;
	uint64_t	ncap;
	uint64_t	nuse;
	uint8_t		nfeat;
	uint8_t		nlbaf;
	uint8_t		flbas;
	uint8_t		mc;
	uint8_t		dpc;
	uint8_t		dps;
	uint8_t		reserved1[98];
	struct {
		uint16_t	ms;
		uint8_t		lbads;
		uint8_t		rp:2;
		uint8_t		reserved:6;
	} lbaf[16];
	uint8_t		reserved2[192];
	uint8_t		vs[3712];
} nvme_identify_namespace_t;

typedef struct NVMePowerStateDescriptor
{
	uint16_t	MAXIMUM_POWER;
	uint8_t		RESERVED1[2];
	uint32_t	ENTRY_LATENCY;
	uint32_t	EXIT_LATENCY;
	uint8_t		RELATIVE_READ_THROUGHPUT;
	uint8_t		RELATIVE_READ_LATENCY;
	uint8_t		RELATIVE_WRITE_THROUGHPUT;
	uint8_t		RELATIVE_WRITE_LATENCY;
	uint8_t		RESERVED2[16];
} NVMePowerStateDescriptor;

OSCompileAssert ( sizeof ( NVMePowerStateDescriptor ) == 32 );

enum
{
	kNVMeIdentifyControllerSerialNumberLen 			= 20,
	kNVMeIdentifyControllerModelNumberLen			= 40,
	kNVMeIdentifyControllerFirmwareRevisionLen		= 8,
	kNVMeIdentifyControllerIEEEOUIIDLen				= 3,
	kNVMeIdentifyControllerMSPRevisionLen			= 16,
	kNVMeIdentifyControllerPTSRevisionLen			= 12,
	kNVMeIdentifyControllerBuildTrainLen			= 16,
	kNVMeIdentifyControllerNANDTypeLen				= 32,
	kNVMeIdentifyControllerNANDVendorLen			= 16
};

enum
{
	NVME_MODESEL_NORMAL	= 0,
	NVME_MODESEL_BOOT 	= 1,
	NVME_MODESEL_READONLY	= 2,
	NVME_MODESEL_RECOVERY	= 3,
	NVME_MODESEL_MAX	= 4
};

#define kNVMeIdentifyControllerNumFuses		8

typedef struct NVMeIdentifyControllerStruct
{
	// Controller Capabilites and Features
	uint16_t	PCI_VID;
	uint16_t	PCI_SSVID;
	uint8_t		SERIAL_NUMBER[kNVMeIdentifyControllerSerialNumberLen];
	uint8_t		MODEL_NUMBER[kNVMeIdentifyControllerModelNumberLen];
	uint8_t		FW_REVISION[kNVMeIdentifyControllerFirmwareRevisionLen];
	uint8_t		RECOMMENDED_ARBITRATION_BURST;
	uint8_t		IEEE_OUI_ID[kNVMeIdentifyControllerIEEEOUIIDLen];
	uint8_t		MIC;
	uint8_t		MAX_DATA_TRANSFER_SIZE;
	uint16_t	CONTROLLER_ID;
	uint8_t		RESERVED1[176];

	// Admin Command Set Attributes
	uint16_t	OPTIONAL_ADMIN_COMMAND_SUPPORT;
	uint8_t		ABORT_COMMAND_LIMIT;
	uint8_t		ASYNC_EVENT_REQUEST_LIMIT;
	uint8_t		FW_UPDATES;
	uint8_t		LOG_PAGE_ATTR;
	uint8_t		ERROR_LOG_PAGE_ENTRIES;
	uint8_t		NUM_OF_POWER_STATE_SUPPORT;
	uint8_t		ADMIN_VENDOR_SPECIFIC_COMMAND_CONFIG;
	uint8_t		AUTONOMOUS_POWER_STATE_TRANSITION_ATTR;
	uint8_t		RESERVED2[246];

	// NVM Command Set Attributes
	uint8_t		SQ_ENTRY_SIZE;
	uint8_t		CQ_ENTRY_SIZE;
	uint8_t		RESERVED3[2];
	uint32_t	NUMBER_OF_NAMESPACES;
	uint16_t	OPTIONAL_NVM_COMMAND_SUPPORT;
	uint16_t	FUSE_OP_SUPPORT;
	uint8_t		FORMAT_NVM_ATTR;
	uint8_t		VOLATILE_WRITE_CACHE;
	uint16_t	ATOMIC_WRITE_UNIT_NORMAL;
	uint16_t	ATOMIC_WRITE_UNIT_POWER_FAIL;
	uint8_t		NVM_VENDOR_SPECIFIC_COMMAND_CONFIG;
	uint8_t		RESERVED4[1];
	uint16_t	ATOMIC_COMPARE_AND_WRITE_UNIT;
	uint8_t		RESERVED5[2];
	uint32_t	SGL_SUPPORT;
	uint8_t		RESERVED6[164];

	// I/O Command Set Attributes
	uint8_t		RESERVED7[1344];

	// Power State Descriptors
	NVMePowerStateDescriptor	POWER_STATE_DESCRIPTORS[32];

	// Vendor Specific Runtime Data
	uint8_t		MSP_REVISION[kNVMeIdentifyControllerMSPRevisionLen];
	uint32_t	TIME_TO_READY;
	uint32_t	TRIMMER_STATUS;
	uint8_t		BOOT_SOURCE;
	uint8_t		BOOT_BLOCK_INDEX;
	uint8_t		RESERVED8[2];
	uint8_t		S3_CHIP_ID;
	uint8_t		S3_CHIP_REVISION;
	uint16_t	S3_DEVICE_DESCRIPTOR;
	uint8_t		S3_ECC_VERSION_NAND_REVISION;
	uint8_t		S3_FTL_MAJOR_VERSION;
	uint8_t		S3_FTL_MINOR_VERSION;
	uint8_t		S3_DM_VERSION;
	uint8_t		S3_CONFIG_VERSION;
	uint8_t		S3_FORMAT_UTIL_MAJOR_VERSION;
	uint8_t		RESERVED9[6];
	uint8_t		ASP_BUILD_TRAIN[kNVMeIdentifyControllerBuildTrainLen];
	uint8_t		MSP_BUILD_TRAIN[kNVMeIdentifyControllerBuildTrainLen];
	uint8_t		RESERVED10[436];
	// Vendor Specific Production Data
	uint8_t		BOARD_TYPE;
	uint8_t		NAND_TYPE;
	uint8_t		NAND_VENDOR;
	uint8_t		DRAM_VENDOR;
	uint8_t		NOR_VENDOR;
	uint8_t		CHANNELS_BITMAP;
	uint16_t	SSD_CAPACITY;
	uint8_t		CORNER_TYPE;
	uint8_t		ROM_VERSION;
	uint8_t		PTS_REVISION[kNVMeIdentifyControllerPTSRevisionLen];
	uint16_t	PTS_STATUS_BITMAP;
	uint8_t		NAND_TYPE_STRING[kNVMeIdentifyControllerNANDTypeLen];
	uint8_t		NAND_VENDOR_STRING[kNVMeIdentifyControllerNANDVendorLen];
	uint32_t	MAXIMUM_PRIMARY_NS_SIZE;
	uint8_t		RESERVED11[180];
	uint32_t	RAW_FUSES[kNVMeIdentifyControllerNumFuses];
	uint8_t		RESERVED12[224];
	
} NVMeIdentifyControllerStruct;

#define NVME_FUSE_VALUE_FOR_HYNIX				1
#define NVME_FUSE_VALUE_FOR_SANDISK				3
#define NVME_FUSE_VALUE_FOR_SAMSUNG				4
#define NVME_FUSE_VALUE_FOR_TOSHIBA				5

typedef enum
{
	NVME_NAND_DEVICE_HYNIX		= NVME_FUSE_VALUE_FOR_HYNIX,
	NVME_NAND_DEVICE_SANDISK	= NVME_FUSE_VALUE_FOR_SANDISK,
    NVME_NAND_DEVICE_SAMSUNG    = NVME_FUSE_VALUE_FOR_SAMSUNG,
	NVME_NAND_DEVICE_TOSHIBA	= NVME_FUSE_VALUE_FOR_TOSHIBA,
	NVME_NAND_NUM_VENDORS
} NVME_NAND_DEVICE_VENDOR_E;

typedef enum
{
	NVME_LYTH_1X,
	NVME_LYTH_1Y,
	NVME_LYTH_1Z,
	NVME_NAND_NUM_LITHOS
} NVME_NAND_DEVICE_LITHOGRAPHY_E;

typedef enum
{
	NVME_NAND_DEV_DENSITY_SLC,
	NVME_NAND_DEV_DENSITY_MLC,
	NVME_NAND_DEV_DENSITY_TLC,
	NVME_NAND_NUM_DENSITIES
} NVME_NAND_DEVICE_DENSITY_E;

typedef enum
{
	NVME_SINGLE_DIMENSION,
	NVME_THREE_DIMENSION,
	NVME_NAND_NUM_DIMENSIONS
} NVME_NAND_DEVICE_TECHNOLOGY_E;

typedef enum {
	NVME_CHIP_S3E,
	NVME_CHIP_S3X,
	NVME_CHIP_NUM_ID
} NVME_IDCMD_ChipID_E;

typedef enum {
	NVME_REV_A,
	NVME_REV_B,
	NVME_REV_C,
	NVME_CHIP_NUM_REV_MAJOR
} NVME_IDCMD_ChipRevMajor_E;

typedef enum {
    B_DIE,
    C_DIE,
    NUM_DIE_TYPES
} NAND_DEVICE_DIE_TYPE_E;

const static char *NVME_VENDOR_STRINGS[NVME_NAND_NUM_VENDORS] =
{
	"Invalid",
	"Hynix",
	"Invalid",
	"Sandisk",
	"Samsung",
	"Toshiba"
};

const static char *NVME_LITHOGRAPHY_STRINGS[NVME_NAND_NUM_LITHOS] =
{
	"1x",
	"1y",
	"1z"
};

const static char *NVME_DENSITY_STRINGS[NVME_NAND_NUM_DENSITIES] =
{
	"SLC",
	"MLC",
	"TLC"
};

const static char *NVME_DIMENSIONS_STRINGS[NVME_NAND_NUM_DIMENSIONS] =
{
	"1D",
	"3D",
};

const static char *NVME_CHIP_STRINGS[NVME_CHIP_NUM_ID] =
{
	"S3E",
	"S1X",
};

const static char *NVME_CHIP_DIE_STRINGS[NUM_DIE_TYPES] =
{
	"b",
	"c"
};


#define NVME_APPLE_CLASS_CODE		(0x018002)
#define NVME_APPLE_BAR0_SIZE		(0x2000)

#define NVME_PAGE_SIZE			(1<<12)
#define NVME_PAGE_MASK			(NVME_PAGE_SIZE - 1)

#define NVME_ADMIN_DELETE_SQ		0x00
#define NVME_ADMIN_CREATE_SQ		0x01
#define NVME_ADMIN_DELETE_CQ		0x04
#define NVME_ADMIN_CREATE_CQ		0x05
#define NVME_ADMIN_IDENTIFY		0x06
#define NVME_ADMIN_SET_FEATURES		0x09
#define NVME_ADMIN_GET_FEATURES		0x0a
#define NVME_ADMIN_FW_ACTIVATE		0x10
#define NVME_ADMIN_FW_DOWNLOAD		0x11

#define NVME_IO_CMD_FLUSH		0x00
#define NVME_IO_CMD_WRITE		0x01
#define NVME_IO_CMD_READ		0x02
#define NVME_IO_CMD_DATASET_MANAGEMENT	0x09

#define NVME_CREATE_SQ_CONTIGUOUS	(1<<0)
#define NVME_CREATE_CQ_CONTIGUOUS	(1<<0)
#define NVME_CREATE_CQ_INTERRUPT_EN	(1<<1)


#define NVME_REG_CAP			(0x0000)
#define NVME_REG_VS			(0x0008)
#define NVME_REG_INTMS			(0x000c)
#define NVME_REG_INTMC			(0x0010)
#define NVME_REG_CC			(0x0014)
#define NVME_REG_CSTS			(0x001c)
#define NVME_REG_AQA			(0x0024)
#define NVME_REG_ASQ			(0x0028)
#define NVME_REG_ACQ			(0x0030)
#define NVME_REG_SQ0TDBL		(0x1000)
#define NVME_REG_MODESEL		(0x1800)
#define NVME_REG_MODESTAT		(0x1804)
#define NVME_REG_DDRREQSIZE		(0x1808)
#define NVME_REG_DDRREQALIGN		(0x180c)
#define NVME_REG_DDRBASE		(0x1810)
#define NVME_REG_DDRSIZE		(0x1818)

// S3E-specific registers
#define NVME_REG_S3E_TIMEOUT_DEBUG	(0x0550)
#define NVME_REG_S3E_ASSERT_ID		(0x1910)
#define NVME_REG_S3E_ASSERT_LOG1	(0x1914)
#define NVME_REG_S3E_ASSERT_LOG2	(0x1918)
#define NVME_REG_S3E_ASSERT_LOG3	(0x191c)
#define NVME_REG_S3E_ASSERT_EDD0	(0x1980)
#define NVME_REG_S3E_ASSERT_EDD1	(0x1984)
#define NVME_REG_S3E_ASSERT_EDD2	(0x1988)
#define NVME_REG_S3E_ASSERT_EDD3	(0x198c)
#define NVME_REG_S3E_ASSERT_EDD4	(0x1990)
#define NVME_REG_S3E_ASSERT_EDD5	(0x1994)
#define NVME_REG_S3E_ASSERT_EDD6	(0x1998)
#define NVME_REG_S3E_ASSERT_EDD7	(0x199c)

// S3E FA registers
#define	NVME_REG_FA_ACTION	(0x195C)
#define NVME_REG_FA_STATUS	(0x1960)
#define	NVME_REG_FA_SIZE	(0x1964)

// S3E FA actions
#define NVME_FA_ACTION_DO_FLUSH		1
#define NVME_FA_ACTION_DO_DUMP_FA	2

// S3E FA status values
#define NVME_FA_STATUS_FLUSH_DONE	1
#define NVME_FA_STATUS_FA_DONE		2

#define NVME_CC_EN			(1 << 0)
#define NVME_CC_MPS(x)			(x << 7)
#define NVME_CC_SHN_NONE		(0 << 14)
#define NVME_CC_SHN_NORMAL		(1 << 14)
#define NVME_CC_SHN_ABRUPT		(2 << 14)
#define NVME_CC_IOSQES(x)		(x << 16)
#define NVME_CC_IOCQES(x)		(x << 20)

#define NVME_CSTS_RDY			(1 << 0)
#define NVME_CSTS_CFS			(1 << 1)
#define NVME_CSTS_SHST_MASK		(3 << 2)
#define NVME_CSTS_SHST_NORMAL		(0 << 2)
#define NVME_CSTS_SHST_PROCESSING	(1 << 2)
#define NVME_CSTS_SHST_DONE		(2 << 2)

#endif // NVME_PROTOCOL_H
