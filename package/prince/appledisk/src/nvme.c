/*
 * Copyright (C) 2012-2015 Apple Inc. All rights reserved.
 *
 * This document is the property of Apple Inc.
 * It is considered confidential and proprietary.
 *
 * This document may not be reproduced or transmitted in any form,
 * in whole or in part, without the express written permission of
 * Apple Inc.
 */

#ifdef  NVME_DEBUG_LEVEL
#define DEBUG_LEVEL NVME_DEBUG_LEVEL
#endif

#include <debug.h>
#include <drivers/pci.h>
#include <drivers/dart.h>
#include <platform.h>
#include <platform/pcimap.h>
#include <platform/timer.h>
#include <string.h>
#include <sys.h>
#include <sys/menu.h>
#include <lib/env.h>
#include <drivers/nvme.h>
#include <drivers/usb/usb_debug.h>
#include "nvme_blockdev.h"
#include "nvme_protocol.h"

#if WITH_MASS_STORAGE
#define NVME_WITH_NORMAL_MODE		(1)
#endif

#define ADMIN_QUEUE_ID			(0)
#define IO_QUEUE_ID			(1)
#define NUM_QUEUES			(2)

#define MAX_TRANSFER_BLOCKS		(512)
#define QUEUE_ENTRIES			(16)

#if MAX_TRANSFER_BLOCKS * 8 > NVME_PAGE_SIZE
#error "NVMe driver assumes PRP list fits in a single page
#endif

#if !RELEASE_BUILD && WITH_MENU
#define NVME_FW_DOWNLOAD_MIN_LENGTH		(4 * 1024)
#define NVME_FW_ACTIVATE_STATUS_SUCCESS (-267)
#define NVME_FW_TYPE_OPERATIONAL 		0
#define NVME_FW_TYPE_BACKUP 			1
#endif

#define NVME_CONTROLLER_DISABLE_TIMEOUT		(5 * 1000 * 1000)
#define NVME_CONTROLLER_SHUTDOWN_TIMEOUT	(240 * 1000 * 1000)
#define NVME_NORMAL_CONTROLLER_ENABLE_TIMEOUT	(240 * 1000 * 1000)
#define NVME_BOOT_CONTROLLER_ENABLE_TIMEOUT	(5 * 1000 * 1000)
#define NVME_ADMIN_TIMEOUT			(3 * 1000 * 1000)
#define NVME_BOOT_IO_TIME_PER_LBA		(30 * 1000)
#define NVME_NORMAL_IO_TIMEOUT			(30 * 1000 * 1000)

// Eventually these may need to be defined per-platform
#define NVME_IOVM_BOOKKEEPING_BASE	(0x80004000)
#define NVME_IOVM_BOOKKEEPING_PAGES	(8)
#define NVME_IOVM_BOOKKEEPING_SIZE	(NVME_PAGE_SIZE * NVME_IOVM_BOOKKEEPING_PAGES)
#define NVME_IOVM_IO_BASE		(NVME_IOVM_BOOKKEEPING_BASE + NVME_IOVM_BOOKKEEPING_SIZE + NVME_PAGE_SIZE)
#define NVME_IOVM_IO_SIZE		(MAX_TRANSFER_BLOCKS * NVME_PAGE_SIZE)
#define NVME_IOVM_SCRATCH_END		(0xB0000000)

enum NVMeEmbeddedNamespaceIDs
{
	EmbeddedNamespaceTypeNone		= 0,
	EmbeddedNamespaceTypeRoot		= 1,
	EmbeddedNamespaceTypeFirmware		= 2,
	EmbeddedNamespaceTypeSysConfig		= 3,
	EmbeddedNamespaceTypeControlBits	= 4,
	EmbeddedNamespaceTypeNVRAM		= 5,
	EmbeddedNamespaceTypeEffaceable		= 6,
	EmbeddedDeviceTypeCalibration		= 7,
	EmbeddedDeviceTypePanicLog		= 8,
	EmbeddedNamespaceTypeLLB		= 9,

	NumEmbeddedDeviceTypes			= 10
};

typedef struct {
	uint64_t completions_iovm_addr;
	uint32_t head;
	uint32_t tail;
	uint32_t elements;
	uint16_t queue_id;
	uint8_t phase:1;
	bool created;

	// NOTE: All of the data structures after this comment are mapped via the DART,
	//       so software MUST NOT assume values will not change. Values should
	//       be copied into safe locations and then validated prior to use
	nvme_completion_t *completions;
} nvme_cq_t;

typedef struct {
	uint64_t commands_iovm_addr;
	uint32_t head;
	uint32_t tail;
	uint32_t elements;
	uint16_t queue_id;
	bool created;
	nvme_cq_t *cq;

	uint16_t last_command_id;

	// NOTE: All of the data structures after this comment are mapped via the DART,
	//       so software MUST NOT assume values will not change. Values should
	//       be copied into safe locations and then validated prior to use
	nvme_command_t *commands;
} nvme_sq_t;

typedef struct {
	nvme_sq_t sq[NUM_QUEUES];
	nvme_cq_t cq[NUM_QUEUES];

	nvme_sq_t *admin_sq;
	nvme_sq_t *io_sq;

	bool initialized;
	bool identified;
	bool enabled;
#if NVME_WITH_NORMAL_MODE
	uint8_t mode;
#endif
	int id;
	pci_device_t bridge_dev;
	pci_device_t pci_dev;
	uintptr_t reg_base;
	uint32_t doorbell_stride;
	unsigned int dart_id;

	uint16_t vendor_id;
	char serial_number[21];
	char model_number[41];
	char firmware_revision[9];

	uint32_t max_transfer_blocks;
	uint32_t num_namespaces;

	uint64_t prp_list_iovm_addr;

	uintptr_t bookkeeping_base;
	uintptr_t bookkeeping_next;
	uintptr_t bookkeeping_limit;

	uint64_t scratch_iovm_base;
	uint32_t scratch_size;

	int last_error;
	bool cfs;
	// holds Apple-only assertion logging info
	uint32_t assert_data[4];
	// holds Apple-only extended debug data 
	uint32_t edd_data[8];

	// NOTE: All of the data structures after this comment are mapped via the DART,
	//       so software MUST NOT assume values will not change. Values should
	//       be copied into safe locations and then validated prior to use
	uint64_t *prp_list;
	void *identify_buffer;
} nvme_t;

#define NVME_DRIVE_CONFIG_BYTE_OFFSET 3100
#define NVME_GET_BIT_FIELD_VALUE(contents, mask, offset) ( ( contents & mask ) >> offset )
#define NVME_FIELD_MASK(location, length) ( ( ( 1 << length ) - 1 ) << location )

enum
{
	NANDDeviceVendorOffset			= 0,
	NANDDeviceVendorLen				= 3,
	NANDDeviceVendorMask			= NVME_FIELD_MASK ( NANDDeviceVendorOffset, NANDDeviceVendorLen ),
	NANDDeviceLithographyOffset		= 3,
	NANDDeviceLithographyLen		= 4,
	NANDDeviceLithographyMask		= NVME_FIELD_MASK ( NANDDeviceLithographyOffset, NANDDeviceLithographyLen ),
	NANDDeviceDensityOffset			= 7,
	NANDDeviceDensityLen			= 2,
	NANDDeviceDensityMask			= NVME_FIELD_MASK ( NANDDeviceDensityOffset, NANDDeviceDensityLen ),
	NANDDeviceTechnologyOffset		= 9,
	NANDDeviceTechnologyLen			= 3,
	NANDDeviceTechnologyMask		= NVME_FIELD_MASK ( NANDDeviceTechnologyOffset, NANDDeviceTechnologyLen ),
	NANDDeviceNumPlanesOffset		= 12,
	NANDDeviceNumPlanesLen			= 2,
	NANDDeviceNumPlanesMask			= NVME_FIELD_MASK ( NANDDeviceNumPlanesOffset, NANDDeviceNumPlanesLen ),
	NANDDeviceCapacityOffset		= 14,
	NANDDeviceCapacityLen			= 2,
	NANDDeviceCapacityMask			= NVME_FIELD_MASK ( NANDDeviceCapacityOffset, NANDDeviceCapacityLen ),
};

enum
{
	NANDDeviceECCVersionOffset		= 0,
	NANDDeviceECCVersionLen		= 4,
	NANDDeviceECCVersionMask		= NVME_FIELD_MASK ( NANDDeviceECCVersionOffset, NANDDeviceECCVersionLen ),
	NANDDeviceNANDVersionOffset	= 4,
	NANDDeviceNANDVersionLen		= 4,
	NANDDeviceNANDVersionMask		= NVME_FIELD_MASK ( NANDDeviceNANDVersionOffset, NANDDeviceNANDVersionLen ),
};

typedef struct {
	NVME_IDCMD_ChipID_E			ChipHW					: 8; /* S3E/S3X/Etc.		*/
	NVME_IDCMD_ChipRevMajor_E	ChipRevMajor			: 4; /* REV_A/REV_B/Etc.	*/
	uint32_t					ChipRevMinor			: 4; /* 0/1/2/Etc.			*/
	uint16_t					NandDeviceDescriptor;
	uint8_t						MSP_ECC_Version;
	uint8_t						FTL_MajorVersion;
	uint8_t						FTL_MinorVersion;
	uint8_t						DM_Version;
} NVME_IDCMD_DriveConfigAndDBVer_T;

#define SQ_TAIL(_q) ((_q)->commands + (_q)->tail)
#define CQ_HEAD(_q) ((_q)->completions + (_q)->head)

#define ASSERT_QUEUE_NOT_FULL(_q) ASSERT(((_q)->tail + 1) % (_q)->elements != (_q)->head)

static void dump_completion(int log_level, nvme_completion_t *completion);
static void dump_command(int log_level, nvme_command_t *command);
static void dump_submission_queue(int log_level, nvme_sq_t *sq);
static void dump_completion_queue(int log_level, nvme_cq_t *cq);
static void dump_controller_state(int log_level, nvme_t *nvme);

int nvme_update_firmware(int nvme_id, const void *fw_buffer, size_t fw_length);
static int do_nvmefw(int argc, struct cmd_arg *args);

static void nvme_write_reg32(nvme_t *nvme, uint32_t offset, uint32_t value)
{
	dprintf(DEBUG_SPEW, "nvme: set reg 0x%08x = 0x%08x\n", offset, value);
	*(volatile uint32_t *)(nvme->reg_base + offset) = value;
}

static void nvme_write_reg64(nvme_t *nvme, uint32_t offset, uint64_t value)
{
	dprintf(DEBUG_SPEW, "nvme: set reg 0x%08x = 0x%016llx\n", offset, value);
	*(volatile uint32_t *)(nvme->reg_base + offset) = (uint32_t)value;
	*(volatile uint32_t *)(nvme->reg_base + offset + 4) = value >> 32;
}

static bool nvme_check_reg_valid32(uint32_t value)
{
	return value != UINT32_MAX;
}

static bool nvme_check_reg_valid64(uint64_t value)
{
	return value != UINT64_MAX;
}

static uint32_t nvme_read_reg32(nvme_t *nvme, uint32_t offset)
{
	uint32_t value;

	dprintf(DEBUG_NEVER, "nvme: read_reg32 0x%08x", offset);

	value = *(volatile uint32_t *)(nvme->reg_base + offset);

	dprintf(DEBUG_NEVER, " == 0x%08x\n", value);

	return value;
}

static uint64_t nvme_read_reg64(nvme_t *nvme, uint32_t offset)
{
	bool valid = true;
	uint32_t low;
	uint32_t high;
	uint64_t value;

	dprintf(DEBUG_NEVER, "nvme: read_reg64 0x%08x", offset);

	low = *(volatile uint32_t *)(nvme->reg_base + offset);
	valid = nvme_check_reg_valid32(low);

	if (valid) {
		high = *(volatile uint32_t *)(nvme->reg_base + offset + 4);

		valid = nvme_check_reg_valid32(high);
	}

	if (valid) {
		value = ((uint64_t)high << 32) | low;

		dprintf(DEBUG_NEVER, " == 0x%016llx\n", value);

		return value;
	} else {
		dprintf(DEBUG_NEVER, " == invalid\n");
		return UINT64_MAX;
	}
}

static int nvme_poll_reg32(nvme_t *nvme, uint32_t offset, uint32_t required_value, uint32_t timeout_ms)
{
	uint32_t ms_left = timeout_ms;
	while (nvme_read_reg32(nvme, offset) != required_value)	{
		if (ms_left == 0) {
			dprintf(DEBUG_CRITICAL, "nvme: timed out polling register. (offset=0x%x, required_value=0x%x, timeout=%d)\n",
					offset, required_value, timeout_ms);
			return NVME_ERR_POLL_REG_TIMEOUT;
		}
		spin(1000);
		ms_left--;
	}

	return NVME_SUCCESS;
}

static uint64_t nvme_translate_bookkeeping(nvme_t *nvme, void *addr)
{
	uint64_t translated;
	translated = (uintptr_t)addr - nvme->bookkeeping_base;
	translated += NVME_IOVM_BOOKKEEPING_BASE;

	return translated;
}

static void * nvme_get_bookkeeping_pages(nvme_t *nvme, uint32_t num_pages)
{
	ASSERT(num_pages > 0);

	void *ret = (void *)nvme->bookkeeping_next;

	nvme->bookkeeping_next += num_pages * NVME_PAGE_SIZE;

	ASSERT(nvme->bookkeeping_base < nvme->bookkeeping_next);
	ASSERT(nvme->bookkeeping_next < nvme->bookkeeping_limit);

	return ret;
}

static void nvme_update_error(nvme_t *nvme, int status)
{
	if (status != NVME_SUCCESS) {
		uint32_t csts = nvme_read_reg32(nvme, NVME_REG_CSTS);

		if ((csts & NVME_CSTS_CFS) != 0 && !nvme->cfs) {
			nvme->cfs = true;
			// This will only work with S3E, but it's the only NVMe device we support for now
			for (unsigned i = 0; i < 4; i++)
				nvme->assert_data[i] = nvme_read_reg32(nvme, NVME_REG_S3E_ASSERT_ID + i * 4);
			for (unsigned i = 0; i < 8; i++)
				nvme->edd_data[i] = nvme_read_reg32(nvme, NVME_REG_S3E_ASSERT_EDD0 + i * 4);
#if ( ( PRODUCT_IBOOT || PRODUCT_LLB ) && !RELEASE_BUILD )
			panic("nvme: fatal status, assertion data %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
				nvme->assert_data[0], nvme->assert_data[1], nvme->assert_data[2], nvme->assert_data[3],
				nvme->edd_data[0], nvme->edd_data[1], nvme->edd_data[2], nvme->edd_data[3],
				nvme->edd_data[4], nvme->edd_data[5], nvme->edd_data[6], nvme->edd_data[7]);
#else
			dprintf(DEBUG_CRITICAL, "nvme: fatal status, assertion data %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
				nvme->assert_data[0], nvme->assert_data[1], nvme->assert_data[2], nvme->assert_data[3],
				nvme->edd_data[0], nvme->edd_data[1], nvme->edd_data[2], nvme->edd_data[3],
				nvme->edd_data[4], nvme->edd_data[5], nvme->edd_data[6], nvme->edd_data[7]);
#endif
		}

		if (nvme->last_error == NVME_SUCCESS) {
			dprintf(DEBUG_INFO, "nvme: setting error status %d\n", status);
			nvme->last_error = status;
		} else {
			dprintf(DEBUG_INFO, "nvme: double error %d not recorded\n", status);
		}
	}
}

// After some errors, we want to stop sending requests to the device and just fail
// all subsequent API calls. API entry points should check with this function to
// see if such an error has occurred.
static int nvme_get_hard_error(nvme_t *nvme)
{
	if (nvme->cfs) {
		return NVME_ERR_CFS;
	}

	switch (nvme->last_error) {
		case NVME_ERR_ENABLE_TIMEOUT:
		case NVME_ERR_DISABLE_TIMEOUT:
		case NVME_ERR_INVALID_REG:
		case NVME_ERR_ADMIN_CMD_TIMEOUT:
		case NVME_ERR_IO_CMD_TIMEOUT:
			return nvme->last_error;
		default:
			return NVME_SUCCESS;
	}
}

static void nvme_sq_init_cmd(nvme_sq_t *sq, nvme_command_t *cmd)
{
	bzero(cmd, sizeof(*cmd));
	sq->last_command_id = ((sq->last_command_id + 1) & 0xff) | sq->queue_id << 8;
	cmd->cid = sq->last_command_id;
}

static void nvme_sq_add_cmd(nvme_sq_t *sq, nvme_command_t *cmd)
{
	ASSERT_QUEUE_NOT_FULL(sq);

	memcpy((void *)SQ_TAIL(sq), cmd, sizeof(*cmd));

	if (DEBUG_LEVEL >= DEBUG_SPEW) {
		dprintf(DEBUG_SPEW, "nvme: Added command to queue 0x%x at index 0x%x\n", sq->queue_id, sq->tail);
		dump_command(DEBUG_SPEW, cmd);
	}

	sq->tail = sq->tail == (sq->elements - 1) ? 0 : sq->tail + 1;
}

static void nvme_push_admin_cmd_create_cq(nvme_t *nvme, nvme_cq_t *cq)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_CREATE_CQ;
	cmd.prp1 = cq->completions_iovm_addr;
	cmd.cdw[10] = ((cq->elements - 1) << 16) | cq->queue_id;
	cmd.cdw[11] = NVME_CREATE_CQ_CONTIGUOUS;

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_create_sq(nvme_t *nvme, nvme_sq_t *sq)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_CREATE_SQ;
	cmd.prp1 = sq->commands_iovm_addr;
	cmd.cdw[10] = ((sq->elements - 1) << 16) | sq->queue_id;
	cmd.cdw[11] = sq->cq->queue_id << 16 | NVME_CREATE_SQ_CONTIGUOUS;

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_delete_cq(nvme_t *nvme, nvme_cq_t *cq)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_DELETE_CQ;
	cmd.cdw[10] = cq->queue_id;

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_delete_sq(nvme_t *nvme, nvme_sq_t *sq)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_DELETE_SQ;
	cmd.cdw[10] = sq->queue_id;

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_identify_namespace(nvme_t *nvme, uint32_t namespace, uint64_t prp)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_IDENTIFY;
	cmd.nsid = namespace;
	cmd.prp1 = prp;
	cmd.cdw[10] = 0; // identify namespace, not controller

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_identify_controller(nvme_t *nvme, uint64_t prp)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_IDENTIFY;
	cmd.nsid = 0;
	cmd.prp1 = prp;
	cmd.cdw[10] = 1; // identify controller

	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

#if !RELEASE_BUILD && WITH_MENU
static void nvme_push_admin_cmd_firmware_download(nvme_t *nvme, uint64_t *prp_list, uint32_t num_dwords, uint32_t offset, uint8_t fw_type)
{
   	nvme_command_t cmd;

	//setup command
	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_FW_DOWNLOAD;
	cmd.lprp = 1;
	cmd.nsid = 0;
	cmd.prp1 = prp_list[0];
	cmd.cdw[10] = num_dwords - 1;   // Number of DWORDS (total bytes should be multiple of 4KB). Zero-based parameter..
	cmd.cdw[11] = offset;           // Offset (should be multiple of 4KB)
	cmd.cdw[12] = fw_type;

	printf ("nvme_update_firmware: Writing NVMe firmware...\n");
	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}

static void nvme_push_admin_cmd_firmware_activate(nvme_t *nvme, uint8_t fw_type)
{
   	nvme_command_t cmd;

	//setup command
	nvme_sq_init_cmd(nvme->admin_sq, &cmd);

	cmd.opc = NVME_ADMIN_FW_ACTIVATE;
	cmd.cdw[10] = fw_type;

	printf ("nvme_update_firmware: Activating NVMe firmware...\n");
	nvme_sq_add_cmd(nvme->admin_sq, &cmd);
}
#endif

static void nvme_push_io_cmd_read(nvme_t *nvme, uint32_t namespace, uint64_t *prp_list, uint64_t lba, uint16_t count)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->io_sq, &cmd);

	cmd.opc = NVME_IO_CMD_READ;
	cmd.nsid = namespace;
	// LPRP: Apple NVMe extension, says that the command targets a contiguous memory range
	cmd.lprp = 1;
	cmd.prp1 = prp_list[0];
	// Since we set LPRP, we don't need to set PRP2, but we're doing it anyway for
	// compatibility with the DV environment, which doesn't know about LPRP mode
	if (count == 1)
		cmd.prp2 = 0;
	else if (count == 2)
		cmd.prp2 = prp_list[1];
	else
		cmd.prp2 = nvme->prp_list_iovm_addr + sizeof(prp_list[0]);
	cmd.cdw[10] = (uint32_t)lba;
	cmd.cdw[11] = (uint32_t)(lba >> 32);
	cmd.cdw[12] = count - 1;

	nvme_sq_add_cmd(nvme->io_sq, &cmd);
}

#if NVME_WITH_NORMAL_MODE
void nvme_push_io_cmd_write(nvme_t *nvme, uint32_t namespace, uint64_t *prp_list, uint64_t lba, uint16_t count)
{
	nvme_command_t cmd;

	nvme_sq_init_cmd(nvme->io_sq, &cmd);

	cmd.opc = NVME_IO_CMD_WRITE;
	// LPRP: Apple NVMe extension, says that the command targets a contiguous memory range
	cmd.lprp = 1;
	cmd.nsid = namespace;
	cmd.prp1 = prp_list[0];
	// Since we set LPRP, we don't need to set PRP2, but we're doing it anyway for
	// compatibility with the DV environment, which doesn't know about LPRP mode
	if (count == 1)
		cmd.prp2 = 0;
	else if (count == 2)
		cmd.prp2 = prp_list[1];
	else
		cmd.prp2 = (uint64_t)&prp_list[1];
	cmd.cdw[10] = (uint32_t)lba;
	cmd.cdw[11] = (uint32_t)(lba >> 32);
	cmd.cdw[12] = count - 1;

	nvme_sq_add_cmd(nvme->io_sq, &cmd);
}
#endif

static void nvme_init_sq(nvme_t *nvme, int queue_id, void *base, uint32_t elements, int cq_queue_id)
{
	nvme_sq_t *sq = &nvme->sq[queue_id];

	ASSERT(elements * sizeof(nvme_command_t) < NVME_PAGE_SIZE);

	sq->queue_id = queue_id;
	sq->commands = (nvme_command_t *)base;
	sq->elements = elements;
	sq->head = 0;
	sq->tail = 0;
	sq->cq = &nvme->cq[cq_queue_id];

	sq->commands_iovm_addr = nvme_translate_bookkeeping(nvme, base);

	dprintf(DEBUG_SPEW, "nvme: submission queue 0x%x at host addr %p pci addr 0x%08llx\n",
			queue_id, base, sq->commands_iovm_addr);
}

static void nvme_init_cq(nvme_t *nvme, int queue_id, void *base, uint32_t elements)
{
	nvme_cq_t *cq = &nvme->cq[queue_id];

	ASSERT(elements * sizeof(nvme_command_t) < NVME_PAGE_SIZE);

	cq->queue_id = queue_id;
	cq->completions = (nvme_completion_t *)base;
	cq->elements = elements;
	cq->head = 0;
	cq->tail = 0;

	cq->completions_iovm_addr = nvme_translate_bookkeeping(nvme, base);

	dprintf(DEBUG_SPEW, "nvme: completion queue 0x%x at host addr %p pci addr 0x%08llx\n",
			queue_id, base, cq->completions_iovm_addr);
}

static int nvme_wait_cq(nvme_t *nvme, nvme_cq_t *cq, utime_t timeout)
{
	int status;
	utime_t start_time = system_time();
	uint16_t cmpl_sqid;
	uint16_t cmpl_sqhead;
	nvme_completion_t completion;
	nvme_sq_t *sq;
	volatile nvme_completion_t *head;

	head = CQ_HEAD(cq);
	while (head->p == cq->phase) {
		if (time_has_elapsed(start_time, timeout)) {
			dump_completion(DEBUG_SPEW, (nvme_completion_t *)head);
			if (cq->queue_id == ADMIN_QUEUE_ID)
				status = NVME_ERR_ADMIN_CMD_TIMEOUT;
			else
				status = NVME_ERR_IO_CMD_TIMEOUT;

#if (DEBUG_BUILD || DEVELOPMENT_BUILD) && !PRODUCT_IBEC
 			dprintf(DEBUG_INFO, "nvme: command timed out, triggering assertion\n");

			// Grab the 0th dword of the outstanding command to write
			// to the assert register on S3E. This assumes there's always
			// a 1-1 mapping of completion queues to submission queues
			// and that we operate with queue depth 1 (both good assumptions
			// in iBoot)
			nvme_sq_t *sq = &nvme->sq[cq->queue_id];
			uint32_t cmd_dw0 = sq->commands[sq->head].cdw[0];
			// Cause S3E to assert, passing it the failed command to help
			// with debug
			nvme_write_reg32(nvme, NVME_REG_S3E_TIMEOUT_DEBUG, cmd_dw0);
			spin(1000);
			// Read and log the assert ID to our serial console
			nvme_update_error(nvme, status);
			// Dump our submission/completion queues
			platform_memory_barrier();
			dump_controller_state(DEBUG_INFO, nvme);

 			panic("nvme: timeout waiting for completion on CQ %u at index %u\n", cq->queue_id, cq->head);
#else
 			dprintf(DEBUG_INFO, "nvme: timeout waiting for completion on CQ %u at index %u\n", cq->queue_id, cq->head);
#endif

			goto done;
		}
		spin(10);
	}

	platform_memory_barrier();

	// need to copy data out of DMA-visible buffer before validating it
	memcpy(&completion, (void *)head, sizeof(completion));

 	if (DEBUG_LEVEL >= DEBUG_SPEW) {
 		dprintf(DEBUG_SPEW, "nvme: received completion on queue 0x%x at index 0x%x\n",
 				cq->queue_id, cq->head);
 		dump_completion(DEBUG_SPEW, &completion);
 	}

	status = -((head->sct << 8) | head->sc);

	cmpl_sqid = completion.sqid;
	cmpl_sqhead = completion.sqhd;

	if (cmpl_sqid > NUM_QUEUES) {
		status = NVME_ERR_INVALID_CMPL_SQID;
		goto done;
	}
	sq = &nvme->sq[cmpl_sqid];

	if (!sq->created) {
		status = NVME_ERR_INVALID_CMPL_SQID;
		goto done;
	}
	if (cmpl_sqhead > nvme->sq[cmpl_sqid].elements) {
		status = NVME_ERR_INVALID_CMPL_SQHD;
		goto done;
	}

	nvme->sq[cmpl_sqid].head = cmpl_sqhead;

	if (completion.cid != sq->last_command_id) {
		status = NVME_ERR_CMPL_CID_MISMATCH;
		goto done;
	}

done:
	nvme_update_error(nvme, status);
	return status;
}

static void nvme_pop_cq(nvme_t *nvme, nvme_cq_t *cq)
{
	cq->head++;
	if (cq->head == cq->elements) {
		cq->head = 0;
		cq->phase ^= 1;
	}
}

static void nvme_ring_cq_doorbell(nvme_t *nvme, nvme_cq_t *cq)
{
	uint32_t doorbell = cq->queue_id * 2 + 1;
	uint32_t value = cq->head;
	uint32_t offset = NVME_REG_SQ0TDBL + (doorbell * nvme->doorbell_stride);

	nvme_write_reg32(nvme, offset, value);
}

static void nvme_ring_sq_doorbell(nvme_t *nvme, nvme_sq_t *sq)
{
	uint32_t doorbell = sq->queue_id * 2;
	uint32_t value = sq->tail;
	uint32_t offset = NVME_REG_SQ0TDBL + (doorbell * nvme->doorbell_stride);

	ASSERT(sq->created);

	platform_memory_barrier();

	nvme_write_reg32(nvme, offset, value);
}

/* Rings the doorbell for the specified queue, and then waits for the
   command to complete. Currently assumes that only a single command
   is outstanding */
static int nvme_wait_for_cmd(nvme_t *nvme, nvme_sq_t *sq, utime_t timeout)
{
	int status;

	nvme_ring_sq_doorbell(nvme, sq);

	status = nvme_wait_cq(nvme, sq->cq, timeout);

	// On timeout, there's nothing to pop from the CQ
	if (status != NVME_ERR_IO_CMD_TIMEOUT && status != NVME_ERR_ADMIN_CMD_TIMEOUT) {
		nvme_pop_cq(nvme, sq->cq);
		nvme_ring_cq_doorbell(nvme, sq->cq);
	}

	return status;
}

static int nvme_create_sq(nvme_t *nvme, nvme_sq_t *sq)
{
	int status;

	ASSERT(sq->queue_id != ADMIN_QUEUE_ID);
	ASSERT(!sq->created);
	ASSERT(sq->cq->created);

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	bzero(sq->commands, sizeof(*sq->commands) * sq->elements);

	nvme_push_admin_cmd_create_sq(nvme, sq);
	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
	if (status != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: error %d creating SQ%u\n", status, sq->queue_id);
		goto done;
	}

	sq->created = true;

done:
	nvme_update_error(nvme, status);
	return status;
}

static int nvme_create_cq(nvme_t *nvme, nvme_cq_t *cq)
{
	int status;

	ASSERT(cq->queue_id != ADMIN_QUEUE_ID);
	ASSERT(!cq->created);

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	bzero(cq->completions, sizeof(*cq->completions) * cq->elements);

	nvme_push_admin_cmd_create_cq(nvme, cq);
	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
	if (status != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: error %d creating CQ%u\n", status, cq->queue_id);
		goto done;
	}

	cq->created = true;

done:
	nvme_update_error(nvme, status);
	return status;
}

static int nvme_delete_sq(nvme_t *nvme, nvme_sq_t *sq)
{
	int status = NVME_SUCCESS;

	ASSERT(sq->queue_id != ADMIN_QUEUE_ID);
	ASSERT(sq->created);

	// In boot mode, we don't send the delete queue command, but
	// we still want to update our internal status (see done label)
	// so that the switch to normal mode goes smoothly
#if NVME_WITH_NORMAL_MODE
	if (nvme->mode != NVME_MODESEL_BOOT) {
		status = nvme_get_hard_error(nvme);
		if (status != NVME_SUCCESS)
			goto done;

		nvme_push_admin_cmd_delete_sq(nvme, sq);
		status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
		if (status != NVME_SUCCESS) {
			dprintf(DEBUG_CRITICAL, "nvme: error %d deleting SQ%u\n", status, sq->queue_id);
			goto done;
		}
	}
#endif

	goto done; // shuts up compiler for !NVME_WITH_NORMAL_MODE
done:
	sq->created = false;
	sq->head = 0;
	sq->tail = 0;

	nvme_update_error(nvme, status);
	return status;
}

static int nvme_delete_cq(nvme_t *nvme, nvme_cq_t *cq)
{
	int status = NVME_SUCCESS;

	ASSERT(cq->queue_id != ADMIN_QUEUE_ID);
	ASSERT(cq->created);

	// In boot mode, we don't send the delete queue command, but
	// we still want to update our internal status (see done label)
	// so that the switch to normal mode goes smoothly
#if NVME_WITH_NORMAL_MODE
	if (nvme->mode != NVME_MODESEL_BOOT) {
		status = nvme_get_hard_error(nvme);
		if (status != NVME_SUCCESS)
			goto done;

		nvme_push_admin_cmd_delete_cq(nvme, cq);
		status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
		if (status != NVME_SUCCESS) {
			dprintf(DEBUG_CRITICAL, "nvme: error %d deleting CQ%u\n", status, cq->queue_id);
			goto done;
		}
	}
#endif

	goto done; // shuts up compiler for !NVME_WITH_NORMAL_MODE
done:
	cq->created = false;
	cq->head = 0;
	cq->tail = 0;
	cq->phase = 0;

	nvme_update_error(nvme, status);
	return status;
}

static int nvme_init_pci(nvme_t *nvme, pci_device_t bridge)
{
	int status;
	uint32_t class_code;
	uint64_t bar_size;
	uintptr_t bar0;
	pci_device_t pci_dev;

	// PCIe devices are always device 0
	nvme->bridge_dev = bridge;
	pci_dev = pci_bridge_probe(bridge, 0, 100000);
	if (pci_dev == NULL) {
		dprintf(DEBUG_CRITICAL, "nvme: Probe of pci bridge \"%s\" failed.\n", pci_get_name(bridge));
		status = NVME_ERR_PROBE_FAILED;
		goto done;
	}

	class_code = pci_get_class_code(pci_dev);
	if (class_code != NVME_APPLE_CLASS_CODE) {
		dprintf(DEBUG_CRITICAL, "nvme: Found class code 0x%06x, expected 0x%06x\n",
				class_code, NVME_APPLE_CLASS_CODE);
		status = NVME_ERR_INVALID_CLASS_CODE;
		goto done;
	}

	bar_size = pci_get_bar_size(pci_dev, 0);
	if (bar_size < NVME_APPLE_BAR0_SIZE) {
		dprintf(DEBUG_CRITICAL, "nvme: BAR0 (size 0x%llx) not valid\n", bar_size);
		status = NVME_ERR_INVALID_BAR;
		goto done;
	}

#if !SUPPORT_FPGA
#if !APPLICATION_SECUREROM
	pci_enable_pcie_aspm(pci_dev);
#endif
#if !SUB_PLATFORM_T7000
	pci_enable_pcie_ltr(pci_dev);
#if !APPLICATION_SECUREROM
	pci_enable_pcie_l1ss(pci_dev);
#endif
#endif
#endif

	bar0 = pci_map_bar(pci_dev, 0);
	nvme->reg_base = bar0;

	pci_memory_space_enable(pci_dev, true);
	pci_bus_master_enable(pci_dev, true);

	nvme->pci_dev = pci_dev;

	dprintf(DEBUG_SPEW, "nvme: Revision ID 0x%02x\n", pci_get_revision_id(pci_dev));
	dprintf(DEBUG_SPEW, "nvme: BAR0 mapped at host addr 0x%010lx\n", bar0);

	status = NVME_SUCCESS;

done:
	if (status != NVME_SUCCESS && pci_dev != NULL)
		pci_free(pci_dev);

	nvme_update_error(nvme, status);
	return status;
}

// Gathers resources that will remain around across
// boot mode to normal transitions
static int nvme_init_resources(nvme_t *nvme)
{
	nvme->bookkeeping_base = (uintptr_t)memalign(NVME_IOVM_BOOKKEEPING_SIZE, NVME_PAGE_SIZE);
	nvme->bookkeeping_next = nvme->bookkeeping_base;
	nvme->bookkeeping_limit = nvme->bookkeeping_base + NVME_IOVM_BOOKKEEPING_SIZE;

	bzero((void *)nvme->bookkeeping_base, NVME_IOVM_BOOKKEEPING_SIZE);
	dart_map_page_range(nvme->dart_id, nvme->bookkeeping_base,
			NVME_IOVM_BOOKKEEPING_BASE, NVME_IOVM_BOOKKEEPING_PAGES, false);

	dprintf(DEBUG_SPEW, "nvme: bookkeeping region from 0x%016lx to 0x%016lx\n", nvme->bookkeeping_base, nvme->bookkeeping_limit);
	dprintf(DEBUG_SPEW, "nvme: mapped bookkeeping region to 0x%08x\n", NVME_IOVM_BOOKKEEPING_BASE);

	// We assume we only ever have one outstanding transaction, so we only need one
	// command's worth of PRPs, which we round up to a single page of PRPs
	nvme->prp_list = nvme_get_bookkeeping_pages(nvme, 1);
	nvme->prp_list_iovm_addr = nvme_translate_bookkeeping(nvme, nvme->prp_list);
	//dart_write_protect_page_range(nvme->dart_id, nvme->prp_list_iovm_addr, 1, true);

	nvme->identify_buffer = nvme_get_bookkeeping_pages(nvme, 1);

	for (int i = 0; i < NUM_QUEUES; i++ ) {
		nvme_init_cq(nvme, i, nvme_get_bookkeeping_pages(nvme, 1), QUEUE_ENTRIES);
		nvme_init_sq(nvme, i, nvme_get_bookkeeping_pages(nvme, 1), QUEUE_ENTRIES, i);
	}

	nvme->admin_sq = &nvme->sq[ADMIN_QUEUE_ID];
	nvme->io_sq = &nvme->sq[IO_QUEUE_ID];

	return NVME_SUCCESS;
}

#if NVME_WITH_NORMAL_MODE
static int nvme_setup_scratch_buffer(nvme_t *nvme)
{
	static bool scratch_used = false;
	uintptr_t scratch_base;
	size_t scratch_size;
	uint32_t scratch_size_request;
	uint32_t scratch_align_request;
	uint64_t scratch_iovm_base;

	// Slightly hackish, but will do until we really need to support
	// more than one NVMe controller at once
	if (scratch_used)
		panic("NVMe driver only supports scratch buffer for a single device");
	scratch_used = true;

	scratch_base = platform_get_memory_region_base(kMemoryRegion_StorageProcessor);
	scratch_size = platform_get_memory_region_size(kMemoryRegion_StorageProcessor);

	bzero((void *)scratch_base, scratch_size);

	scratch_size_request = nvme_read_reg32(nvme, NVME_REG_DDRREQSIZE);
	if (!nvme_check_reg_valid32(scratch_size_request))
		return NVME_ERR_INVALID_REG;

	scratch_align_request = nvme_read_reg32(nvme, NVME_REG_DDRREQALIGN);
	if (!nvme_check_reg_valid32(scratch_align_request))
		return NVME_ERR_INVALID_REG;

	if (scratch_size_request == 0)
		return NVME_SUCCESS;

	// Round requested size up to the page size, just in case
	if ((scratch_size_request & (NVME_PAGE_SIZE - 1)) != 0)
		scratch_size_request = (scratch_size_request + (NVME_PAGE_SIZE - 1)) & (NVME_PAGE_SIZE - 1);

	// We're going to require the alignment to be a power of 2
	if ((scratch_align_request & (scratch_align_request - 1)) != 0)
		return NVME_ERR_INVALID_DDRREQALIGN;

	// We're also going to require the alignment to be smaller than the size
	// we've allocated for the scratch region
	if (scratch_align_request > scratch_size)
		return NVME_ERR_INVALID_DDRREQALIGN;

	if (scratch_size_request > scratch_size)
		return NVME_ERR_INVALID_DDRREQSIZE;

	// Put everything at the top of the pre-allocated region in PCI address space, respecting
	// the alignment request
	// NOTE: Right now we're giving all of the memory we have available. We could also give
	// just as much as requested, and reclaim the rest of the memory
	// NOTE 2: Assuming NVME_IOVM_SCRATCH_END isn't weirdly aligned (should normally be on a GB boundary)
	scratch_iovm_base = NVME_IOVM_SCRATCH_END - __max(scratch_size, scratch_align_request);

	// Make sure that nothing pathological happened
	if ((scratch_iovm_base & (scratch_align_request - 1)) != 0)
		return NVME_ERR_INVALID_DDRREQALIGN;
	if (scratch_iovm_base < (NVME_IOVM_IO_BASE + NVME_IOVM_IO_SIZE))
		return NVME_ERR_INVALID_DDRREQSIZE;

	dart_map_page_range(nvme->dart_id, scratch_base, scratch_iovm_base, scratch_size / NVME_PAGE_SIZE, false);

	nvme->scratch_iovm_base = scratch_iovm_base;
	nvme->scratch_size = scratch_size;

	dprintf(DEBUG_SPEW, "nvme: scratch host/virt/size %p/0x%llx/0x%zx\n", (void *)scratch_base, scratch_iovm_base, scratch_size);

	nvme_write_reg64(nvme, NVME_REG_DDRBASE, scratch_iovm_base);
	nvme_write_reg32(nvme, NVME_REG_DDRSIZE, scratch_size);

	return NVME_SUCCESS;
}
#endif

static int nvme_set_enable(nvme_t *nvme, bool enable)
{
	utime_t start;
	uint32_t cc;
	uint32_t csts;
	int status;
	bool timeout;

	start = system_time();

	cc = nvme_read_reg32(nvme, NVME_REG_CC);
	if (!nvme_check_reg_valid32(cc)) {
		status = NVME_ERR_INVALID_REG;
		goto done;
	}

	if (enable) {
		cc |= NVME_CC_EN;
	} else {
		cc &= ~NVME_CC_EN;
	}

	nvme_write_reg32(nvme, NVME_REG_CC, cc);

	while (1) {
		if (enable) {
#if NVME_WITH_NORMAL_MODE
			if (nvme->mode != NVME_MODESEL_BOOT)
				timeout = time_has_elapsed(start, NVME_NORMAL_CONTROLLER_ENABLE_TIMEOUT);
			else
#endif
				timeout = time_has_elapsed(start, NVME_BOOT_CONTROLLER_ENABLE_TIMEOUT);
		} else {
			timeout = time_has_elapsed(start, NVME_CONTROLLER_DISABLE_TIMEOUT);
		}

		csts = nvme_read_reg32(nvme, NVME_REG_CSTS);
		if (!nvme_check_reg_valid32(csts)) {
			status = NVME_ERR_INVALID_REG;
			goto done;
		}

		if (enable) {
			if ((csts & NVME_CSTS_RDY) != 0)
				break;

			if (timeout) {
				dprintf(DEBUG_CRITICAL, "nvme: timeout enabling controller\n");
				status = NVME_ERR_ENABLE_TIMEOUT;
				goto done;
			}
		} else {
			if ((csts & NVME_CSTS_RDY) == 0)
				break;

			if (timeout) {
				dprintf(DEBUG_CRITICAL, "nvme: timeout disabling controller\n");
				status = NVME_ERR_DISABLE_TIMEOUT;
				goto done;
			}
		}
		spin(1000);
	}

	status = NVME_SUCCESS;

done:
	nvme_update_error(nvme, status);
	return status;
}

static void nvme_quiesce_pci(nvme_t *nvme)
{
	pci_memory_space_enable(nvme->pci_dev, false);
	pci_bus_master_enable(nvme->pci_dev, false);

	// Place the PCI device into D3-hot
	pci_set_powerstate(nvme->pci_dev, 3);

	pci_free(nvme->pci_dev);

	nvme->pci_dev = NULL;
}

static int nvme_quiesce_controller(nvme_t *nvme)
{
	int status = NVME_SUCCESS;

	if (nvme->enabled) {
		// Before sending the shutdown command, we need to delete any non-admin
		// queues that were created. We can only do this if we didn't previously
		// run into an issue with the admin queue
		// If there was a timeout on an IO queue, it's likely the delete queue
		// command will fail. That's fine, we'll just keep going in that case.
		// If this ever becomes an issue, we can try aborting the outstanding
		// IO command before doing the delete queues commands
		if (nvme->io_sq->created) {
			status = nvme_delete_sq(nvme, nvme->io_sq);
			nvme_update_error(nvme, status);
		}

		// completion queues can't be deleted until after associated
		// submission queue is deleted
		if (nvme->io_sq->cq->created) {
			status = nvme_delete_cq(nvme, nvme->io_sq->cq);
			nvme_update_error(nvme, status);
		}

#if NVME_WITH_NORMAL_MODE
		if ( (nvme->mode != NVME_MODESEL_BOOT) && (nvme_get_hard_error(nvme) == NVME_SUCCESS) ) {
			uint32_t cc;
			uint32_t csts;
			utime_t start;

			start = system_time();

			cc = nvme_read_reg32 (nvme, NVME_REG_CC);
			cc &= ~(NVME_CC_SHN_NORMAL | NVME_CC_SHN_ABRUPT);
			cc |= NVME_CC_SHN_NORMAL;
			nvme_write_reg32(nvme, NVME_REG_CC, cc);

			// Poll for shutdown complete.
			while ((nvme_read_reg32(nvme, NVME_REG_CSTS) & NVME_CSTS_SHST_MASK) != NVME_CSTS_SHST_DONE) {
				if (time_has_elapsed(start, NVME_CONTROLLER_SHUTDOWN_TIMEOUT)) {
					csts = nvme_read_reg32(nvme, NVME_REG_CSTS);
					dprintf(DEBUG_CRITICAL, "nvme: timeout shutting down controller, csts 0x%08x\n", csts);
					status = NVME_ERR_SHUTDOWN_TIMEOUT;
					break;
				}
				spin(1000);
			}
		}
#endif

		nvme_set_enable(nvme, false);

		nvme->enabled = false;
	}

#if NVME_WITH_NORMAL_MODE
	// Reset the mode variable 
	nvme->mode = NVME_MODESEL_MAX;
#endif

	nvme_update_error(nvme, status);
	return status;
}


// Validates that the controller's capabilities are consistent with NVMe for iOS section 6.2
static int nvme_parse_cap(nvme_t *nvme)
{
	int status;
	uint64_t cap;
	uint32_t dstrd;
	uint32_t mpsmin;
	uint32_t mqes;

	cap = nvme_read_reg64(nvme, NVME_REG_CAP);
	if (!nvme_check_reg_valid64(cap)) {
		status = NVME_ERR_INVALID_REG;
		goto error;
	}

	dprintf(DEBUG_SPEW, "nvme: CAP = 0x%016llx\n", cap);

	dstrd = (cap >> 32) & 0xf;
	if (dstrd > 7) {
		status = NVME_ERR_INVALID_DSTRD;
		goto error;
	}
	nvme->doorbell_stride = 1 << (2 + dstrd);

	mpsmin = (cap >> 48) & 0xf;
	if (mpsmin != 0) {
		status = NVME_ERR_INVALID_MPSMIN;
		goto error;
	}

	mqes = (cap >> 0) & 0xffff;
	if (mqes < QUEUE_ENTRIES - 1) {
		status = NVME_ERR_INVALID_MQES;
		goto error;
	}

	return NVME_SUCCESS;

error:
	nvme_update_error(nvme, status);
	dprintf(DEBUG_CRITICAL, "nvme: error %d validating CAP register\n", status);
	return status;
}

static void nvme_identify_build_vendor_specific_string(NVMeIdentifyControllerStruct *identify_result)
{
#if NVME_WITH_NORMAL_MODE

	if (identify_result->S3_CHIP_ID < NVME_CHIP_NUM_ID)
		dprintf(DEBUG_CRITICAL, "nvme: %s ", NVME_CHIP_STRINGS[identify_result->S3_CHIP_ID]);
	else
		dprintf(DEBUG_CRITICAL, "nvme: Model Unknown ");

	dprintf(DEBUG_CRITICAL, "%X ", ( 0xA0 + identify_result->S3_CHIP_REVISION));

	if (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceVendorMask, NANDDeviceVendorOffset) < NVME_NAND_NUM_VENDORS)
		dprintf(DEBUG_CRITICAL, "%s ", NVME_VENDOR_STRINGS[NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceVendorMask, NANDDeviceVendorOffset)]);
	else
		dprintf(DEBUG_CRITICAL, "Vendor Unknown ");

	if (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceLithographyMask, NANDDeviceLithographyOffset) < NVME_NAND_NUM_LITHOS)
		dprintf(DEBUG_CRITICAL, "%s ", NVME_LITHOGRAPHY_STRINGS[NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceLithographyMask, NANDDeviceLithographyOffset)]);
	else
		dprintf(DEBUG_CRITICAL, "Lithography Unknown ");

	if (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceDensityMask, NANDDeviceDensityOffset) < NVME_NAND_NUM_DENSITIES)
		dprintf(DEBUG_CRITICAL, "%s ", NVME_DENSITY_STRINGS[NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceDensityMask, NANDDeviceDensityOffset)]);
	else
		dprintf(DEBUG_CRITICAL, "Density Unknown ");

	if (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceTechnologyMask, NANDDeviceTechnologyOffset) < NVME_NAND_NUM_DIMENSIONS)
		dprintf(DEBUG_CRITICAL, "%s ", NVME_DIMENSIONS_STRINGS[NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceTechnologyMask, NANDDeviceTechnologyOffset)]);
	else
		dprintf(DEBUG_CRITICAL, "Dimension Unknown ");

	dprintf(DEBUG_CRITICAL, "%d plane %dGB ",
				(2 << (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceNumPlanesMask, NANDDeviceNumPlanesOffset) - 1)),
				identify_result->SSD_CAPACITY);

	if (NVME_GET_BIT_FIELD_VALUE(identify_result->S3_DEVICE_DESCRIPTOR, NANDDeviceVendorMask, NANDDeviceVendorOffset) == NVME_NAND_DEVICE_HYNIX)
	{
		dprintf(DEBUG_CRITICAL, "%s Die ", NVME_CHIP_DIE_STRINGS[NVME_GET_BIT_FIELD_VALUE ( identify_result->S3_ECC_VERSION_NAND_REVISION, NANDDeviceNANDVersionMask, NANDDeviceNANDVersionOffset )] );
	}

	dprintf(DEBUG_CRITICAL, "NAND\n" );

#endif
}

static int nvme_identify_controller(nvme_t *nvme)
{
#if NVME_WITH_NORMAL_MODE
	NVMeIdentifyControllerStruct *identify_result = NULL;
	uint64_t iovm_addr;
	uint8_t mdts;
	int status;
	char buildString[kNVMeIdentifyControllerBuildTrainLen+1] = {0};

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	iovm_addr = nvme_translate_bookkeeping(nvme, nvme->identify_buffer);

	nvme_push_admin_cmd_identify_controller(nvme, iovm_addr);
	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
	if (status != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: identify controller failed\n");
		goto done;
	}

	identify_result = malloc(sizeof(*identify_result));
	memcpy(identify_result, nvme->identify_buffer, sizeof(*identify_result));

	nvme->vendor_id = identify_result->PCI_VID;
	memcpy(nvme->serial_number, identify_result->SERIAL_NUMBER, sizeof(nvme->serial_number) - 1);
	memcpy(nvme->model_number, identify_result->MODEL_NUMBER, sizeof(nvme->model_number) - 1);
	memcpy(nvme->firmware_revision, identify_result->FW_REVISION, sizeof(nvme->firmware_revision) - 1);
	nvme->serial_number[sizeof(nvme->serial_number) - 1] = 0;
	nvme->model_number[sizeof(nvme->model_number) - 1] = 0;
	nvme->firmware_revision[sizeof(nvme->firmware_revision) - 1] = 0;

	dprintf(DEBUG_INFO, "nvme: identified controller\n");
	dprintf(DEBUG_INFO, "nvme: Vendor ID:     0x%04x\n", nvme->vendor_id);
	nvme_identify_build_vendor_specific_string(identify_result);
	dprintf(DEBUG_CRITICAL, "nvme: Model Number:  %s\n", nvme->model_number);
	dprintf(DEBUG_CRITICAL, "nvme: Serial Number: %s\n", nvme->serial_number);
	dprintf(DEBUG_CRITICAL, "nvme: Firmware Rev:  %s\n", nvme->firmware_revision);
	dprintf(DEBUG_CRITICAL, "nvme: chip hw/rev:   0x%02x/0x%02x\n", identify_result->S3_CHIP_ID, identify_result->S3_CHIP_REVISION);
	dprintf(DEBUG_CRITICAL, "nvme: nand device:   %04x\n", identify_result->S3_DEVICE_DESCRIPTOR);
	dprintf(DEBUG_CRITICAL, "nvme: FTL ver:       %d.%d\n", identify_result->S3_FTL_MAJOR_VERSION, identify_result->S3_FTL_MINOR_VERSION);
	dprintf(DEBUG_CRITICAL, "nvme: MSP/DM vers:   %d/%d\n", identify_result->S3_ECC_VERSION_NAND_REVISION, identify_result->S3_DM_VERSION);

	// Get the ASP build string from the vendor specific portion of the identify data.
	memcpy(buildString, identify_result->ASP_BUILD_TRAIN, kNVMeIdentifyControllerBuildTrainLen);
	dprintf(DEBUG_CRITICAL, "nvme: ASP_BUILD_TRAIN: %s\n", buildString);

	// Probably not necessary, but avoid artifacts ...
	bzero (buildString, kNVMeIdentifyControllerBuildTrainLen);

	// Get the MSP build string from the vendor specific portion of the identify data.
	memcpy(buildString, identify_result->MSP_BUILD_TRAIN, kNVMeIdentifyControllerBuildTrainLen);
	dprintf(DEBUG_CRITICAL, "nvme: MSP_BUILD_TRAIN: %s\n", buildString);

	if (((identify_result->SQ_ENTRY_SIZE & 0xf) > 6) || ((identify_result->SQ_ENTRY_SIZE >> 4) < 6)) {
		status = NVME_ERR_INVALID_SQES;
		goto done;
	}
	if (((identify_result->CQ_ENTRY_SIZE & 0xf) > 4) || ((identify_result->CQ_ENTRY_SIZE >> 4) < 4)) {
		status = NVME_ERR_INVALID_CQES;
		goto done;
	}

	mdts = identify_result->MAX_DATA_TRANSFER_SIZE;
	if (mdts > 31 || (1 << mdts) > MAX_TRANSFER_BLOCKS)
		nvme->max_transfer_blocks = MAX_TRANSFER_BLOCKS;
	else
		nvme->max_transfer_blocks = 1 << mdts;

	nvme->num_namespaces = identify_result->NUMBER_OF_NAMESPACES;

	status = NVME_SUCCESS;

done:
	if (identify_result != NULL)
		free(identify_result);
	nvme_update_error(nvme, status);
	return status;
#else
	nvme->max_transfer_blocks = NVME_BOOT_MAX_TRANSFER_BLOCKS;
	nvme->num_namespaces = 6;

	return NVME_SUCCESS;
#endif
}

static int nvme_init_controller(nvme_t *nvme, uint8_t mode)
{
	uint32_t cc;
	int status;
	nvme_sq_t *admin_sq;
	nvme_cq_t *admin_cq;

	ASSERT(mode < NVME_MODESEL_MAX);

#if !NVME_WITH_NORMAL_MODE
	ASSERT( mode == NVME_MODESEL_BOOT);
#endif

	status = nvme_set_enable(nvme, false);
	if (status != NVME_SUCCESS)
		goto error;

#if NVME_WITH_NORMAL_MODE
	nvme->mode = mode;

	if (nvme->mode != NVME_MODESEL_BOOT) {
		status = nvme_setup_scratch_buffer(nvme);
		if (status != NVME_SUCCESS)
			goto error;
	}
#endif

	status = nvme_parse_cap(nvme);
	if (status != NVME_SUCCESS)
		goto error;

	admin_sq = nvme->admin_sq;
	admin_cq = admin_sq->cq;

	// clean up internal house-keeping if this is our second time through
	// the init when switching to normal mode
	admin_sq->head = 0;
	admin_sq->tail = 0;
	admin_sq->cq->head = 0;
	admin_sq->cq->tail = 0;
	// and make sure there aren't old commands or completions hanging around
	bzero(admin_sq->commands, sizeof(*admin_sq->commands) * admin_sq->elements);
	bzero(admin_cq->completions, sizeof(*admin_cq->completions) * admin_cq->elements);

	nvme_write_reg32(nvme, NVME_REG_AQA, ((admin_cq->elements - 1) << 16) | (admin_sq->elements - 1));
	nvme_write_reg64(nvme, NVME_REG_ACQ, admin_cq->completions_iovm_addr);
	nvme_write_reg64(nvme, NVME_REG_ASQ, admin_sq->commands_iovm_addr);

	// no explicit command is needed to create the admin queues,
	// other than the writes to the ACQ and ASQ registers
	admin_cq->created = true;
	admin_sq->created = true;

	nvme_write_reg32(nvme, NVME_REG_MODESEL, mode);

	cc = NVME_CC_MPS(0)
	   | NVME_CC_IOSQES(6)
	   | NVME_CC_IOCQES(4);
	nvme_write_reg32(nvme, NVME_REG_CC, cc);

	status = nvme_set_enable(nvme, true);
	if (status != NVME_SUCCESS) {
		goto error;
	}

	nvme->enabled = true;

	if (!nvme->identified) {
		status = nvme_identify_controller(nvme);
		if (status != NVME_SUCCESS) {
			dprintf(DEBUG_CRITICAL, "Error %d on identify controller command", status);
			goto error;
		}
		nvme->identified = true;
	}

	status = nvme_create_cq(nvme, nvme->io_sq->cq);
	if (status != NVME_SUCCESS)
		goto error;

	status = nvme_create_sq(nvme, nvme->io_sq);
	if (status != NVME_SUCCESS)
		goto error;

	return NVME_SUCCESS;

error:
	nvme_quiesce_controller(nvme);

	return status;
}

#if NVME_WITH_NORMAL_MODE
static int nvme_switch_to_mode(nvme_t *nvme, uint8_t mode)
{
	int status;

	ASSERT(mode < NVME_MODESEL_MAX);

	// Allow only these transitions:
	// 1. Boot mode to any other mode
	// 2. Normal mode to normal mode
	// 3. Readonly mode to readonly and normal mode
	if ( (nvme->mode == NVME_MODESEL_BOOT) || ( (nvme->mode == NVME_MODESEL_NORMAL) && (mode == NVME_MODESEL_NORMAL) ) || ((nvme->mode == NVME_MODESEL_READONLY) && ( (mode == NVME_MODESEL_NORMAL) || (mode == NVME_MODESEL_READONLY))) ) {
		dprintf(DEBUG_INFO,"nvme: transition from %02x to %02x mode\n", nvme->mode, mode);
	} else {
		panic("nvme: trying to do transition from %02x to %02x mode\n", nvme->mode, mode);
	}

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;
	
	// Return if already in the new mode
	if (nvme->mode == mode)
		return NVME_SUCCESS;

	// We need to do a whole bunch of teardown, because switching to
	// normal mode involves set CC.EN = 0, which resets a lot state
	// on the controller
	status = nvme_quiesce_controller(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	// Now re-enable in new mode
	status = nvme_init_controller(nvme, mode);
	if (status != NVME_SUCCESS)
		goto done;

	// We just publushed the boot mode bdevs in nvme_init, so
	// publish all other bdevs here. This will also scan for
	// partitions on the mass storage namespace
	if (mode != NVME_MODESEL_RECOVERY)
		nvme_blockdev_init_normal(nvme->id);

done:
	nvme_update_error(nvme, status);

	return status;
}
#endif


static nvme_t nvme_controllers[2];

static nvme_t *get_nvme_controller(int nvme_id)
{

	if ((size_t)nvme_id > sizeof(nvme_controllers) / sizeof(nvme_controllers[0]))
		panic("Invalid NVMe controller ID %d", nvme_id);

	return &nvme_controllers[nvme_id];
}

int nvme_init(int nvme_id, pci_device_t bridge, uint32_t dart_id)
{
	nvme_t *nvme;
	int status;

	dprintf(DEBUG_CRITICAL, "nvme: initializing controller %d\n", nvme_id);

	nvme = get_nvme_controller(nvme_id);

	nvme->id = nvme_id;

	// no backsies
	ASSERT(!nvme->initialized);

	status = nvme_init_pci(nvme, bridge);
	if (status != NVME_SUCCESS)
		goto error;

	nvme->initialized = true;
	nvme->dart_id = dart_id;

	status = nvme_init_resources(nvme);
	if (status != NVME_SUCCESS)
		goto error;

	// start out in boot mode
	status = nvme_init_controller(nvme, NVME_MODESEL_BOOT);
	if (status != NVME_SUCCESS)
		goto error;

	nvme_blockdev_init_boot(nvme_id);

	return NVME_SUCCESS;

error:
	dprintf(DEBUG_INFO, "nvme: init failed with status %d\n", status);
	nvme_quiesce(nvme_id);
	return status;
}

void nvme_quiesce(int nvme_id)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);

	// We might get asked to quiesce without an enable in the case of SecureROM
	// trying to power off the controller even though it wasn't detected on the bus
	if (nvme->pci_dev != NULL) {
		nvme_quiesce_controller(nvme);
		nvme_quiesce_pci(nvme);
		dart_disable_translation(nvme->dart_id);
	}

	if (nvme->bookkeeping_base != 0) {
		dart_unmap_page_range(nvme->dart_id, NVME_IOVM_BOOKKEEPING_BASE, NVME_IOVM_BOOKKEEPING_PAGES);
		free((void *)nvme->bookkeeping_base);
		nvme->bookkeeping_base = 0;
	}

#if NVME_WITH_NORMAL_MODE
	if (nvme->scratch_iovm_base != 0) {
		dart_unmap_page_range(nvme->dart_id, nvme->scratch_iovm_base, nvme->scratch_size / NVME_PAGE_SIZE);
		nvme->scratch_iovm_base = 0;
		nvme->scratch_size = 0;
	}
#endif
}

void nvme_quiesce_all(void)
{
	unsigned int i;
	for (i = 0; i < sizeof(nvme_controllers) / sizeof(nvme_controllers[0]); i++)
		nvme_quiesce(i);
}

#if NVME_WITH_NORMAL_MODE
int nvme_flush_namespace(int nvme_id, uint32_t namespace)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);
	nvme_command_t cmd;
	int status;

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	nvme_sq_init_cmd(nvme->io_sq, &cmd);

	cmd.opc = NVME_IO_CMD_FLUSH;
	cmd.nsid = namespace;
	// LPRP: Apple NVMe extension, says that the command targets a contiguous memory range
	cmd.lprp = 1;

	nvme_sq_add_cmd(nvme->io_sq, &cmd);

	status = nvme_wait_for_cmd(nvme, nvme->io_sq, NVME_NORMAL_IO_TIMEOUT);

done:
	nvme_update_error(nvme, status);
	return status;
}
#endif

static int nvme_do_io(int nvme_id, uint32_t namespace, void *ptr, uint32_t block, uint32_t count, bool write)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);
	int status;
	utime_t timeout;

	RELEASE_ASSERT(((uintptr_t)ptr & (NVME_PAGE_SIZE - 1)) == 0);
	RELEASE_ASSERT(count > 0 && count <= nvme->max_transfer_blocks);

	if (!nvme->enabled) {
		dprintf(DEBUG_CRITICAL, "nvme: controller offline, aborting IO\n");
		return 0;
	}

	if (nvme_get_hard_error(nvme) != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: previous fatal error, aborting IO\n");
		return 0;
	}

	// NOTE: Securing against attackers accessing past the end of the buffer relies on the
	// assumption that the NVMe block size, the NVMe page size, and the DART page size are all equal
	// If this assumption ever changes, nvme_blockdev.c and lib/blockdev.c will need to be audited to
	// make sure that we still secure against this
	dart_map_page_range(nvme->dart_id, (uintptr_t)ptr, NVME_IOVM_IO_BASE, count, write);

	for (uint32_t i = 0; i < count; i++)
		nvme->prp_list[i] = NVME_IOVM_IO_BASE + i * NVME_PAGE_SIZE;


	if (write) {
		// boot mode is read-only
#if NVME_WITH_NORMAL_MODE
		ASSERT(nvme->mode != NVME_MODESEL_BOOT);

		if (namespace != EmbeddedNamespaceTypeRoot)
		{
			status = nvme_flush_namespace(nvme_id, namespace);
			if (status != NVME_SUCCESS)
				goto done;
		}

		nvme_push_io_cmd_write(nvme, namespace, nvme->prp_list, block, count);
#else
		ASSERT(0);
#endif
	} else {
		nvme_push_io_cmd_read(nvme, namespace, nvme->prp_list, block, count);
	}

	// boot mode official timeout is 30 ms per block, we'll pad
	// it a little just to be on the safe side what with it
	// going into ROM
#if NVME_WITH_NORMAL_MODE
	if (nvme->mode != NVME_MODESEL_BOOT)
		timeout = NVME_NORMAL_IO_TIMEOUT;
	else
#endif
		timeout = count * NVME_BOOT_IO_TIME_PER_LBA + (100 * 1000);

	status = nvme_wait_for_cmd(nvme, nvme->io_sq, timeout);
	if (status != NVME_SUCCESS)
		goto done;

#if NVME_WITH_NORMAL_MODE
	if (write && (namespace != EmbeddedNamespaceTypeRoot))
	{
		status = nvme_flush_namespace(nvme_id, namespace);
		if (status != NVME_SUCCESS)
			goto done;

	}
#endif

done:
	dart_unmap_page_range(nvme->dart_id, NVME_IOVM_IO_BASE, count);

	nvme_update_error(nvme, status);

	if (status == NVME_SUCCESS) {
		return count;
	} else {
		dprintf(DEBUG_CRITICAL, "nvme: error %d %s %u blocks at LBA %u of nsid %u\n",
				status, write ? "writing" : "reading", count, block, namespace);
		return 0;
	}
}

int nvme_read_blocks(int nvme_id, uint32_t namespace, void *ptr, uint32_t block, uint32_t count)
{
	return nvme_do_io(nvme_id, namespace, ptr, block, count, false);
}

int nvme_write_blocks(int nvme_id, uint32_t namespace, const void *ptr, uint32_t block, uint32_t count)
{
	return nvme_do_io(nvme_id, namespace, (void *)ptr, block, count, true);
}

#if WITH_DEVICETREE
void nvme_update_devicetree (int nvme_id, dt_node_t *node)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);
	uint8_t value = 0;

	if (nvme == NULL)
	{
		dprintf(DEBUG_CRITICAL, "nvme: unable to find a valid nvme device for id %d\n", nvme_id);
		return;
	}

	if (node == NULL)
	{
		dprintf(DEBUG_CRITICAL, "nvme: s3e node is NULL\n");
		return;
	}

	value = nvme->num_namespaces;

	if (nvme->num_namespaces >= 1)
	{
		dt_set_prop(node, "namespaces-present", (void * )(&value), 1);
		dprintf(DEBUG_CRITICAL, "nvme: setting namespaces-present property %d\n", nvme->num_namespaces);
	}

#if	PRODUCT_IBEC
	value = 1;
	dt_set_prop(node, "ibec-boot", (void * )(&value), 1);
#endif
}
#endif

int nvme_identify_namespace(int nvme_id, uint32_t nsid, nvme_namespace_params_t *params_out)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);
	nvme_identify_namespace_t *identify_result = NULL;
	uint64_t iovm_addr;
	int status;
	int lba_format;
	uint32_t block_size;

	status = nvme_get_hard_error(nvme);
	if (status != NVME_SUCCESS)
		goto done;

	if ( nvme->num_namespaces < nsid )
	{
		dprintf(DEBUG_CRITICAL, "nvme: nsid=%u not present\n", nsid);
		return NVME_ERR_NOT_ENABLED;
	}

	iovm_addr = nvme_translate_bookkeeping(nvme, nvme->identify_buffer);

	nvme_push_admin_cmd_identify_namespace(nvme, nsid, iovm_addr);

	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_ADMIN_TIMEOUT);
	if (status != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: identify nsid=%u failed status=%d\n", nsid, status);
		return status;
	}

	identify_result = malloc(sizeof(*identify_result));
	memcpy(identify_result, nvme->identify_buffer, sizeof(*identify_result));

	dprintf(DEBUG_SPEW, "nvme: identify nsid=%u buffer=%p", nsid, identify_result);

	if (identify_result->ncap == 0 || identify_result->nsze == 0) {
		params_out->formatted = false;
		dprintf(DEBUG_SPEW, " formatted=FALSE\n");
	} else {
		lba_format = identify_result->flbas & 0xf;

		block_size = 1 << identify_result->lbaf[lba_format].lbads;
		// XXX: Obviously this is silly, but there are all
		//      kinds of assumptions about block size == page size right now
		if (block_size != NVME_BLOCK_SIZE) {
			dprintf(DEBUG_SPEW, "\n");
			dprintf(DEBUG_CRITICAL, "nvme: unsupported block size %u in namesapce %u", block_size, nsid);
			params_out->formatted = false;
			goto done;
		}

		if (identify_result->lbaf[lba_format].ms != 0) {
			dprintf(DEBUG_SPEW, "\n");
			dprintf(DEBUG_CRITICAL, "nvme: unsupported metadata size %u in namesapce %u",
					identify_result->lbaf[lba_format].ms, nsid);
			params_out->formatted = false;
			goto done;
		}

		params_out->formatted = true;
		params_out->num_blocks = identify_result->nsze;
		params_out->block_size = block_size;
		dprintf(DEBUG_SPEW, " formatted=TRUE block_size=0x%x blocks=0x%llx\n",
				block_size, identify_result->nsze);
	}

done:
	if (identify_result != NULL)
		free(identify_result);

	// unlike most other commands, identify failures can be non-fatal, for example if
	// a particular namespace isn't present.
	return nvme_get_hard_error(nvme);
}

uint32_t nvme_get_max_transfer_blocks(int nvme_id)
{
	nvme_t *nvme = get_nvme_controller(nvme_id);

	return nvme->max_transfer_blocks;
}

#if NVME_WITH_NORMAL_MODE
int nvme_init_mass_storage(int nvme_id)
{
	int status;
	nvme_t *nvme;

	nvme = get_nvme_controller(nvme_id);

	if (!nvme->enabled) {
		status = NVME_ERR_NOT_ENABLED;
		goto done;
	}

	status = nvme_switch_to_mode(nvme, NVME_MODESEL_READONLY);
	if (status != NVME_SUCCESS)
		goto done;

done:
	return status;
}

int nvme_init_mass_storage_panic(int nvme_id)
{

	int status;
	nvme_t *nvme;

	nvme = get_nvme_controller(nvme_id);

	if (!nvme->enabled) {
		status = NVME_ERR_NOT_ENABLED;
		goto done;
	}

	status = nvme_switch_to_mode(nvme, NVME_MODESEL_RECOVERY);
	if (status != NVME_SUCCESS)
		goto done;

done:

	return status;
}
#endif

#if !RELEASE_BUILD && WITH_MENU
int nvme_update_firmware(int nvme_id, const void *fw_buffer, size_t fw_length)
{
    int err = -1;
    void *fw_buffer_aligned = NULL;
    uint32_t fw_buffer_aligned_length = fw_length;
    uint32_t count = fw_length / NVME_PAGE_SIZE;
    uint32_t num_dwords = 0;
    nvme_t *nvme;
	int status;

	dprintf(DEBUG_INFO, "nvme_update_firmware: fw_buffer %p fw_length %zu count %u\n", fw_buffer, fw_length, count);

	nvme = get_nvme_controller(nvme_id);

	if (!nvme->enabled) {
		dprintf(DEBUG_CRITICAL, "nvme: controller offline, aborting IO\n");
		return -1;
	}

	if (nvme_get_hard_error(nvme) != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: previous fatal error, aborting IO\n");
		return -1;
	}

	// If FW length is not a multiple of 4k then,
	// adjust aligned buffer length to the next higher multiple of 4k
	if (fw_buffer_aligned_length & (NVME_FW_DOWNLOAD_MIN_LENGTH - 1))
	{
		count = ((((fw_buffer_aligned_length / NVME_FW_DOWNLOAD_MIN_LENGTH) + 1) * NVME_FW_DOWNLOAD_MIN_LENGTH) / NVME_PAGE_SIZE);
		fw_buffer_aligned_length = count * NVME_PAGE_SIZE;
		dprintf(DEBUG_INFO, "nvme_update_firmware: fw_length not a multiple of 4KB. adjusting length to %d. num pages is now %d\n", fw_buffer_aligned_length, count);
	}

	if (count > nvme->max_transfer_blocks) {
		dprintf(DEBUG_CRITICAL, "nvme_update_firmware: fw_length exceeds max transfer size. aborting update.\n");
		return -1;
	}

    //allocate buffer
    fw_buffer_aligned = memalign(fw_buffer_aligned_length, NVME_PAGE_SIZE);
    bzero(fw_buffer_aligned,fw_buffer_aligned_length);
    memcpy(fw_buffer_aligned, fw_buffer, fw_length);

	//map buffer
	dart_map_page_range(nvme->dart_id, (uintptr_t)fw_buffer_aligned, NVME_IOVM_IO_BASE, (fw_length / NVME_PAGE_SIZE), true);

	for (uint32_t i = 0; i < count; i++)
		nvme->prp_list[i] = NVME_IOVM_IO_BASE + i * NVME_PAGE_SIZE;

	num_dwords = (fw_buffer_aligned_length/4);

	nvme_push_admin_cmd_firmware_download(nvme, nvme->prp_list, num_dwords, 0, NVME_FW_TYPE_OPERATIONAL);

	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_CONTROLLER_SHUTDOWN_TIMEOUT);

	dart_unmap_page_range(nvme->dart_id, NVME_IOVM_IO_BASE, count);

	nvme_update_error(nvme, status);

	if (status != NVME_SUCCESS) {
		dprintf(DEBUG_CRITICAL, "nvme: error %d downloading firmware to device %d\n", status, nvme_id);
		goto done;
	}

	nvme_push_admin_cmd_firmware_activate(nvme, NVME_FW_TYPE_OPERATIONAL);

	status = nvme_wait_for_cmd(nvme, nvme->admin_sq, NVME_CONTROLLER_SHUTDOWN_TIMEOUT);

	// SCT 0x1 STS 0xB indicates controller reset required successful FW activation.
	if (status == NVME_FW_ACTIVATE_STATUS_SUCCESS) {
		printf("nvme: NVMe FW updated successfully. Reset controller.\n");
		err = 0;
	} else {
		dprintf(DEBUG_CRITICAL, "nvme: error %d activating firmware on device %d\n", status, nvme_id);
		err = status;
	}

done:
    free(fw_buffer_aligned);

    return err;
}

#endif // #if !RELEASE_BUILD && WITH_MENU

#if NVME_BLOCK_SIZE != NVME_PAGE_SIZE
#error "NVMe driver currently assumes that the page size and block size are the same"
#endif


static void dump_completion(int log_level, nvme_completion_t *completion)
{
	dprintf(log_level, "    CID:0x%04x SC:0x%02x SCT:%x P:%x DNR:%x CS:0x%08x",
			completion->cid, completion->sc, completion->sct, completion->p, completion->dnr, completion->cs);
	for(unsigned int i = 0; i < sizeof(*completion)/4; i++) {
		if (i % 4 == 0)
			dprintf(log_level, "\n    DW%02u ", i);
		dprintf(log_level, "%08x ", completion->cdw[i]);
	}
	dprintf(log_level, "\n");
}

static void dump_command(int log_level, nvme_command_t *command)
{
	dprintf(log_level, "    CID:0x%04x OPC:0x%02x NSID:0x%04x PRP1:0x%08llx PRP2:0x%08llx",
			command->cid, command->opc, command->nsid, command->prp1, command->prp2);
	for(unsigned int i = 0; i < sizeof(*command)/4; i++) {
		if (i % 4 == 0)
			dprintf(log_level, "\n    DW%02u ", i);
		dprintf(log_level, "%08x ", command->cdw[i]);
	}
	dprintf(log_level, "\n");
}

#if DEBUG_BUILD || DEVELOPMENT_BUILD
static void dump_submission_queue(int log_level, nvme_sq_t *sq)
{
	if (sq == NULL) return;

	dprintf(log_level, "== Submission Queue 0x%x ==\n", sq->queue_id);
	dprintf(log_level, "Elements:%02x Head:%02x Tail:%02x\n", sq->elements, sq->head, sq->tail);
	dprintf(log_level, "Buffer address 0x%010llx (pci 0x%08llx)\n", (uint64_t)sq->commands, sq->commands_iovm_addr);
	for (unsigned int i = 0; i < sq->elements; i++)
		dump_command(log_level, &sq->commands[i]);
}

static void dump_completion_queue(int log_level, nvme_cq_t *cq)
{
	if (cq == NULL) return;

	dprintf(log_level, "== Completion Queue 0x%x ==\n", cq->queue_id);
	dprintf(log_level, "Elements:%02x Head:%02x Tail:%02x\n", cq->elements, cq->head, cq->tail);
	dprintf(log_level, "Buffer address 0x%010llx (pci 0x%08llx)\n", (uint64_t)cq->completions, cq->completions_iovm_addr);
	for (unsigned int i = 0; i < cq->elements; i++)
		dump_completion(log_level, &cq->completions[i]);
}

static void dump_controller_state(int log_level, nvme_t *nvme)
{
	for (int i = 0; i < NUM_QUEUES; i++) {
		dump_submission_queue(log_level, &nvme->sq[i]);
	}
	for (int i = 0; i < NUM_QUEUES; i++) {
		dump_completion_queue(log_level, &nvme->cq[i]);
	}
}
#endif

#if !RELEASE_BUILD && WITH_MENU
static void print_usage(void)
{
	printf("Usage: \n"
			"1. usb get <path-to-file>\n"
			"2. nvmefw\n"
			"-or-\n"
			"nvmefw <load-address> <filesize-in-bytes>\n");
}

static int do_nvmefw(int argc, struct cmd_arg *args)
{
    addr_t      buf_ptr;
    size_t      buf_len;
    void        *fw_buffer = NULL;
    uint32_t    fw_length;

    if (argc > 3)
    {
        return -1;
    }

    buf_ptr = (argc > 1) ? args[1].u : env_get_uint("loadaddr", DEFAULT_LOAD_ADDRESS);
    buf_len = (argc > 2) ? args[2].u : env_get_uint("filesize", 0);

    if ((buf_len == 0) || (buf_ptr == 0))
    {
    	print_usage();
    	return -1;
    }

    if (!security_allow_memory((void*)buf_ptr, buf_len))
    {
    	printf("do_nvmefw: security_allow_memory failed\n");
        return -2;
    }

    fw_buffer = (void *)buf_ptr;
    fw_length = (uint32_t)buf_len;

    if ((NULL == fw_buffer))
    {
        printf("do_nvmefw: couldn't find the fw payload\n");
        return -3;
    }

    return nvme_update_firmware(0, fw_buffer, fw_length);
}

MENU_COMMAND_DEVELOPMENT(nvmefw, do_nvmefw, "Update NVMe controller firmware", NULL);

static int send_scratchpad_to_file(const nvme_t *nvme, size_t max_size, const char *filename)
{
	uintptr_t scratch_base;
	size_t scratch_size;

	scratch_base = platform_get_memory_region_base(kMemoryRegion_StorageProcessor);
	scratch_size = platform_get_memory_region_size(kMemoryRegion_StorageProcessor);

	if (max_size != 0 && scratch_size > max_size) {
		scratch_size = max_size;
	}

	printf("scratch_base=%p actual_length=%zd filename=%s\n", (void *)scratch_base, scratch_size, filename);

	return usb_send_data_to_file(filename, scratch_size, scratch_base, 0);
}

static int nvme_get_fa(nvme_t *nvme, const char *out_scratch_filename, const char *out_fa_file_name)
{
	int status;
	uint32_t assert_id;
	uint32_t fa_size;

	assert_id = nvme_read_reg32(nvme, NVME_REG_S3E_ASSERT_ID);
	if (!nvme_check_reg_valid32(assert_id) || assert_id == 0) {
		printf("nvmefa error: Drive is not asserted. FA is not available.\n");
		return -1;
	}

	nvme_write_reg32(nvme, NVME_REG_FA_ACTION, NVME_FA_ACTION_DO_FLUSH);
	status = nvme_poll_reg32(nvme, NVME_REG_FA_STATUS, NVME_FA_STATUS_FLUSH_DONE, 5000);
	if (status != NVME_SUCCESS) {
		return status;
	}

	status = send_scratchpad_to_file(nvme, 0, out_scratch_filename);
	if (status != 0) {
		printf("nvmefa error: Failed writing scatchpad contents to file. error=%d\n", status);
		return status;
	}

	nvme_write_reg32(nvme, NVME_REG_FA_ACTION, NVME_FA_ACTION_DO_DUMP_FA);
	status = nvme_poll_reg32(nvme, NVME_REG_FA_STATUS, NVME_FA_STATUS_FA_DONE, 5000);
	if (status != NVME_SUCCESS)
	{
		return status;
	}

	fa_size = nvme_read_reg32(nvme, NVME_REG_FA_SIZE);
	if (!nvme_check_reg_valid32(fa_size) || fa_size == 0) {
		printf("nvmefa error: Invalid FA size (0x%x)\n", fa_size);
		return -1;
	}

	status = send_scratchpad_to_file(nvme, fa_size, out_fa_file_name);
	if (status != 0) {
		printf("nvmefa error: Failed writing FA contents to file. error=%d\n", status);
		return status;
	}

	printf("nvmefa: FA was pulled successfully.\n");
	return 0;
}

static int do_nvme_fa(int argc, struct cmd_arg *args)
{
	nvme_t *nvme = get_nvme_controller(0);
	bool need_pci_quiesce = false;
	int status;

	if (argc != 3)
	{
		printf("nvmefa <output_scratchpad_file> <output_fa_dump_file>\n");
		return -1;
	}

	// If PCI is uninitialized, we need to initialize it ourselves, and quiesce it after we're done.
	if (!nvme->pci_dev) {
		if (!nvme->bridge_dev) {
			printf("nvmefa error: PCI in uninitialized and no bridge device.\n");
			return -1;
		}
		status = nvme_init_pci(nvme, nvme->bridge_dev);
		if (status != 0) {
			return -1;
		}
		need_pci_quiesce = true;
	}

	status = nvme_get_fa(nvme, args[1].str, args[2].str);

	if (need_pci_quiesce) {
		nvme_quiesce_pci(nvme);
	}
	return status;
}

MENU_COMMAND_DEVELOPMENT(nvmefa, do_nvme_fa, "Pull NVMe FA data", NULL);

#endif // #if !RELEASE_BUILD && WITH_MENU
