/*
 * Copyright (C) 2012-2013 Apple Inc. All rights reserved.
 *
 * This document is the property of Apple Inc.
 * It is considered confidential and proprietary.
 *
 * This document may not be reproduced or transmitted in any form,
 * in whole or in part, without the express written permission of
 * Apple Inc.
 */
#include <debug.h>
#include <list.h>
#include <malloc.h>
#include "..//partition//partition.h"
#include "nvme.h"
#include "nvme_blockdev.h"
#include "..//blockdev//blockdev.h"
#include "..//syscfg//syscfg.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct nvme_bdev {
	struct blockdev blockdev;

	int nvme_id;
	uint32_t nsid;

	uint32_t max_transfer_blocks;
	uint32_t dummy_blocks;
	uint32_t blocks_per_virtual_block;

	struct list_node node;
} nvme_bdev_t;
//---------------------------------------------------------------------------------
static struct list_node blockdev_list = LIST_INITIAL_VALUE(blockdev_list);
//---------------------------------------------------------------------------------
static nvme_bdev_t *nvme_bdev_for_blockdev(struct blockdev *blockdev)
{
	nvme_bdev_t *entry;

	list_for_every_entry(&blockdev_list, entry, nvme_bdev_t, node) {
		if (blockdev == &entry->blockdev)
			return entry;
	}

	panic("Unknown block device %p", blockdev);
}

static int nvme_bdev_read(struct blockdev *blockdev, void *ptr, block_addr block, uint32_t count)
{
	nvme_bdev_t *nvme_bdev;
	uint32_t real_count;
	uint32_t blocks_read;
	
	nvme_bdev = nvme_bdev_for_blockdev(blockdev);

	// For special namespaces like NVRAM and SysCfg, we lie about the block size
	block *= nvme_bdev->blocks_per_virtual_block;
	count *= nvme_bdev->blocks_per_virtual_block;

	real_count = __min(count, nvme_bdev->max_transfer_blocks);

	// Hack for <rdar://problem/18591842> Maui platforms not loading syscfg in iBoot
	if (nvme_bdev->dummy_blocks > 0) {
		if (block < nvme_bdev->dummy_blocks) {
			return -1;
		}
		block -= nvme_bdev->dummy_blocks;
	}

	blocks_read = nvme_read_blocks(nvme_bdev->nvme_id, nvme_bdev->nsid, ptr, block, real_count);

	//printf("blocks_read:%d\n",blocks_read);
	//wp_dump_ascii(ptr,2048);

	if ( blocks_read )  {
		blocks_read /= nvme_bdev->blocks_per_virtual_block;
	}

	return blocks_read;
}

static int nvme_bdev_write(struct blockdev *blockdev, const void *ptr, block_addr block, uint32_t count)
{
	nvme_bdev_t *nvme_bdev;
	uint32_t real_count;
	uint32_t blocks_written;
	
	nvme_bdev = nvme_bdev_for_blockdev(blockdev);

	// For special namespaces like NVRAM and SysCfg, we lie about the block size
	block *= nvme_bdev->blocks_per_virtual_block;
	count *= nvme_bdev->blocks_per_virtual_block;

	real_count = __min(count, nvme_bdev->max_transfer_blocks);

	// Hack for <rdar://problem/18591842> Maui platforms not loading syscfg in iBoot
	if (nvme_bdev->dummy_blocks > 0) {
		if (block < nvme_bdev->dummy_blocks) {
			return -1;
		}
		block -= nvme_bdev->dummy_blocks;
	}

	//blocks_written = nvme_write_blocks(nvme_bdev->nvme_id, nvme_bdev->nsid, ptr, block, real_count);

	if ( blocks_written )  {
		blocks_written /= nvme_bdev->blocks_per_virtual_block;
	}

	return blocks_written;
}

static int nvme_bdev_erase(struct blockdev *blockdev, off_t offset, uint64_t length)
{
	nvme_bdev_t *nvme_bdev = NULL;
	uint32_t buffer_length = 0;
	uint32_t count = 0;
	uint32_t real_count = 0;
	uint32_t blocks_written = 0;
	uint8_t *buffer = NULL;
	uint32_t block = 0;

	nvme_bdev = nvme_bdev_for_blockdev(blockdev);

	return 0;

	if((nvme_bdev->nsid != NVME_NAMESPACE_NAND) && (offset > 0))
	{
		dprintf(DEBUG_CRITICAL,"erase should start at offset 0 for nsid %d\n", nvme_bdev->nsid);
		return -1;
	}

	// If offset/length is not a multiple of block size, return error.
	if((offset % (nvme_bdev->blocks_per_virtual_block * NVME_BLOCK_SIZE))
		|| (length % (nvme_bdev->blocks_per_virtual_block * NVME_BLOCK_SIZE)))
	{
		dprintf(DEBUG_CRITICAL,"offset/length is not a multiple of block size %d\n", (nvme_bdev->blocks_per_virtual_block * NVME_BLOCK_SIZE));
		return -1;
	}

	block = offset/(nvme_bdev->blocks_per_virtual_block * NVME_BLOCK_SIZE);

	// For special namespaces like NVRAM and SysCfg, we lie about the block size
	count = length/(nvme_bdev->blocks_per_virtual_block * NVME_BLOCK_SIZE);


	// allocate buffer
	buffer_length = count * NVME_BLOCK_SIZE;
	buffer = memalign(buffer_length, NVME_BLOCK_SIZE);

	real_count = __min(count, nvme_bdev->max_transfer_blocks);

	// Hack for <rdar://problem/18591842> Maui platforms not loading syscfg in iBoot
	if (nvme_bdev->dummy_blocks > 0) {
		if (block < nvme_bdev->dummy_blocks) {
			return -1;
		}
		block -= nvme_bdev->dummy_blocks;
	}

	dprintf(DEBUG_CRITICAL,"nvme: erasing %d blocks from LBA %d for nsid %d\n", real_count, block, nvme_bdev->nsid);
	blocks_written = nvme_write_blocks(nvme_bdev->nvme_id, nvme_bdev->nsid, buffer, block, real_count);

	if ( blocks_written )  {
		blocks_written /= nvme_bdev->blocks_per_virtual_block;
	}

	free(buffer);

	return blocks_written;
}


static nvme_bdev_t *nvme_bdev_create(int nvme_id, uint32_t nsid, uint32_t blocks)
{
	char blockdev_name[16];
	nvme_bdev_t *nvme_bdev;
	int result;
	uint32_t block_size;

	nvme_bdev = calloc(1, sizeof(*nvme_bdev));
	nvme_bdev->nvme_id = nvme_id;
	nvme_bdev->nsid = nsid;

	nvme_bdev->max_transfer_blocks = nvme_get_max_transfer_blocks(nvme_id);
	nvme_bdev->blocks_per_virtual_block = 1;

	switch (nsid) {
		case NVME_NAMESPACE_NAND:
			snprintf(blockdev_name, sizeof(blockdev_name), "nvme_nand%d", nvme_id);
			break;

		case NVME_NAMESPACE_FIRMWARE:
			snprintf(blockdev_name, sizeof(blockdev_name), "nvme_firmware%d", nvme_id);
			break;

		case NVME_NAMESPACE_NVRAM:
			strlcpy(blockdev_name, "nvram", sizeof(blockdev_name));

			// S3E doesn't allow NVRAM partition to be written piecewise, so
			// we lie about the blocksize to upper layers to force writes to
			// go through in a single operation. Otherwise the blockdev deblocking
			// code can generate two writes if the source buffer isn't aligned
			nvme_bdev->blocks_per_virtual_block = 2;
			break;

		case NVME_NAMESPACE_SYSCFG:
			snprintf(blockdev_name, sizeof(blockdev_name), "nvme_syscfg%d", nvme_id);

			// Hack for <rdar://problem/18591842> Maui platforms not loading syscfg in iBoot
			nvme_bdev->blocks_per_virtual_block = 2;
#if(0)																				//here add by wangping
			nvme_bdev->dummy_blocks = kSysCfgBdevOffset / NVME_BLOCK_SIZE;
			blocks += nvme_bdev->dummy_blocks;
#else
			nvme_bdev->dummy_blocks = 0;
#endif
			break;

		case NVME_NAMESPACE_EFFACEABLE:
			snprintf(blockdev_name, sizeof(blockdev_name), "nvme_efface%d", nvme_id);
			break;

		case NVME_NAMESPACE_PANICLOG:
			strlcpy(blockdev_name, "paniclog", sizeof(blockdev_name));
			nvme_bdev->blocks_per_virtual_block = 256; 
			break;

		default:
			panic("Unknown nvme namespace %d", nsid);
	}

	ASSERT(nvme_id < 100);
	ASSERT(sizeof(blockdev_name) == sizeof(nvme_bdev->blockdev.name));

	// For special namespaces like NVRAM and SysCfg, we need to lie about
	// the block size
	blocks /= nvme_bdev->blocks_per_virtual_block;
	block_size = NVME_BLOCK_SIZE * nvme_bdev->blocks_per_virtual_block;

	result = construct_blockdev(&nvme_bdev->blockdev, blockdev_name, (uint64_t)blocks * block_size, block_size);

	if (result != 0){
		printf("Couldn't construct blockdev for namespace %d", nsid);
		free(nvme_bdev);
			return NULL;
	}

	blockdev_set_buffer_alignment(&nvme_bdev->blockdev, NVME_BLOCK_SIZE);

	nvme_bdev->blockdev.read_block_hook = &nvme_bdev_read;

	if (nsid == NVME_NAMESPACE_NVRAM || nsid == NVME_NAMESPACE_PANICLOG)
		nvme_bdev->blockdev.write_block_hook = &nvme_bdev_write;

	if (nsid == NVME_NAMESPACE_FIRMWARE)
		nvme_bdev->blockdev.erase_hook = &nvme_bdev_erase;

	if (register_blockdev(&nvme_bdev->blockdev) != 0) {
		free(nvme_bdev);
		return NULL;
	}

	list_add_tail(&blockdev_list, &nvme_bdev->node);

	return nvme_bdev;
}
//---------------------------------------------------------------------------------------------------
extern const char* string_namespace[];
uint32_t nvme_user_nsid1_length=0;
static nvme_bdev_t *nvme_bdev_identify_and_create(int nvme_id, uint32_t nsid)
{
	nvme_namespace_params_t params;
	int result;

	result = nvme_identify_namespace(nvme_id, nsid, &params);
	if (result != NVME_SUCCESS) {
		dprintf(DEBUG_SPEW, "nvme: error %d trying to identify nsid %u\n", result, nsid);
		dprintf2(DEBUG_SPEW, "nvme: error %d trying to identify nsid %u\n", result, nsid);
		return NULL;
	}
	if (!params.formatted) {
		dprintf(DEBUG_SPEW, "nvme: nsid %u not formatted\n", nsid);
		dprintf2(DEBUG_SPEW, "nvme: nsid %u not formatted\n", nsid);
		return NULL;
	}

	//dprintf(1, "nvme: Creating blkdev for NSID %s num_blocks %08x\n", string_namespace[nsid], (unsigned int)params.num_blocks);
	//dprintf2(1, "nvme: %s num_blocks %08x\n", string_namespace[nsid], (unsigned int)params.num_blocks);

	dprintf(1, "nvme: %s blocks:0x%08x size:0x%010x\n", string_namespace[nsid], (unsigned int)params.num_blocks,(unsigned long)params.num_blocks*params.block_size);
	dprintf2(1, "nvme: %s blocks:0x%08x size:0x%010x\n", string_namespace[nsid], (unsigned int)params.num_blocks,(unsigned long)params.num_blocks*params.block_size);

	//dprintf(1, "---size:0x%010x\n",((unsigned long)params.num_blocks*params.block_size)/0x10000);

	if(nsid == 1) nvme_user_nsid1_length = (unsigned int)params.num_blocks; //保存nsid1长度

	return nvme_bdev_create(nvme_id, nsid, params.num_blocks);

}
//---------------------------------------------------------------------------------------------------
#define INT_MAX		((int)(~0U>>1))
extern uint8_t nvme_user_init_on;
int nvme_blockdev_init_boot(int nvme_id)
{
	// In boot mode, we can't determine the size of the namespace ahead of time,
	// so it's up to callers to not read past the end of the device, and the NVMe
	// driver to return errors as needed
	// These other ones are only in iBoot/iBEC, where we're less concerned about speed
	// and thus can do the identify step

	list_initialize(&blockdev_list);
	printf("------------------------------------------------------------------------------\n");
	printf2("-------------------------------------------------------\n");

	if(nvme_user_init_on==0x11) nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_NAND);

	nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_FIRMWARE);

	nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_SYSCFG);

	nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_NVRAM);

	nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_EFFACEABLE);

	nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_PANICLOG);
	printf("------------------------------------------------------------------------------\n");
	printf2("-------------------------------------------------------\n");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------
int nvme_blockdev_init_normal(int nvme_id)
{
	nvme_bdev_t *nvme_nand_bdev;
	list_initialize(&blockdev_list);
	nvme_nand_bdev = nvme_bdev_identify_and_create(nvme_id, NVME_NAMESPACE_NAND);
	if (nvme_nand_bdev != NULL) {
		const char *name = nvme_nand_bdev->blockdev.name;
		return partition_scan_and_publish_subdevices(name);
	} else {
		printf("nvme_bdev_identify_and_create <<NVME_NAMESPACE_NAND>> failed!\n");
		return -1;
	}
}
//-------------------------------------------------------------------------------------------------------------
int nvme_blockdev_remove(int nvme_id)
{
	nvme_bdev_t *entry;
	while(1){
		list_for_every_entry(&blockdev_list, entry, nvme_bdev_t, node) {
			if(entry){
				list_remove_head(entry);
				free(entry);
			}
		}
		break;
	}
	printf("free all nvme_block_devs!\n");
	return 0;
}
//-------------------------------------------------------------------------------------------------------------

