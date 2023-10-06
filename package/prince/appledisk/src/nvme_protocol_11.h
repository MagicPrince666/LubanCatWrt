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

#include <common.h>
#include <linux/types.h>

typedef union nvme_command {
	struct {
		uint8_t		cmd;		// 00(read)-01(prepare data)-02(xx)-03(erase)-04(Write)
		uint8_t		die:4;		// chip die 0 1
		uint8_t     op:4;		// 0:read 8:for write erase
		uint16_t	cid;		// CDW[0] bits 16-31
		uint32_t	page_cnt;	// CDW[1]
		uint32_t	mode;		// 0x20002 SLC MLC etc
		uint16_t	page_idl;	//
		uint16_t	page_idh;	//
		uint32_t    block_id;	//
		uint32_t    s1;			// reserved
		uint32_t    s2;			// reserved
		uint32_t    s3;			// reserved
	};
	uint32_t cdw[8];
} nvme_command_t;

typedef union nvme_completion {
	struct {
		uint32_t	page_attr;
		uint32_t	id1;
		uint32_t	id2;
		uint32_t	meta_check;
	};
	uint32_t cdw[4];
} nvme_completion_t;
//-----------------------------------------------------------------------------------
typedef enum Nvme_Vendor {
	unknow = 0,
	Hynix = 1,		   //DCB "Hynix", 0
	Micron = 2,        //DCB "Micron", 0
	Sandisk = 3,       //DCB "Sandisk", 0
	Samsung = 4,       //DCB "Samsung", 0
	Toshiba = 5,       //DCB "Toshiba", 0
	Intel = 6,        //DCB "Intel", 0
};
//-----------------------------------
typedef struct _nvme_geom_t
{
	char     header[16];
	unsigned char	b10;				//全部相同为1
	unsigned char	b11;				//全部相同为0      只有歪的Hynix0x05
	unsigned short	s12;				//全部相同为0x000c 只有歪的Hynix0x0009
	unsigned int	Interface_revision;	//全部相同 0x14--->0x64=100
	unsigned int	u18;				//全部相同 0x00000000
	unsigned short  s1c;				//全部相同为 0x01F0		0x1c
	unsigned short	pages_per_block_slc;//全部相同为 0x0100		0x1e
	unsigned char	b20;				//全部相同为 0x02 只有Hynix 为 0x04
	unsigned char	b21;				//64G:0x01 128G:0x02 256G:0x04
	unsigned char	die_cnt;			//64G:0x01 128G:0x02 256G:0x04 芯片数量
	unsigned char	b23;				//全部相同为 0x02
	unsigned short  num_bands;			//0x24
	unsigned short  s26;				//0x26  0x0100
	unsigned short  s28;				//0x28	0x0100
	unsigned short  pages_per_block;	//0x2a
	unsigned char	b2c;
	unsigned char	b2d;
	unsigned char	b2e;
	unsigned char	b2f;
	unsigned char	b30[16];			//
	unsigned char	Vendor;				//0x40 1:Hynix  2:Micron 3:Sandisk 4:Samsung 5:Toshiba 6:Intel
	unsigned char	b41;				//0x41	--0x02
	unsigned char	b42;				//0x42	--0x01
	unsigned char	b43;				//0x43  --0x00   but hynix:0x03
	unsigned int    u44;				//0x44  --0x00000001
			 char	MSP_Name[0x40];		//0x48-0x87
			 char   MSP_FW[0x50];		//0x88-0xd7
	unsigned char   unique_id[8];		//0xd8-0xdf
	unsigned char   buf80[8];			//0xe0
			 char   ver[16];			//0xe8-0xF7
	unsigned char   die_id[16][16];		//0xF8-0x1F7
	unsigned short  id1f8;				//
	unsigned char   chip_id[6];			//0x1FA-0x1FF			到了 0x200 512  下面为引导的其他数据
	unsigned char   xxx[0x100];
	unsigned int    syscfg_cmd[2][8];	//0x300-0x31f		//syscfg区的位置命令
	unsigned int    syscfg_cnt;			//
	unsigned int    syscfg_page_pos;	//
	unsigned int    syscfg_page_pos0;	//第一次出现0xFFFFFF09
	unsigned int    syscfg_page_pos1;	//第二次出现0xFFFFFF09
	unsigned char   yyy[0xb0];			//保留的数据
}nvme_geom_t;
//-----------------------------------

#define NVME_APPLE_CLASS_CODE		(0x018002)
#define NVME_APPLE_BAR0_SIZE		(0x2000)

#define NVME_PAGE_SIZE				(1<<12)				//4096bytes/page
#define NVME_PAGE_MASK				(NVME_PAGE_SIZE - 1)

#define NVME_ADMIN_DELETE_SQ		0x00
#define NVME_ADMIN_CREATE_SQ		0x01
#define NVME_ADMIN_DELETE_CQ		0x04
#define NVME_ADMIN_CREATE_CQ		0x05
#define NVME_ADMIN_IDENTIFY			0x06
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
#define NVME_REG_DDRREQALIGN	        (0x180c)
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
#define	NVME_REG_FA_ACTION		(0x195C)
#define NVME_REG_FA_STATUS		(0x1960)
#define	NVME_REG_FA_SIZE		(0x1964)

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
