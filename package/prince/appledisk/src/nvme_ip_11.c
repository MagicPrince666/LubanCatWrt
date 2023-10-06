
#include <debug.h>
#include <common.h>
#include <malloc.h>
#include "debug.h"
#include "asm/io.h"
#include "configs/rt2880.h"
#include "rt_mmap.h"

#include "..//pcie//pci.h"
#include "..//pcie//pci_private.h"
#include "..//pcie//apcie.h"
#include "..//blockdev//algo_7688.h"

#include "nvme_protocol_11.h"
#include "nvme.h"

#ifndef uint64_t
#define uint64_t long long
#endif

unsigned int NVME_IOVM_BOOKKEEPING_BASE;
#define NVME_IOVM_BOOKKEEPING_PAGES			(10)
#define NVME_IOVM_BOOKKEEPING_SIZE			(NVME_PAGE_SIZE * NVME_IOVM_BOOKKEEPING_PAGES)
#define NVME_IOVM_IO_BASE					(NVME_IOVM_BOOKKEEPING_BASE + NVME_IOVM_BOOKKEEPING_SIZE + NVME_PAGE_SIZE)
#define NVME_IOVM_IO_SIZE					(MAX_TRANSFER_BLOCKS * NVME_PAGE_SIZE)
#define NVME_IOVM_SCRATCH_END				(0x88000000)
#define Delay_ms(ms)	mdelay(ms)
//-------------------------------------------------------------------------------------------------------------------
typedef struct {
	uint8_t* 	page_buf;		//4k*8
	uint32_t	page_ecc[8][4];
}page_data_t;

typedef struct {
	bool 			initialized;
	bool 			enabled;
	uint8_t 		mode;
	uint8_t 		resev8;
	pci_device_t    bridge_dev;
	pci_device_t    pci_dev;
	uintptr_t       reg_base;

	uintptr_t 		bookkeeping_base;
	uintptr_t 		bookkeeping_next;
	uintptr_t 		bookkeeping_limit;

	uintptr_t  		bar2_base_addr;		        	//added by wp 2020
	uintptr_t  		bar2_completion_base;           //03f33000
	uint32_t   		bar2_completion_ptr;
	uint64_t 		prp_list_iovm_addr1;
	uint64_t 		prp_list_iovm_addr2;

	page_data_t*	page_data;
	nvme_geom_t		nvme_geom;

	uint32_t		syscfg_data32[32][1024];
} nvme_t;

static nvme_t nvme_controllers = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static nvme_t *nvme;
static unsigned int g_InitCmd[4] = {0x0};
//-------------------------------------------------------------------------------------------------------------------
int	echo = 0;
int nvme_reconnect(void);
unsigned int buf_get4(unsigned char* buf){return (buf[0]+buf[1]*0x100+buf[2]*0x10000+buf[3]*0x1000000);}
unsigned int buf_get4no(unsigned char* buf){return (buf[3]+buf[2]*0x100+buf[1]*0x10000+buf[0]*0x1000000);}
void buf_put4(unsigned char* buf,unsigned int data){buf[0]=data&0xff;buf[1]=(data>>8)&0xff;buf[2]=(data>>16)&0xff;buf[3]=(data>>24)&0xff;}

extern int P11_reset(void);
extern int ecc_vertify_page(unsigned char* page_buffer,unsigned char* meta_buffer);

extern uint8_t   P7_INFO_BUF[0X800]; //info
extern uint16_t  P7_INFO_length;
extern uint8_t   syscfg_buf[0x20000];//128K
extern uint8_t   p11_page_buf[0x4000];//4k
extern uint32_t  p11_all_buf_syscfg_offset_address;
extern uint8_t   p11_all_buf[0x800000];//1m
extern uint8_t   p11_all_buf_ecc[0x8000];

uint32_t P11_page_step=0;
//-------------------------------------------------------------------------------------------------------------------
unsigned short  check_snm_16(unsigned char * buf, int length)
{
	int i;
	unsigned short check = 0;
	for (i = 0; i < length; i++) check += buf[i];

	return check;

}
//-------------------------------------------------------------------------------------------------------------------
static void nvme_write_reg32(uint32_t offset, uint32_t value)
{
	dprintf(DEBUG_SPEW, "nvme: set reg 0x%08x = 0x%08x\n", offset, value);
	*(volatile uint32_t *)(nvme->reg_base + offset) = value;
}
static void nvme_write_reg64(uint32_t offset, uint64_t value)
{
	dprintf(DEBUG_SPEW, "nvme: set reg 0x%08x = 0x%016llx\n", offset, value);
	*(volatile uint32_t *)(nvme->reg_base + offset) = (uint32_t)value;
	*(volatile uint32_t *)(nvme->reg_base + offset + 4) = value >> 32;
}
static uint32_t nvme_read_reg32(uint32_t offset)
{
	uint32_t value;
	dprintf(DEBUG_NEVER, "nvme: read_reg32 0x%08x", offset);
	value = *(volatile uint32_t *)(nvme->reg_base + offset);
	dprintf(DEBUG_NEVER, " == 0x%08x\n", value);
	return value;
}
static uint64_t nvme_translate_bookkeeping(void *addr)
{
	uint64_t translated;
	translated = (uintptr_t)addr - nvme->bookkeeping_base;
	translated += NVME_IOVM_BOOKKEEPING_BASE;
	return translated;
}
static void * nvme_get_bookkeeping_pages(uint32_t num_pages)
{
	void *ret = (void *)nvme->bookkeeping_next;
	nvme->bookkeeping_next += num_pages * NVME_PAGE_SIZE;
	return ret;
}
//------------------------------------------------------------------------------------------------------------------------------
static int nvme_init_pci2(void)
{
	uint32_t v32;
	int status = NVME_SUCCESS;
	pci_device_t pci_dev;
	pci_dev = nvme->pci_dev;
	//----Step7                     L1 PM Substates Control
#if(1)
	pci_device_enable_pcie_l1ss(pci_dev);
#else
	v32 = pci_config_read32(pci_dev, 0x188);
	if(echo)printf("PCIe[0x188]RD:%08x\n",v32);
	pci_config_write32(pci_dev, 0x188,0x00000000);

	v32 = pci_config_read32(pci_dev, 0x18C);
	if(echo)printf("PCIe[0x18C]RD:%08x\n",v32);
	pci_config_write32(pci_dev, 0x18C,0x00000028);
	if(echo)printf("PCIe[0x18C]WR:%08x\n",pci_config_read32(pci_dev, 0x18C));

	pci_config_write32(pci_dev, 0x188,0x4055000F);
#endif
	//
	v32 = pci_config_read32(pci_dev, 0x7C);
	if(echo)printf("PCIe[0x7C]RD:%08x\n",v32);

	v32 = pci_config_read32(pci_dev, 0x80);         //Active State PM Control = 0x02
	if(echo)printf("PCIe[0x80]RD:%08x\n",v32);
	pci_config_write32(pci_dev, 0x80,0x0002);

	v32 = pci_config_read32(pci_dev, PCI_CONFIG_COMMAND);
	if(echo)printf("PCIe[0x04]RD:%08x\n",v32);
	pci_config_write32(pci_dev, PCI_CONFIG_COMMAND, (v32|0x0406)&0xffff);                   //Interrupt Disable

	v32 = pci_config_read32(pci_dev, 0x104);
	if(echo)printf("PCIe[0x104]RD:%08x\n",v32);
	pci_config_write32(pci_dev, 0x104,0x00000000);

	v32 = pci_config_read32(pci_dev, 0x110);
	if(echo)printf("PCIe[0x110]RD:%08x\n",v32);
	pci_config_write32(pci_dev, 0x110,0x00000000);

	v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL);                                     //v32=0x2810-0010       add by wp 2020
	if(echo)printf("PCIe[0x78]RD:%08x\n",v32);

	pci_config_write32(pci_dev,PCI_CONFIG_DEVICE_CONTROL,(v32|0x000f)&0xffff);
	if(echo)printf("PCIe[0x78]RD:%08x\n",pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL));

	pci_config_write32(pci_dev, PCI_CONFIG_COMMAND, 0x0406);
	return status;
}
//------------------------------------------------------------------------------------------------------------------------------
static int nvme_init_pci( pci_device_t port_bridge)
{
	int status;
	uint32_t class_code,v32,idPos;
	uint64_t bar_size;
	uintptr_t bar0,bar2;
	pci_device_t pci_dev;

	// PCIe devices are always device 0
	nvme->bridge_dev = port_bridge;
	pci_dev = pci_port_bridge_probe(port_bridge, 0, 100000);//100000
	if (pci_dev == NULL) {
		dprintf(50, "nvme: Probe of pci bridge \"%s\" failed.\n", pci_get_name(port_bridge));
		status = NVME_ERR_PROBE_FAILED;
		goto done;
	}

	class_code = pci_get_class_code(pci_dev);
	if (class_code != NVME_APPLE_CLASS_CODE) {
		dprintf(50, "nvme: Found class code 0x%06x, expected 0x%06x\n",class_code, NVME_APPLE_CLASS_CODE);
		status = NVME_ERR_INVALID_CLASS_CODE;
		goto done;
	}

	bar_size = pci_get_bar_size(pci_dev, 0);
	if (bar_size < NVME_APPLE_BAR0_SIZE) {
		dprintf(50, "nvme: BAR0 (size 0x%llx) not valid\n", bar_size);
		status = NVME_ERR_INVALID_BAR;
		goto done;
	}
	//pci_enable_pcie_aspm(pci_dev);
	//pci_enable_pcie_ltr(pci_dev);

	bar0 = pci_map_bar(pci_dev, 0);
	nvme->reg_base = bar0;                                  //0x20000000

	bar2 = pci_map_bar(pci_dev, 2);
	nvme->bar2_base_addr = bar2;                            //0x20004000
	//----Step2--split Tra(38)
	/*
	v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL);					//v32=0x2810-0010	add by wp 2020
	//printf("PCIe[0x78]RD:%08x\n",v32);
	pci_config_write32(pci_dev,PCI_CONFIG_DEVICE_CONTROL,(v32|0x1000)&0xffff);

	v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CAPABILITIES);
	//printf("PCIe[0x74]RD:%08x\n",v32);                                        //0x10008B42

	v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL);					//v32=0x2810-0010
	//printf("PCIe[0x78]RD:%08x\n",v32);
	pci_config_write32(pci_dev,PCI_CONFIG_DEVICE_CONTROL,(v32|0x20)&0xffff);
	*/
	v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CAPABILITIES2);
	//printf("PCIe[0x94]RD:%08x\n",v32);                                         //0x0000081F

	idPos = pci_find_extended_capability(pci_dev, 0x18, 1);
	//printf("PCIe ext<id:0x18>---reg:%X\n",idPos);

        v32 = pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL2);                   //v32=0x2810-0010
        //printf("PCIe[0x98]RD:%08x\n",v32);
        pci_config_write32(pci_dev,PCI_CONFIG_DEVICE_CONTROL2,v32|0x40f);              //Enabe LTR Mechanism Enable ???
        //printf("PCIe[0x98]RD:%08x\n",pci_config_read32(pci_dev,PCI_CONFIG_DEVICE_CONTROL2));

        v32 = pci_config_read32(pci_dev,idPos+4);              //idPos+4
        //printf("PCIe[0x%X]RD:%08x\n",idPos+4,v32);
        pci_config_write32(pci_dev,idPos+4,0x30180000);
        //printf("PCIe[0x17C]RD:%08x\n",pci_config_read32(pci_dev,idPos+4));
	//----Step3
	//pci_memory_space_enable(pci_dev, true);		//0x20
	//pci_bus_master_enable(pci_dev, true);			//0x40
	v32 = pci_config_read32(pci_dev, PCI_CONFIG_COMMAND);
	//printf("PCIe[0x04]RD:%08x\n",v32);

	pci_config_write32(pci_dev, PCI_CONFIG_COMMAND, (v32|PCI_CMD_MEMORY_SPACE_ENABLE)&0xffff);
	v32 = pci_config_read32(pci_dev, PCI_CONFIG_COMMAND);
	//printf("PCIe[0x04]RD:%08x\n",v32);
	pci_config_write32(pci_dev, PCI_CONFIG_COMMAND, (v32|0x0400)&0xffff);			//Interrupt Disable
	//printf("PCIe[0x04]RD:%08x\n",pci_config_read32(pci_dev, PCI_CONFIG_COMMAND));

	//----Step4
	nvme->pci_dev = pci_dev;
	status = NVME_SUCCESS;
	//----Step5
	v32 = nvme_read_reg32(NVME_REG_CAP);			//(0x0000))
	if(echo)printf("NVMe[0x0000]RD:%08x\n",v32);
	return status;
done:
	if (status != NVME_SUCCESS && pci_dev != NULL)
	  pci_free(pci_dev);
	return status;
}
//------------------------------------------------------------------------------------------------------------------------------
// Gathers resources that will remain around across
// boot mode to normal transitions
static int nvme_init_resources(void)
{
	if (nvme->bookkeeping_base != 0){
	    memset((void *)nvme->bookkeeping_base, 0xA5,NVME_IOVM_BOOKKEEPING_SIZE);
	    return NVME_SUCCESS;
	}
	nvme->bookkeeping_base = (uintptr_t)memalign(NVME_IOVM_BOOKKEEPING_SIZE, NVME_PAGE_SIZE);
	nvme->bookkeeping_next = nvme->bookkeeping_base;
	nvme->bookkeeping_limit = nvme->bookkeeping_base + NVME_IOVM_BOOKKEEPING_SIZE;

	bzero((void *)nvme->bookkeeping_base, NVME_IOVM_BOOKKEEPING_SIZE);

	NVME_IOVM_BOOKKEEPING_BASE = (unsigned int)virt_to_phys((volatile void *)nvme->bookkeeping_base);								//added by wangping

	if(echo)dprintf(1, "NVMe->bookkeeping_base: 0x%016lx\nNVMe->bookkeeping_limit: 0x%016lx\n", nvme->bookkeeping_base, nvme->bookkeeping_limit);
	if(echo)dprintf(1, "NVMe_IOVM_BOOKKEEPING_BASE: 0x%08x\n", NVME_IOVM_BOOKKEEPING_BASE);

	// We assume we only ever have one outstanding transaction, so we only need one
	// command's worth of PRPs, which we round up to a single page of PRPs
	nvme->prp_list_iovm_addr1 = nvme_translate_bookkeeping( nvme_get_bookkeeping_pages( 8));
	if(echo)printf("nvme->prp_list_iovm_addr1:0x%016lx\n",(uintptr_t)nvme->prp_list_iovm_addr1);

	nvme->prp_list_iovm_addr2 =  nvme_translate_bookkeeping( nvme_get_bookkeeping_pages( 1));      //NVMe控制命令完成数据校验队列 16Bytes对应一个 等效于(0x80000000----)
	if(echo)printf("nvme->prp_list_iovm_addr2:0x%016lx\n",(uintptr_t)nvme->prp_list_iovm_addr2);

	nvme->bar2_completion_base = nvme_translate_bookkeeping(nvme_get_bookkeeping_pages( 1));       //NVMe控制命令完成队列 8Bytes对应一个 等效于(0x7ff02000----)
	if(echo)dprintf(1, "bar2_completion_base: 0x%016lx \n", (uintptr_t)nvme->bar2_completion_base);
	nvme->bar2_completion_ptr = 0;
	//---------------------------------------
	nvme->page_data = 0;
	nvme->page_data = malloc(sizeof(page_data_t));
	if(nvme->page_data == 0){
		printf("[ERROR]malloc page_data failed!\n");
		return -1;
	}

	nvme->page_data->page_buf = p11_page_buf;//syscfg_buf;

	//---------------------------------------
	return NVME_SUCCESS;
}
//------------------------------------------------------------------------------------------------------------------------------
static int nvme_init_controller(void)
{
	uint32_t v32,a32,b32;
	if(echo)printf("start to setup nvme map address to pcie\n");

	nvme_write_reg64(0x200,nvme->bar2_completion_base);        	//Bar2 命令返回的状态基地址  0x03f34000
	nvme_write_reg32(0x208,0x00000200);		        			//0x200*8 = 0x1000(4K)
	nvme->bar2_completion_ptr = 0;

	a32 = nvme_read_reg32(0x200);        b32 = nvme_read_reg32(0x204);
	if(echo)printf("NVMe[0x0200]RD:%08x %08x\n",a32,b32);

	v32 = nvme_read_reg32(NVME_REG_CAP);                       //[nvme:00] = 0x00000001
	if(echo)printf("NVME[0x0000]RD:%08x\n",v32);
	v32 = nvme_read_reg32(0x0A40);                             //[nvme:A40] = 0x00000000
	if(echo)printf("NVME[0x0A40]RD:%08x\n",v32);
	v32 = nvme_read_reg32(0x0900);                             //[nvme:900] = 0x000001f0
	if(echo)printf("NVME[0x0900]RD:%08x\n",v32);
	//----Step6
	nvme_write_reg32(0x600,0x00000001);        //I don't know what meaning?
	nvme_write_reg32(0xA20,0x00000000);        //I don't know what meaning?

	nvme_write_reg64(0xA00,nvme->prp_list_iovm_addr1);         //NVMe read buffer address		是虚地址千万不能加|0x80000000     0x00000000
	a32 = nvme_read_reg32(0xA00);        b32 = nvme_read_reg32(0xA04);
	if(echo)printf("NVMe[0x0A00]RD:%08x %08x\n",a32,b32);

	nvme_write_reg64(0xA08,nvme->prp_list_iovm_addr2);         //NVMe bar2 completions queue  是虚地址千万不能加|0x80000000
	a32 = nvme_read_reg32(0xA08);        b32 = nvme_read_reg32(0xA0C);
	if(echo)printf("NVMe[0x0A08]RD:%08x %08x\n",a32,b32);

	nvme_write_reg64(0x500,0x00000000);                        //this is not address register  but don't know???
	a32 = nvme_read_reg32(0x500);        b32 = nvme_read_reg32(0x504);
	if(echo)printf("NVMe[0x0500]RD:%08x %08x\n",a32,b32);

	return NVME_SUCCESS;
}
//----------------------------------- ------------------------------------------------------------------------------
static void nvme_quiesce_pci(void)
{
	pci_memory_space_enable(nvme->pci_dev, false);
	pci_bus_master_enable(nvme->pci_dev, false);
	// Place the PCI device into D3-hot
	pci_set_powerstate(nvme->pci_dev, 3);
	pci_free(nvme->pci_dev);
	nvme->pci_dev = NULL;
}

//-----------------------------------------------------------------------------------------------------------------
static int nvme_quiesce_controller(void)
{
	int status = NVME_SUCCESS;
	if (nvme->enabled) {
		nvme->enabled = false;
	}
	return status;
}
//-----------------------------------------------------------------------------------------------------------------
static uint32_t ack_h = 0,ack_l = 0;
uint32_t nvme_ack_read_low(void){
	ack_l = (*(volatile unsigned int *)(nvme->bar2_completion_base + (nvme->bar2_completion_ptr&0x1ff)*8));
	return ack_l;
}
uint32_t nvme_ack_read_high(void){
	ack_h = (*(volatile unsigned int *)(nvme->bar2_completion_base + (nvme->bar2_completion_ptr&0x1ff)*8 + 4));
	return ack_h;
}
void nvme_ackpoint_inc(void){
	if((ack_l != 0xffffffff)&&(ack_h != 0xffffffff)){
		nvme->bar2_completion_ptr++;
		if((nvme->bar2_completion_ptr)>=200){
			nvme->bar2_completion_ptr = 0;
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_cmd_process(nvme_command_t* nvme_cmd,uint32_t* page_buf32,uint32_t* meta_buf32,unsigned int page_len)
{
	uint32_t i,sl32,sh32;
	*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,nvme_cmd->cdw[i]);
	//--------------------------
	mdelay(10);              //
	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	if((sh32&0x07) == 0x00){
		if(page_buf32&&page_len){
			for(i=0;i<(page_len/4);i++){
				page_buf32[i] = REG32((ulong)nvme->prp_list_iovm_addr1 + 4*i);
			}
			if(meta_buf32){
				for(i=0;i<((page_len+4095)/4096);i++){
					meta_buf32[i] = REG32((ulong)nvme->prp_list_iovm_addr2 + 4*i);
				}
			}
		}
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
		if(meta_buf32[i]==0x87654321)	return 0xffffffff;
	}else{
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
	}
	return sh32;
}
//-----------------------------------------------------------------------------------------------------------------
//a 6 1 1 20000 0 0 ffff0001 ffffffff 0
//海力士盘 0000 1 20000 40000 0 0 0 0
//海力士盘 0100 1 20000 40000 0 0 0 0
//擦除第一个底层指令为  8003 0 2000a 20000 ffffffff ffff0000 0 0
//擦除第一个底层指令为  8103 0 2000a 20000 ffffffff ffff0000 0 0
//写第一个底层指令为    8001 2 2000a  00xx ffffffff ffff0000 0 0
//					 18004 0     0     0      0    0      0 0
//写第二个底层指令为    8101 2 2000a  00xx ffffffff ffff0000 0 0
//					 18104 0     0     0      0    0     0 0
//-----------------------------------------------------------------------------------------------------------------
static unsigned int g_PageEcc[8][4];
int nvme_page_read(nvme_command_t* cmd)
{
	uint32_t i,k,e,pages,sl32,sh32;
	uint32_t* pPageBuffer32 = (uint32_t*)nvme->page_data->page_buf;
	uint32_t time_out = 0x80000;
	cmd->cdw[0] &= 0xff00;	cmd->cdw[1] = ((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.'))?2:4;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,cmd->cdw[i]);
	pages = cmd->cdw[1];
	while(--time_out){
	   sl32 = nvme_ack_read_low();
	   if(sl32 == 0) break;
	}
	if(sl32&0xff){
		nvme_ackpoint_inc();					//nvme_command_ack_point increase
		return sl32;
	}
	sh32 = nvme_ack_read_high()&0x0f;
	nvme_ackpoint_inc();						//nvme_command_ack_point increase
	if(sh32 == 0){								//good data
		for(k=0,e=0;k<pages;k++){
			for(i=0;i<0x400;i++)	pPageBuffer32[k*0x400 + i] = REG32((ulong)nvme->prp_list_iovm_addr1 + k*0x1000 + i*4);
			for(i=0;i<0x4;i++){
				nvme->page_data->page_ecc[k][i] = REG32((ulong)nvme->prp_list_iovm_addr2 + k*0x10 + i*4);
				if(nvme->page_data->page_ecc[k][i]!=g_PageEcc[k][i])
				{
					e++;
					g_PageEcc[k][i] = nvme->page_data->page_ecc[k][i];
				}
			}
		}
		/*
		if((nvme->page_data->page_ecc[0][0]==0x87654321)||(e == 0)){
			return 0xffffffff;
		}*/
	}
	if(echo){
		for(k=0;k<pages;k++)	wp_dump_data((unsigned char*)&nvme->page_data->page_buf[4096*k],0x80);//4096
		for(k=0;k<pages;k++)	wp_dump_memory((unsigned char*)&nvme->page_data->page_ecc[k],16);
	}
	return sh32;
}
int nvme_page_read_ss(nvme_command_t* cmd)
{
	uint32_t i,k,pages,sl32,sh32;
	uint32_t* pPageBuffer32 = (uint32_t*)nvme->page_data->page_buf;
	uint32_t time_out = 0x80000;
	*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,cmd->cdw[i]);
	pages = cmd->cdw[1];
	udelay(100);
	while(--time_out){
	   sl32 = nvme_ack_read_low();
	   if((sl32&0xff)==0) break;
	}
	sh32 = nvme_ack_read_high();
	nvme_ackpoint_inc();					//nvme_command_ack_point increase
	if(sl32&0xff){printf("error_sl32:%08x\n",sl32);	return sl32;}
	if(sh32 == 0){							//good data
		for(k=0;k<pages;k++){
			for(i=0;i<0x400;i++)	pPageBuffer32[k*0x400 + i] = REG32((ulong)nvme->prp_list_iovm_addr1 + k*0x1000 + i*4);
			for(i=0;i<0x4;i++){
				nvme->page_data->page_ecc[k][i] = REG32((ulong)nvme->prp_list_iovm_addr2 + k*0x10 + i*4);
			}
		}
	}else{
		printf("error_sh32:%08x\n",sh32); return sh32;
	}
	for(k=0;k<pages;k++)	wp_dump_data((unsigned char*)&nvme->page_data->page_buf[4096*k],128);//4096
	for(k=0;k<pages;k++)	wp_dump_memory((unsigned char*)&nvme->page_data->page_ecc[k],16);
	printf("============================================================================\n");
	return sh32;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_block_erase(nvme_command_t* cmd)
{
	uint32_t i,sl32,sh32;	//
	volatile uint32_t time_out = 0x80000;
	cmd->cdw[0] = (cmd->cdw[0]&0xf00)|0x8003;
	cmd->cdw[1] = 0x00;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,cmd->cdw[i]);
	mdelay(1);              		//
	while(--time_out)
	{
	   sl32 = nvme_ack_read_low();
	   if(sl32 == 0) break;
	}
	//printf("time_out: %08x\n",time_out);
	//------check ack---
	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	if(echo)printf("%3x %x %08x %08x %08x %08x ->%08x %08x\n",\
			cmd->cdw[0],cmd->cdw[1],cmd->cdw[2],cmd->cdw[3],cmd->cdw[4],cmd->cdw[5],sl32,sh32);
	if(sl32&0xff){
		if(echo)printf("Erase block: get ack:%08x-%08x failed!\n",sl32,sh32);
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
		return -1;
	}
	if(sh32 == 0x00000000){
		if(echo)printf("Erase OKAY!:  *!!!!!!!!!* %08x %08x\n",sl32,sh32);
	}else{
		if(echo)printf("Erase ERROR!: *Error ACK* %08x %08x\n",sl32,sh32);
	}
	nvme_ackpoint_inc();			//nvme_command_ack_point increase
	return sh32;				//ack->0:correct,otherwise failed!
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_page_write(nvme_command_t* cmd)
{
	uint32_t i,k,sl32,sh32,pps;
	volatile uint32_t time_out = 0x80000;
	uint32_t* pPageBuffer32 = (uint32_t*)nvme->page_data->page_buf;
	pps = ((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.'))?2:4;
	//--------------
	for(k = 0;k < pps;k++){
		for(i=0;i<0x400;i++){
			*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr1 + k*0x1000 + 4*i) = pPageBuffer32[k*0x400 + i];
		}
		for(i=0;i<0x4;i++){
			*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2 + k*0x10 + i*4) = nvme->page_data->page_ecc[k][i];
		}
	}
	cmd->cdw[0] = (cmd->cdw[0]&0x0f00)|0x8001;
	cmd->cdw[1] = pps;
	if((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.'))
	{
		cmd->cdw[2] = 0x2000A;		cmd->cdw[3] &= 0xff;
	}
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,cmd->cdw[i]);
	mdelay(1);		//10-5
	while(--time_out)
	{
	   sl32 = nvme_ack_read_low();
	   if(sl32 == 0) break;
	}
	//
	nvme_write_reg32(0x4000,(cmd->cdw[0]&0x0f00)|0x18004);
	for(i=1;i<8;i++)	nvme_write_reg32(0x4000,0x00000000);
	mdelay(1);		//300->50
	time_out = 0x80000;
	while(--time_out)
	{
	   sl32 = nvme_ack_read_low();
	   if(sl32 == 0) break;
	}
	//
	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	if(echo)printf("WritePage:%x wACK1====%08x %08x\n",cmd->cdw[3]&0xff,sl32,sh32);
	nvme_ackpoint_inc();		//nvme_command_ack_point increase
	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	if(echo)printf("WritePage:%x wACK2====%08x %08x\n",cmd->cdw[3]&0xff,sl32,sh32);
	nvme_ackpoint_inc();		//nvme_command_ack_point increase
	return sh32;
}
//-----------------------------------------------------------------------------------------------------------------
int nv_block_copy(nvme_command_t* src_cmd,nvme_command_t* dst_cmd)
{
	unsigned int i,p = 0,cnt = 0,sp = 0;
	unsigned int pps = ((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.'))?2:4;		//pages per sector
	unsigned int* psyscfg_buf8;

	for(p = 0,cnt = 0;;p++){		//p:page
		src_cmd->cdw[0] &= 0x0f00;	src_cmd->cdw[1] = pps;
		src_cmd->cdw[3] = (src_cmd->cdw[3]&0xffff0000)|p;
		dst_cmd->cdw[0] &= 0x0f00;	dst_cmd->cdw[1] = pps;
		dst_cmd->cdw[3] = (dst_cmd->cdw[3]&0xffff0000)|p;
		if(nvme_page_read(src_cmd) !=0 ){
			ack_h = 0x55AA1234;		ack_l = cnt;		//此处返回比较特殊
			return cnt; 		//over or read failed!
		}
		//-----
		if(nvme->page_data->page_ecc[0][0]==0xffffff09){		//syscfg page
			for(i=0;i<pps;i++){
				psyscfg_buf8 = (unsigned int*)&nvme->syscfg_data32[sp++][0];
				memcpy(&nvme->page_data->page_buf[i*4096],psyscfg_buf8,4096);
				ecc_vertify_page(&nvme->page_data->page_buf[i*4096],(unsigned char*)&nvme->page_data->page_ecc[i][0]);
			}
			if(sp>=32)	sp = 0;
		}
		//-----
		if(nvme_page_write(dst_cmd) !=0 )	return 0;
		cnt++;
	}
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
int	nvme_cmd_send(uint32_t* pcmd,uint8_t* buf,unsigned int len)	//line command just for user input
{
	uint32_t i,new_c32,status_c32;
	//*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,pcmd[i]);
	for(i=0;i<8;i++){
		if(i==0)    printf("cmd:<%08x ",pcmd[i]); else if(i==7) printf("%08x\n",pcmd[i]);  else printf("%08x ",pcmd[i]);
	}
	//--------------------------
	mdelay(10);              //
	//------check ack-------
	new_c32 = nvme_ack_read_low();
	status_c32 = nvme_ack_read_high();
	if(((new_c32&0xff)==0x00)&&((pcmd[0]&0x0f)==0)){
		if(buf&&len){
			if((status_c32&0xffff0000)==0x00000000){
				for(i=0;i<(len/0x40);i++){
					//data32 = REG32((ulong)nvme->prp_list_iovm_addr1 + i*0x40);
					wp_dump_data((unsigned char*)(ulong)nvme->prp_list_iovm_addr1 + i*0x40,0x40);                      //如何更新???
				}
				if((pcmd[0]&0xff)==0){
					if(pcmd[1]==0)	pcmd[1] = 1;
					for(i=0;i<((pcmd[1]<8)?pcmd[1]:8);i++){
						wp_dump_memory((unsigned char*)(ulong)nvme->prp_list_iovm_addr2 + 0x10*i,0x10);
					}
				}
			}
		}
		printf("ACK---OKAY! %08x %08x\n",new_c32,status_c32);
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
		return status_c32;
	}else if((new_c32&0xff00)==0x00){
		printf("ACK---NoKnow! %08x %08x\n",new_c32,status_c32);
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
		return status_c32;
	}else{
		printf("*Error ACK* %08x %08x\n",new_c32,status_c32);
		nvme_ackpoint_inc();		//nvme_command_ack_point increase
		return -1;
	}
}
//---------------------------------------------------------------------------------------------------------------------
//Flush the data buffer
int	nvme_flush(void)
{
	uint32_t i,pcmd[8],sl32,sh32;
	memset(pcmd,0x00,32);	pcmd[0] = 0x0001ffa6;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,pcmd[i]);
	//------check ack-------

	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	nvme_ackpoint_inc();		//nvme_command_ack_point increase
	printf("sl32:%08x sh32:%08x\n",sl32,sh32);
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_index_sub(nvme_command_t* nvme_cmd,uint32_t* meta)
{
	uint32_t i,sl32,meta32;
	*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,nvme_cmd->cdw[i]);
	mdelay(6);	if(buf_get4no(&nvme->nvme_geom.MSP_Name[0])=='s5e.') mdelay(6);
	sl32 = nvme_ack_read_low();
	nvme_ackpoint_inc();		//nvme_command_ack_point increase
	if(sl32)	{if(meta) *meta = 0xabcd0000|(sl32&0xffff);	return -1;}
	meta32 = REG32((ulong)nvme->prp_list_iovm_addr2);
	if(meta32 == 0x87654321)	{if(meta) *meta = meta32;	return -2;}
	if( ((meta32>=0xffffff04)&&(meta32<=0xffffff09)) ||	(meta32==0xffffff0B)||(meta32==0xffffff0C)||(meta32==0xffffff0E)){
		if(meta) *meta = meta32;
		return 1;
	}
	if(meta) *meta = meta32;
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_index_pos(uint32_t die,nvme_command_t* nvme_cmd)
{
	//一般的字库码片位置在 [100 1 20002 0 ffff0000 ffffffff 0 0]
	//需要找的是备份区位置 [000 1 20000 0 ffff00xx ffffffff 0 0]
	//		           [000 1 20000 0 00xxffff ffffffff 0 0]
	//--------------------------------------------------------
	//		       [200 1 20004 0 ffff0000 ffffffff 0 0](256G Samsung)
	//Hynix: (1)   [100 1 20000 40000 0 0 0][000 1 20000 40000 0 0]	第一种
	//Hynix: (2)   [100 1 20000 00000 0 0 0][000 1 20000 60000 0 0]
	uint32_t lr,block,meta32;		uint32_t block_id; int r,nc;
	nvme_command_t cmd;	memset(&cmd,0x00,sizeof(cmd));
	/*
	if((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])=='s5e.')){
		mdelay(100);
		cmd.cdw[0] = die*0x100;			cmd.cdw[1] = 0x1;	//只读一页
		cmd.cdw[2] = 0x20000|(die*2);	cmd.cdw[3] = 0;
		cmd.cdw[4] = 0xffff0001;		cmd.cdw[5] = 0xffffffff;
		cmd.cdw[6] = 0;					cmd.cdw[7] = 0;
		r = nvme_index_sub(&cmd,&meta32);
		printf("===%3x %x %08x %08x %08x %08x ->%08x\n",cmd.cdw[0],cmd.cdw[1],cmd.cdw[2],cmd.cdw[3],cmd.cdw[4],cmd.cdw[5],meta32);
		if(r==1)	goto found;
	}*/
	//below for hynix-------
	if((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.')){	//海力士的每块分8页 cmd[1]:读的页数[3]:x0000的x从 0-7指的是索引
		cmd.cdw[0] = die*0x100;cmd.cdw[1] = 0x1;cmd.cdw[2] = 0x20000|(die*2);
		for(block_id = 0x00000;block_id<=0x60000;block_id+=0x20000){
			cmd.cdw[3] = block_id;
			if(nvme_index_sub(&cmd,&meta32)==1)	goto found;
		}
		return 0;
	}
	//----below for samsung toshiba sandisk
	for(lr = 0;lr<2;lr++){
		for(block = 0,nc = 0;block<0x100;block++){
			cmd.cdw[0] = die*0x100;			cmd.cdw[1] = 0x1;	//只读一页
			cmd.cdw[2] = 0x20000|(die*2);	cmd.cdw[3] = 0;
			cmd.cdw[5] = 0xffffffff;		cmd.cdw[6] = 0;		cmd.cdw[7] = 0;
			if(die == 0){
				block_id = (lr==0)?((block*0x10000)|0xffff):(0xffff0000|block);
			}else{
				block_id = (lr==0)? (block|0xffff0000):((block*0x1000)|0xffff);
			}
			cmd.cdw[4] = block_id;
			r = nvme_index_sub(&cmd,&meta32);
			if(nvme_cmd==NULL)
				printf("%3x %x %08x %08x %08x %08x ->%08x\n",cmd.cdw[0],cmd.cdw[1],cmd.cdw[2],cmd.cdw[3],cmd.cdw[4],cmd.cdw[5],meta32);
			if(r < 0){++nc;if(nc>0x10) break;}
			else nc = 0;
			if(r==1)	goto found;
		}
	}
	return 0;
found:
	if(nvme_cmd)	memcpy(nvme_cmd,&cmd,sizeof(cmd));
	return 1;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_index_nv(uint32_t id)
{
	nvme_command_t cmd;
	unsigned int i = 0,page = 0,sl32,meta32 = 0,sys_pos = 0,page09 = 0;
	memcpy(&cmd,nvme->nvme_geom.syscfg_cmd[id],sizeof(nvme_command_t));	//
	if(cmd.cdw[0] == 0xffffffff)	return -1;
	cmd.cdw[1] = 0x01;		//只读一页									//
	while(1){
		cmd.cdw[3] = (cmd.cdw[3]&0xffff0000)|page;
		*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
		for(i = 0;i < 8;i++)	nvme_write_reg32(0x4000,cmd.cdw[i]);
		mdelay(6);						//mdelay(6);
		meta32 = *(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2);
		sl32 = nvme_ack_read_low();
		nvme_ackpoint_inc();											//nvme_command_ack_point increase
		if(meta32 == 0xffffff09){
			sys_pos = page;
			if((nvme->nvme_geom.syscfg_page_pos0 == 0x0000)&&(page09==0)){
				nvme->nvme_geom.syscfg_page_pos0 = page;
			}else if((nvme->nvme_geom.syscfg_page_pos0)&&(nvme->nvme_geom.syscfg_page_pos1 == 0x0000)&&(page09==0))
				nvme->nvme_geom.syscfg_page_pos1 = page;
			//printf("%3x %x %08x %08x %08x %08x ->%08x\n",cmd.cdw[0],cmd.cdw[1],cmd.cdw[2],cmd.cdw[3],cmd.cdw[4],cmd.cdw[5],meta32);
			page09++;
		}else page09 = 0;

		if(((sl32&0x0f)||(meta32==0x87654321))&&((meta32&0xffffff00)!=0xffffff00)){
			if(echo)printf("sl32:%x meta32:%x\n",sl32,meta32);
			nvme->nvme_geom.syscfg_cnt = page;
			if((nvme->nvme_geom.Vendor!=Hynix)||(buf_get4no(&nvme->nvme_geom.MSP_Name[0])=='s5e.'))
				nvme->nvme_geom.syscfg_page_pos = (sys_pos>0x08)?(sys_pos-0x07):0x00;
			else
				nvme->nvme_geom.syscfg_page_pos = (sys_pos>0x10)?(sys_pos-0x0f):0x00;
			return 0;
		}
		page++;	if(page>=0x100)	return -2;
	}
	return -1;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_index1(bool becho)
{
	int k,nvc = 0,r = 0,gotid = 0xff; uint32_t* pv32,meta32;	nvme_command_t cmd;
	cmd.cdw[1] = 0x1;cmd.cdw[3] = 0x00;cmd.cdw[4] = 0xffff0000;cmd.cdw[5] = 0xffffffff;cmd.cdw[6] = 0x0000;cmd.cdw[7] = 0x0000;
	//----为了快速找到就先查问常见快和页
	if (((nvme->nvme_geom.Vendor!=Hynix)||(buf_get4no(&nvme->nvme_geom.MSP_Name[0])=='s5e.')) && (nvme->nvme_geom.die_cnt >= 1)){
		for(k=1,nvc=0;k < (nvme->nvme_geom.die_cnt*2);k++){
			cmd.cdw[0] = 0x100*k;cmd.cdw[2] = 0x20000|(k*2);
			r = nvme_index_sub(&cmd,&meta32);
			if(r==1){
				memcpy(&nvme->nvme_geom.syscfg_cmd[nvc],&cmd,sizeof(cmd));
				pv32 = nvme->nvme_geom.syscfg_cmd[nvc];
				if(becho)printf("boot%d:%04x %x %05x %5x %08x %8x %x %x\n",nvc,pv32[0],pv32[1],pv32[2],pv32[3],pv32[4],pv32[5],pv32[6],pv32[7]);
				gotid = k; nvc++; goto nv_ok;
			}
		}
	}
	//---------------------------------
	for(k=0;k<(nvme->nvme_geom.die_cnt*2);k++){
		if(k == gotid)	continue;		//快速查找已经找到的Die
		r = nvme_index_pos(k,(nvme_command_t*)&nvme->nvme_geom.syscfg_cmd[nvc]);	//die0区的位置命令
		if(r == 0){
			if(becho)	printf("index_die:%d failed!\n",k);
		}else{
			pv32 = nvme->nvme_geom.syscfg_cmd[nvc];
			if(becho)printf("boot%d:%04x %x %05x %5x %08x %8x %x %x\n",nvc,pv32[0],pv32[1],pv32[2],pv32[3],pv32[4],pv32[5],pv32[6],pv32[7]);
			nvc++; goto nv_ok;
		}
	}
	//---------------------------------
nv_ok:
	if(nvc){
		if(nvme_index_nv(0)<0){				//引导数据位置
			if(becho)printf("boot0 index nv failed!\n");
		}
	}
	if(becho)printf(">>>nv_pages:%x pos:%x\n",nvme->nvme_geom.syscfg_cnt,nvme->nvme_geom.syscfg_page_pos);
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_index2(bool becho)
{
	int k,nvc = 0,r = 0,gotid = 0xff; uint32_t* pv32,meta32;	nvme_command_t cmd;
	cmd.cdw[1] = 0x1;cmd.cdw[3] = 0x00;cmd.cdw[4] = 0xffff0000;cmd.cdw[5] = 0xffffffff;cmd.cdw[6] = 0x0000;cmd.cdw[7] = 0x0000;
	//----为了快速找到就先查问常见快和页
	if(((nvme->nvme_geom.Vendor!=Hynix)||(buf_get4no(&nvme->nvme_geom.MSP_Name[0])=='s5e.'))&&(nvme->nvme_geom.die_cnt>=1)){
		for(k=1,nvc=0;k<(nvme->nvme_geom.die_cnt*2);k++){
			cmd.cdw[0] = 0x100*k;cmd.cdw[2] = 0x20000|(k*2);
			r = nvme_index_sub(&cmd,&meta32);
			if(r==1){
				memcpy(&nvme->nvme_geom.syscfg_cmd[nvc],&cmd,sizeof(cmd));
				pv32 = nvme->nvme_geom.syscfg_cmd[nvc];
				if(becho)printf("boot%d:%04x %x %05x %5x %08x %8x %x %x\n",nvc,pv32[0],pv32[1],pv32[2],pv32[3],pv32[4],pv32[5],pv32[6],pv32[7]);
				gotid = k; nvc++;	if(nvc>=2)	goto nv_ok;
			}
		}
	}
	//---------------------------------
	for(k=0;k<(nvme->nvme_geom.die_cnt*2);k++){
		if(k == gotid)	continue;		//快速查找已经找到的Die
		r = nvme_index_pos(k,(nvme_command_t*)&nvme->nvme_geom.syscfg_cmd[nvc]);	//die0区的位置命令
		if(r == 0){
			if(becho)	printf("index_die:%d failed!\n",k);
		}else{
			pv32 = nvme->nvme_geom.syscfg_cmd[nvc];
			if(becho)printf("boot%d:%04x %x %05x %5x %08x %8x %x %x\n",nvc,pv32[0],pv32[1],pv32[2],pv32[3],pv32[4],pv32[5],pv32[6],pv32[7]);
			nvc++;
		}
		if(nvc>=2)	break;
	}//---------------------------------
nv_ok:
	if(nvc){
		if(nvme_index_nv(0)<0){				//引导数据位置
			if(becho)printf("boot0 index nv failed!\n");
		}
	}
	if(becho)printf(">>>nv_pages:%x pos:%x\n",nvme->nvme_geom.syscfg_cnt,nvme->nvme_geom.syscfg_page_pos);
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
int nvme_look_for(unsigned char p)
{
	nvme_command_t cmd;unsigned int i,pos,sl32,meta32,data32,*pv32;
	cmd.cdw[0] = 0x100;cmd.cdw[1] = 0x1;cmd.cdw[2] = 0x20002;cmd.cdw[3] = 0;
	pos = 0;cmd.cdw[5] = 0xffffffff;cmd.cdw[6] = 0x00;cmd.cdw[7] = 0x00;
	printf("Start to look for sth!\n");
	while(1){
		cmd.cdw[4] = 0xffff0000+pos;
		*(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2) = 0x87654321;
		for(i = 0;i < 8;i++)	nvme_write_reg32(0x4000,cmd.cdw[i]);
		mdelay(6);						//mdelay(6);
		meta32 = *(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr2);
		sl32 = nvme_ack_read_low();
		nvme_ackpoint_inc();			//nvme_command_ack_point increase
		data32 = *(volatile uint32_t *)((ulong)nvme->prp_list_iovm_addr1);
		if(((data32&0xffff)==0x8230)||(pos == 0x12)||(meta32==0x87654321)){
			pv32 = (unsigned int*)&cmd.cdw[0];
			printf("<0x%08x> %04x %x %05x %5x %08x %8x %x %x\n",data32,pv32[0],pv32[1],pv32[2],pv32[3],pv32[4],pv32[5],pv32[6],pv32[7]);
		}
		pos++; if(pos>0x10000)	break;
	}
	printf("Over look for!\n");
	return -1;
}
//-----------------------------------------------------------------------------------------------------------------
void Set_InitCmd(void){	g_InitCmd[0] = 0xFFA1;	g_InitCmd[1] = 0xFF16;	g_InitCmd[2] = 0xFF16;	g_InitCmd[3] = 0xFFB8;}
static int nvme_init_ans2(void)
{
	int k,i,r = 0;uint32_t buf32[100];
	uint32_t cmd[8];
	//const uint32_t cmdx[] = {0x0000FFAD,0x00000001,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
	//const uint32_t cmdx[] = {0x01FFFF14,0x00000020,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
	const uint32_t cmd1[] = {/*0x0000FFA1,*/0x0,0x00000001,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
	const uint32_t cmd2[] = {/*0x0000FF16,*/0x0,0x00000001,0x0000000C,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
	const uint32_t cmd3[] = {/*0x0000FF16,*/0x0,0x00000001,0x0000001A,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
	const uint32_t cmd4[] = {/*0x0000FFB8,*/0x0,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};

	memset((void*)nvme->bar2_completion_base,0xff,0x800);

	memcpy(cmd,cmd1,32);	cmd[0] = g_InitCmd[0];
	r = nvme_cmd_process((nvme_command_t*)cmd,(uint32_t*)&nvme->nvme_geom,NULL,0x200);	//
	//wp_dump_memory((unsigned char*)&nvme->nvme_geom,sizeof(nvme->nvme_geom));
	memcpy(cmd,cmd2,32);	cmd[0] = g_InitCmd[1];
	r = nvme_cmd_process((nvme_command_t*)cmd,buf32,NULL,0);
	memcpy(cmd,cmd3,32);	cmd[0] = g_InitCmd[2];
	r = nvme_cmd_process((nvme_command_t*)cmd,buf32,NULL,0);
	memcpy(cmd,cmd4,32);	cmd[0] = g_InitCmd[3];
	r = nvme_cmd_process((nvme_command_t*)cmd,NULL,NULL,0);
	if(r!=0)	return -1;
	//---------------------------------
	for(k=0;k<2;k++){
		for(i=0;i<8;i++)
			nvme->nvme_geom.syscfg_cmd[k][i] = 0xffffffff;
	}
	nvme->nvme_geom.syscfg_cnt = 0x00;
	nvme->nvme_geom.syscfg_page_pos = 0;
	//---------------------------------
	nvme->initialized = true;
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------
void nvme_test_menu(void)
{
	unsigned char title[128];
	strcpy(title,"=====ANS2 S4E test menu========\r");
	serial_putbuffer((unsigned char*)title,32);
	/*printf("0.a a show this menu\n");
	printf("1.a 2 nvme command    ->erase block\n");
	printf("2.c cmd1 cmd2 ....    ->exec cmd\n");
	printf("3.d display buffer    ->display buffer\n");
	printf("4.f host command      ->pc command\n");*/
}
//-----------------------------------------------------------------------------------------------------------------
#define SOFTRES_REG (RALINK_SYSCTL_BASE + 0x0034)
#define SYS_RST		(0x01<<0)
static int do_reset(void)
{
	*(volatile unsigned int*)(SOFTRES_REG) = SYS_RST;
	return 1;
}
int nvme_cmd_pc(nvme_command_t* nvme_cmd)
{
	uint32_t i,sl32,sh32;
	for(i=0;i<8;i++)	nvme_write_reg32(0x4000,nvme_cmd->cdw[i]);
	//--------------------------
	mdelay(20);              //
	sl32 = nvme_ack_read_low();	sh32 = nvme_ack_read_high();
	printf("%08x %08x\n",sl32,sh32);
	return sh32;
}
//-----------------------------------------------------------------------------------------------------------------
//处理上位机发的命令
//-----------------------------------------------------------------------------------------------------------------
void nvme_process_host_command(uint32_t* cmd)
{

	uint32_t i,v32;	uint8_t* pSysCfgData8;	uint8_t crc_buf8[4];	nvme_command_t m_command;
	echo = 0;
	switch(cmd[1]){
	case 1:			//[f 1 (0-31)]receiver the syscfg data buffer 132k
		pSysCfgData8 = (uint8_t*)&nvme->syscfg_data32[cmd[2]][0];	//32*1024
		if(serial_getbuffer(pSysCfgData8,0x1000) != 0x1000){	v32 = -1;serial_putbuffer((unsigned char*)&v32,4);	break;	}
		if(serial_getbuffer(crc_buf8,4)!=4){v32 = -2;serial_putbuffer((unsigned char*)&v32,4);break;	}
		v32 = crc32(0,pSysCfgData8,0x1000);
		if(v32!=buf_get4(crc_buf8)){v32 = -3;serial_putbuffer((unsigned char*)&v32,4);	break;}
		else{v32 = 0;serial_putbuffer((unsigned char*)&v32,4);}
		break;
	case 6:			//[f 6 (0-31)]read back the syscfg data buffer
		pSysCfgData8 = (uint8_t*)&nvme->syscfg_data32[cmd[2]][0];
		serial_putbuffer(pSysCfgData8,0x1000);
		v32 = crc32(0,pSysCfgData8,0x1000);
		serial_putbuffer((unsigned char*)&v32,4);
		break;
	case 0:			//[f 0 src_cmd1 dst_cmd]
		v32 = nv_block_copy((nvme_command_t*)&cmd[2],(nvme_command_t*)&cmd[10]);
		serial_putbuffer((unsigned char*)&v32,4);
		break;
	case 2:			//host erase the block
		v32 = nvme_block_erase((nvme_command_t*)&cmd[2]);
		serial_putbuffer((unsigned char*)&v32,4);
		break;
	case 3:			//f 3 1 index chip
		if(cmd[2] == 1)	nvme_index2(false);
		v32 = sizeof(nvme->nvme_geom);												serial_putbuffer((unsigned char*)&v32,4);
		v32  = crc32(0,(unsigned char*)&nvme->nvme_geom,sizeof(nvme->nvme_geom));	serial_putbuffer((unsigned char*)&v32,4);
		serial_putbuffer((unsigned char*)&nvme->nvme_geom,sizeof(nvme->nvme_geom));
		break;
	case 4:			//f 4 [nvme_command]	read command
		for(i=0;i<8;i++)	m_command.cdw[0+i] = cmd[2+i];
		v32 = nvme_page_read(&m_command);		//die,page
		serial_putbuffer((unsigned char*)&v32,4);
		if(v32 == 0){
			for(i=0;i<m_command.cdw[1];i++){
				serial_putbuffer(&nvme->page_data->page_buf[i*0x1000],0x1000);
				serial_putbuffer((unsigned char*)nvme->page_data->page_ecc[i],0x10);
				v32 = crc32(0,&nvme->page_data->page_buf[i*0x1000],0x1000);
				serial_putbuffer((unsigned char*)&v32,4);
				v32 = crc32(0,(unsigned char*)nvme->page_data->page_ecc[i],0x10);
				serial_putbuffer((unsigned char*)&v32,4);
			}
		}
		break;
	case 5:
		do_reset();
		break;
	case 7:
		nvme_cmd_pc((nvme_command_t*)&cmd[2]);
		break;
	}

}
//-----------------------------------------------------------------------------------------------------------------
void process_serial_command(unsigned int* cmd)
{
	uint8_t buffer[1024];
	int r;
	switch(cmd[0]){
		case 0x0a:
			echo = 1;
			if(cmd[1] == 2)			nvme_block_erase((nvme_command_t*)&cmd[2]);
			else if(cmd[1] == 5)	nv_block_copy((nvme_command_t*)&cmd[2],(nvme_command_t*)&cmd[10]);
			else if(cmd[1] == 3)	nvme_index_pos(cmd[2],NULL);
			else if(cmd[1] == 4)	nvme_index2(true);
			else if(cmd[1] == 8)	nvme_index1(true);
			else if(cmd[1] == 6)    nvme_page_read_ss((nvme_command_t*)&cmd[2]);//Read pages
			else if(cmd[1] == 7)	nvme_page_write((nvme_command_t*)&cmd[2]);	//pages write
			else if(cmd[1] == 0xa)	nvme_test_menu();
			else if(cmd[1] == 0xb)	nvme_look_for(cmd[2]);
			break;
		case 0x0b:
			nvme_flush();
			return;
		case 0x0c:
			nvme_cmd_send(&cmd[1],buffer,0x800);
			break;
		case 0x0d:
			if(cmd[1] == 1)	wp_dump_data((unsigned char*)&nvme->nvme_geom,sizeof(nvme->nvme_geom));
			break;
		case 0x0f:
			nvme_process_host_command(cmd);
			break;
	}
}
//------------------------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------------------------
static int nvme_device_init(pci_device_t port_bridge)	//port_bridge
{
	int status;
	status = nvme_init_pci( port_bridge);
	if (status != NVME_SUCCESS)		goto error;
	status = nvme_init_resources();
	if (status != NVME_SUCCESS)    goto error;
	status = nvme_init_controller();
	if (status != NVME_SUCCESS)		goto error;
	// start out in boot mode
	status = nvme_init_pci2();
	return NVME_SUCCESS;
error:
	dprintf(1, "nvme: init failed with status: %d\n", status); //hzq
	return status;
}
//------------------------------------------------------------------------------------------------------------------------------
int  nvme_init_11(pci_device_t port_bridge)
{
    nvme = &nvme_controllers;
    nvme->initialized = false;
    if(nvme_device_init(port_bridge)!=NVME_SUCCESS) return -1;
    if(nvme_init_ans2()!=0)	return -1;
    P11_page_step = ((nvme->nvme_geom.Vendor==Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.'))?2:4;
    return 0;
}
//------------------------------------------------------------------------------------------------------------------------------
/*
void nvme_quiesce_11(int nvme_id)
{
      nvme_t *nvme = &nvme_controllers;
      // We might get asked to quiesce without an enable in the case of SecureROM
      // trying to power off the controller even though it wasn't detected on the bus
      if (nvme->pci_dev != NULL) {
		  nvme_quiesce_controller();
		  nvme_quiesce_pci();
      }
}
*/
int nvme_reconnect(void)
{
	uint32_t v32;
	v32 = pci_config_read32(nvme->pci_dev,0x110);
	if(echo)printf("[config:0x110]:0x%08x\n",v32);
	pci_config_write32(nvme->pci_dev,0x110,0x0001);
	v32 = pci_config_read32(nvme->pci_dev,0x104);
	if(echo)printf("[config:0x104]:0x%08x\n",v32);
	pci_config_write32(nvme->pci_dev,0x104,0x2000);
	return 0;
}
//------------------------------------------------------------------------------------------------------------------------------
//uint32_t nvme_get_max_transfer_blocks(int nvme_id){return 0;}
//------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push, 1)
typedef enum {
	MT7688_HELLO	= 0,	//
	NVME_SYSTEM_RESET,		//系统复位
	NVME_INIT,				//初始化芯片
	NVME_INDEX_NV1,			//引导配置1
	NVME_INDEX_NV2,			//引导配置2
	NVME_GET_GEOM,			//读字库信息
	NVME_PAGE_READ,			//读Page命令
	NVME_PAGE_WRITE,		//写Page命令
	NVME_BLOCK_ERASE,		//删除块命令
	NVME_NV_COPY,			//
	NVME_USER_CMD,			//自定义命令
	NVME_GET_ACK,			//读取命令执行状态
	NVME_SET_PAGE_DATA,		//设置Page数据
	NVME_GET_PAGE_DATA,		//读Page数据
	NVME_SET_ECC_DATA,		//写ECC数据
	NVME_GET_ECC_DATA,		//读ECC数据
	NVME_SET_SYSCFG,		//写SYSCFG数据
	NVME_GET_SYSCFG,		//读SYSCFG数据

	NVME_SET_CODE,			//写MT7688_CODE数据
	NVME_GET_CODE,			//读MT7688_CODE数据
	NVME_SET_TABLE,			//写MT7688_TABLE数据
	NVME_GET_TABLE,			//读MT7688_TABLE数据

	NVME_GET_ALGO1,			//得到PC机校验数据
	NVME_GET_ALGO2,			//得到CH563校验数据
	NVME_SET_ALGO1,			//PC发送数据
	NVME_SET_ALGO2,			//CH563发送校验数据

	NVME_BLOCK_SERACH,		//在块中找关键页

	//备用
	NOP1,
	NOP2,
	NOP3,
	NOP4,
	NOP5,
	NOP6,
	NOP7,
	NOP8,

	//6-7P 指令
	NVME_7_INFO,
	NVME_7_INFO_RETURN_DATA,
	NVME_7_READ,
	NVME_7_READ_RETURN_DATA,
	NVME_7_WRITE,
	NVME_7_WRITE_SEND_DATA,
	NVME_7_PARSE_WIFI_BM_SN,
	//
	NVME_11_PARSE_WIFI_BM_SN,
	NVME_11_SINGLE_WRITE,
	NVME_7_11_SINGLE_WRITE_STATE,
	NVME_7_SINGLE_WRITE,


}MT7688_NVME_CMD;

//以下并行参数包含在 512HID数据中
typedef struct _PARALLEL_CMD {
	unsigned short cmd_len;			//发送到MT7688的指令长度
	unsigned short cmd_crc;			//数据校验
	unsigned short cmd_id;			//主命令
	unsigned short cmd_dir;			//数据方向
	unsigned short buf_len;			//数据长度
	unsigned short buf_type;		//数据类型 0:NVME_CMD 1:Command ACK 2:Chip infor 3:PageData 4:PageCRC 5:Syscfg
	unsigned int   buf_addr;		//数据地址
	union {
		struct {
			unsigned int   nvme_cmd1[8];	//
			unsigned int   nvme_cmd2[8];	//
		}NVMe_CMD;
		struct {
			unsigned int ack1[2];
			unsigned int ack2[2];
		}NVMe_ACK;
		struct {
			unsigned char buf[256 + 128];
		}NVMe_Buffer;
	}PARA_DATA;
}PARALLEL_CMD;
#pragma pack(pop)


//------------------------------------------------------------------------------------------------------------------------------

extern bool pcie_nvme_link(void);
extern int  E3NAND_INFO(void);
extern int lookup_image_in_bdev_hzq(uint32_t type,uint8_t* read_buf,uint32_t block_address,uint16_t count);
extern  int lookup_image_in_bdev( char *name, uint32_t type,uint8_t* read_buf,uint32_t address,uint16_t page_size);
extern int syscfg_write_in_bdev(const char *name, uint32_t type);
extern void P7_get_wifi_bt_sn(void);
extern void P11_get_wifi_bt_sn(void);
int P11_update_new_disk_information(void);
int P7_update_new_disk_information(void);

uint8_t nvme_user_init_on=0;

void process_parallel_command(unsigned char* pcmd,unsigned char* pack)
{
	int r=0;
	int s;
	unsigned char* pData8;
	PARALLEL_CMD* pParallelCommand = (PARALLEL_CMD*)pcmd;
	unsigned short check1,check2;
	static int state= -8989;
	//
	//uint32_t nvme_user_block_address = *(uint32_t *)(& pParallelCommand->PARA_DATA.NVMe_Buffer.buf[0]);
	nvme_user_init_on = pParallelCommand->PARA_DATA.NVMe_Buffer.buf[0x10];
/*
	printf("cmd:%d ",pParallelCommand->cmd_id);
	printf("  len:%d",pParallelCommand->cmd_len);
	printf("  addr:%08x",pParallelCommand->buf_addr);
	printf("  buf_type%d:\n",pParallelCommand->buf_type);
*/
	switch(pParallelCommand->cmd_id){
		case MT7688_HELLO:
			for(r=0;r<16;r++)	pack[r] = r;
			//printf("hello\n");
			break;
		case NVME_SYSTEM_RESET:		//系统复位
			//do_reset();
			break;
		case NVME_INIT:				//初始化芯片
			r = -1;
			if (pcie_nvme_link()){
				if (nvme_init_11(apcie_get_port_bridge()) == 0){
					r = 0x00;
					printf("Init NVMe NAND success!\n");
				}
				else printf("Init NVMe NAND Failed1!\n");
			}else printf("Init NVMe NAND Failed2!\n");
			memcpy(pack,&r,0x04);
			break;
		case NVME_INDEX_NV1:			//引导配置
			r = nvme_index1(0);
			memcpy(pack,&r,0x04);
			break;
		case NVME_INDEX_NV2:			//引导配置
			r = nvme_index2(0);
			memcpy(pack,&r,0x04);
			break;
		case NVME_GET_GEOM:			//读字库信息
			pData8 = (unsigned char*)&nvme->nvme_geom;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_PAGE_READ:		//读Page命令
			//printf("%X %X %X %X %X %X\n",pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]\
								,pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[1]\
								,pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[2]\
								,pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[3]\
								,pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[4]\
								,pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[5]);
			//printf("READ:\n");
			//for(r=0;r<8;r++) printf(" %08x",pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[r]);printf("\n");
			for(r=0;r<5;r++)
			{
				nvme_page_read((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]);
				check1 = check_snm_16((unsigned char*)&nvme->page_data->page_buf[0],P11_page_step*0x1000);
				nvme_page_read((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]);
				check2 = check_snm_16((unsigned char*)&nvme->page_data->page_buf[0],P11_page_step*0x1000);
				//printf("r:%d check1:%4x check2:%4x\n",r,check1,check2);
				if(check1 == check2) break; //连续读两次，数据一样后退出
			}
			break;
		case NVME_PAGE_WRITE:		//写Page命令
			//printf("WRITE:\n");
			//for(r=0;r<8;r++) printf(" %08x",pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[r]);printf("\n");
			s = nvme_page_write((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]);
			//printf("s %04x\n",s);
			break;
		case NVME_BLOCK_ERASE:		//删除块命令
			//printf("ERASE:\n");
			//for(r=0;r<8;r++) printf(" %08x",pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[r]);printf("\n");
			nvme_block_erase((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]);
			//printf("s %04x\n",s);
			break;
		case NVME_NV_COPY:			//
			nv_block_copy((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0],\
					(nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd2[0]);
			break;
		case NVME_USER_CMD:			//自定义命令

			break;
			/*
		case NVME_BLOCK_SERACH:
			r = nvme_block_search((nvme_command_t*)&pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[0]);
			if(r>=0){	ack_l = r;	ack_h =  pParallelCommand->PARA_DATA.NVMe_CMD.nvme_cmd1[4];}
			else {	ack_l = -1;	ack_h = -1;}
			break;*/
		case NVME_GET_ACK:			//读取命令执行状态
			memcpy(&pack[0],&ack_l,4);memcpy(&pack[4],&ack_h,4);
			//printf("al:%08x ah:%08x\n",ack_l,ack_h);
			break;
		case NVME_SET_PAGE_DATA: 	//设置Page数据
			pData8 = (unsigned char*)&nvme->page_data->page_buf[0];
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			break;
		case NVME_GET_PAGE_DATA: 	//读Page数据
			pData8 = (unsigned char*)&nvme->page_data->page_buf[0];
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_SET_ECC_DATA:		//写ECC数据
			pData8 = (unsigned char*)&nvme->page_data->page_ecc;
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			break;
		case NVME_GET_ECC_DATA: 	//读ECC数据
			pData8 = (unsigned char*)&nvme->page_data->page_ecc;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_SET_SYSCFG:		//写SYSCFG数据 ok
			pData8 = (unsigned char*)&nvme->syscfg_data32;
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			break;
		case NVME_GET_SYSCFG:		//读SYSCFG数据 ok
			pData8 = (unsigned char*)&nvme->syscfg_data32;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
#if(0)
		case NVME_SET_CODE:
			pData8 = (unsigned char*)&ecc_vertify_crc;
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			break;
		case NVME_GET_CODE:
			pData8 = (unsigned char*)&ecc_vertify_crc;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_SET_TABLE:
			pData8 = (unsigned char*)ecc_table2;
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			break;
		case NVME_GET_TABLE:
			pData8 = (unsigned char*)ecc_table2;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
#endif
		case NVME_GET_ALGO1:			//得到PC机校验数据
			Algo1_Buf_Get();
			pData8 = (unsigned char*)algo1_buf;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_SET_ALGO1:			//PC发送数据
			pData8 = (unsigned char*)&algo1_buf[64];
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			Algo1_Buf_Vertify(&g_InitCmd[0]);
			break;
		case NVME_GET_ALGO2:			//得到CH563校验数据
			Algo2_Buf_Get();
			pData8 = (unsigned char*)algo2_buf;
			memcpy(pack,&pData8[pParallelCommand->buf_addr],pParallelCommand->buf_len);
			break;
		case NVME_SET_ALGO2:			//CH563发送校验数据
			pData8 = (unsigned char*)&algo2_buf[64];
			memcpy(&pData8[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,pParallelCommand->buf_len);
			Algo2_Buf_Vertify();
			break;
	     //----------------------------------以下是7代盘---------------------------------
		case NVME_7_INFO:
			E3NAND_INFO();
			//for(r=0;r<printf2_length;r++)  printf("%c",printf2_BUF[r]);
			//printf("\n");
			break;
		case NVME_7_INFO_RETURN_DATA:
			 memcpy(pack,&P7_INFO_BUF[pParallelCommand->buf_addr],0x100);
			 if(pParallelCommand->buf_addr == 0x400) P7_INFO_length=0; //数据返回完成
			 break;
		case NVME_7_READ:

			for(r=0;r<0x20000;r+=0x1000)
			{
				s = lookup_image_in_bdev_hzq(3,&syscfg_buf[r],r/NVME_BLOCK_SIZE,1);
			}
			/*
			for(r=0;r<0x20000;r+=0x8000)
			{
				s = lookup_image_in_bdev("nvme_syscfg0",0,&syscfg_buf[r],r,0x8000);
				//printf("s:%d\n",s);
			}*/
			/*
			for(r=0;r<0x20000;r+=0x10000)//一个0x20000 每次读0x10000
			{
				s = lookup_image_in_bdev_hzq(pParallelCommand->buf_type,&syscfg_buf[r],pParallelCommand->buf_addr+r/NVME_BLOCK_SIZE,16);//16=0x8000/NVME_BLOCK_SIZE
				udelay(100);
			}*/
			check1 = check_snm_16(syscfg_buf,0x20000);
			printf("check1:%04x\n",check1);
			break;
		case NVME_7_READ_RETURN_DATA:
			memcpy(pack,&syscfg_buf[pParallelCommand->buf_addr],0x100);
			break;
		case NVME_7_WRITE_SEND_DATA:
			//for(r=0;r<0x100;r++) { if(r%0x10==0)printf("\n");printf(" %02x",pParallelCommand->PARA_DATA.NVMe_Buffer.buf[r]);}
			//printf("\n");
			memcpy(&syscfg_buf[pParallelCommand->buf_addr],pParallelCommand->PARA_DATA.NVMe_Buffer.buf,0x100);
			break;
		case NVME_7_WRITE:
			//syscfg_write_in_bdev("nvme_syscfg0", 0xff);//ff=擦除
			syscfg_write_in_bdev("nvme_syscfg0", 1);
			break;
		case NVME_7_PARSE_WIFI_BM_SN:
			P7_get_wifi_bt_sn();
			break;
		case NVME_11_PARSE_WIFI_BM_SN:
			P11_get_wifi_bt_sn();
			break;
		case NVME_11_SINGLE_WRITE:
			state = P11_update_new_disk_information();
			break;
		case NVME_7_11_SINGLE_WRITE_STATE:
			(*(int *)(&pack[0])) = state;
			break;
		case NVME_7_SINGLE_WRITE:
			state = P7_update_new_disk_information();
			break;

	}
}
//-------------------------------------------------------------------------------------------------------
int P11_read_syscfg_all(void)
{
	int i,k,ch1;
	unsigned short check1,check2;
	unsigned int page_step=0;
	int die = 1;							//默认读第二区
	nvme_command_t nvme_command_h;
	uint32_t dat1,dat2;
	//read
	if ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) 	page_step = 2;
	 	 else page_step = 4;

    if ((nvme->nvme_geom.syscfg_cnt < 2) || (nvme->nvme_geom.syscfg_cnt >= 0x200)) {
         return -111;
    }
    //----
    if ((nvme->nvme_geom.syscfg_cmd[0][0]==0xffffffff) && (nvme->nvme_geom.syscfg_cmd[1][0] == 0xffffffff)) {
        return -102;
    }
    if ((nvme->nvme_geom.syscfg_cmd[0][0] == 0xffffffff) || (nvme->nvme_geom.syscfg_cmd[1][0] == 0xffffffff)) {
        die = 0;
    }
    //处理页
    if ((nvme->nvme_geom.syscfg_page_pos == 0)&&(nvme->nvme_geom.syscfg_page_pos0 == 0) && (nvme->nvme_geom.syscfg_page_pos1 == 0)) {
       // Edit_1("   获取芯片PCIe数据失败!\n",2);
       return -105;
    }else if ((nvme->nvme_geom.syscfg_page_pos == 0) && (nvme->nvme_geom.syscfg_page_pos0) && (nvme->nvme_geom.syscfg_page_pos1)) {
        nvme->nvme_geom.syscfg_page_pos = nvme->nvme_geom.syscfg_page_pos0;
        nvme->nvme_geom.syscfg_cnt = nvme->nvme_geom.syscfg_page_pos1;
    }
    else if ((nvme->nvme_geom.syscfg_page_pos == 0) && (nvme->nvme_geom.syscfg_page_pos0) && (nvme->nvme_geom.syscfg_page_pos1==0)) {
        nvme->nvme_geom.syscfg_page_pos = nvme->nvme_geom.syscfg_page_pos0;
        if ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.'))
            nvme->nvme_geom.syscfg_cnt = nvme->nvme_geom.syscfg_page_pos0 + 0x10;
        else
            nvme->nvme_geom.syscfg_cnt = nvme->nvme_geom.syscfg_page_pos0 + 0x08;

    }
    if (((nvme->nvme_geom.syscfg_cnt==0)||(nvme->nvme_geom.syscfg_cnt >= 64))&&(nvme->nvme_geom.syscfg_page_pos0) && (nvme->nvme_geom.syscfg_page_pos1)) {
        nvme->nvme_geom.syscfg_page_pos = nvme->nvme_geom.syscfg_page_pos0;
        nvme->nvme_geom.syscfg_cnt = nvme->nvme_geom.syscfg_page_pos1;
    }

    //
    memset(p11_all_buf_ecc,0,sizeof(p11_all_buf_ecc));
    p11_all_buf_syscfg_offset_address=0;
    //
    printf("cnt: %04x ",nvme->nvme_geom.syscfg_cnt);
    printf("pos: %04x\n",nvme->nvme_geom.syscfg_page_pos);
    //printf(" read:\n");
    //
    for (i = 0; i < nvme->nvme_geom.syscfg_cnt; i++)
    {
    	nvme_command_h.cdw[0] = nvme->nvme_geom.syscfg_cmd[die][0];
    	nvme_command_h.cdw[1] = page_step;
    	nvme_command_h.cdw[2] = nvme->nvme_geom.syscfg_cmd[die][2];
    	nvme_command_h.cdw[3] = nvme->nvme_geom.syscfg_cmd[die][3] + i;
    	nvme_command_h.cdw[4] = nvme->nvme_geom.syscfg_cmd[die][4];
    	nvme_command_h.cdw[5] = nvme->nvme_geom.syscfg_cmd[die][5];
    	nvme_command_h.cdw[6] = 0;
    	nvme_command_h.cdw[7] = 0;
    	//for(k=0;k<6;k++) printf(" %08x",nvme_command_h.cdw[k]);printf("\n");
		for(k=0;k<5;k++)
		{
			ch1 = nvme_page_read((nvme_command_t*)&nvme_command_h);
			check1 = check_snm_16((unsigned char*)&nvme->page_data->page_buf[0],P11_page_step*0x1000);
			ch1 = nvme_page_read((nvme_command_t*)&nvme_command_h);
			check2 = check_snm_16((unsigned char*)&nvme->page_data->page_buf[0],P11_page_step*0x1000);
			//printf("k:%d check1:%4x check2:%4x\n",k,check1,check2);
			if(check1 == check2) break; //连续读两次，数据一样后退出
			if(ch1 != 0) return -103;
		}
		for(k=0;k<page_step;k++)  memcpy(&p11_all_buf[i*(page_step*0x1000)+k*0x1000],(unsigned char*)&nvme->page_data->page_buf[0x1000*k],0x1000);
		for(k=0;k<page_step;k++)  memcpy(&p11_all_buf_ecc[i*(page_step*0x10)+k*0x10],(unsigned char*)&nvme->page_data->page_ecc[k],0x10);
		//查找syscfg位置
		/*
    	dat1 = (*(uint32_t *)(&p11_all_buf[i*(page_step*0x1000)]));
    	dat2 = (*(uint32_t *)(&p11_all_buf_ecc[i*(page_step*0x10)]));
    	if((dat1 == 0x53436667)&&(dat2 == 0xffffff09))
    	{
    		if(p11_all_buf_syscfg_offset_address == 0) p11_all_buf_syscfg_offset_address = i*(page_step*0x1000);//获取第一个syscfg地址
    	}*/
    }
    p11_all_buf_syscfg_offset_address = nvme->nvme_geom.syscfg_page_pos*page_step*0x1000;
    //
    if(p11_all_buf_syscfg_offset_address == 0) return -104; //没有syscfg数据
    //
    return 0;
}
//-------------------------------------------------------------------------------------------------------
int update_syscfg_s(int n,uint32_t type,uint32_t dat_length)
{
	uint8_t TEST[4];
	uint32_t i,k,address,cntb;
	*(uint32_t *)(&TEST[0x00]) = type;

	for(i=0;i<0x20000;i++)
	{
		for(k=0;k<4;k++)
		{
			if(p11_all_buf[p11_all_buf_syscfg_offset_address+i+k] != TEST[k]) break;
		}
		if(k == 4)
		{
			cntb = *(uint32_t *)(&p11_all_buf[p11_all_buf_syscfg_offset_address+i+k-8]);
			if(cntb == 0x434E5442)//cntb
			{
				//length  = *(uint32_t *)(&p11_all_buf[p11_all_buf_syscfg_offset_address+i+k]);
				address = *(uint32_t *)(&p11_all_buf[p11_all_buf_syscfg_offset_address+i+k+4]);
				memcpy(&p11_all_buf[p11_all_buf_syscfg_offset_address+address],&syscfg_buf[n*0x100+0x10],dat_length);
				printf("u_16->ok\n");
				return 0;
			}
			else
			{
				memcpy(&p11_all_buf[p11_all_buf_syscfg_offset_address+i+k],&syscfg_buf[n*0x100+0x10],dat_length);
				printf("u->ok\n");
				return 0;
			}
		}
	}
	return -511;
}

int P11_update_syscfg_all(void)
{
	uint32_t n,numbers,type,dat_length;
	int ch1= -506;
	numbers = *(uint32_t *)(&syscfg_buf[0x00]);  //修改的项目总数
	if((numbers == 0)&&(numbers > 0x50)) return -501;
	//
	for(n=0;n<numbers;n++)
	{
		type = *(uint32_t *)(&syscfg_buf[n*0x100+0x08]);
		dat_length = *(uint32_t *)(&syscfg_buf[n*0x100+0x0c]);

		//if(dat_length <= 16) ch1 = update_syscfg_s_1(n,type,dat_length);
		//else                 ch1 = update_syscfg_s_2(n,type,dat_length);
		ch1 = update_syscfg_s(n,type,dat_length);

	}
	return ch1;

}
//--------------------------------------------------------- ----------------------------------------------
int P11_erase_syscfg_all(uint8_t erase_n)
{
    int dies = 0, die_wr = 0, die_er = -1,die;
    int ch1;
    unsigned int i,k, page_step;
    nvme_command_t nvme_command_h;
    //
    if ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) page_step = 2;
     else page_step = 4;
    //
    if (nvme->nvme_geom.syscfg_cmd[0][0] != 0xffffffff)	dies++;
    if (nvme->nvme_geom.syscfg_cmd[1][0] != 0xffffffff)	dies++;
    if (dies == 2) {
        if (nvme->nvme_geom.syscfg_cmd[0][0] != 0) { die_er = 1; die_wr = 0; }
        else if (nvme->nvme_geom.syscfg_cmd[1][0] != 0) { die_er = 0; die_wr = 1; }
    }
    else if (dies == 1) {
        if (nvme->nvme_geom.syscfg_cmd[0][0] != 0xffffffff) die_wr = 0;
        if (nvme->nvme_geom.syscfg_cmd[1][0] != 0xffffffff) die_wr = 1;
    }
    else if (dies == 0) {
        //硬盘数据错误!!!
        return -201;
    }
    //
    if(erase_n == 1) //第一次擦除需要写的区域
    {
    	die = die_wr;
    }
    else if(erase_n == 2)//第二次擦除备份区域
    {
    	if ((die_er != die_wr) && (die_er != -1))
    	{
    		die =  die_er;
    	}
    	else return 0; //没有备份区域就返回，不擦除了
    }
    //
    nvme_command_h.cdw[0] = nvme->nvme_geom.syscfg_cmd[die][0];
    nvme_command_h.cdw[1] = nvme->nvme_geom.syscfg_cmd[die][1];

    if ((nvme->nvme_geom.syscfg_cmd[die][0] >= 0x100)&&(nvme->nvme_geom.syscfg_cmd[die][3] == 0x0000) && (nvme->nvme_geom.Vendor == Hynix)&&(buf_get4no(&nvme->nvme_geom.MSP_Name[0])!='s5e.')) {
        nvme_command_h.cdw[2] = 0x20002;
        nvme_command_h.cdw[3] = 0x00;
        nvme_command_h.cdw[4] = 0x00;
        nvme_command_h.cdw[5] = 0x00;
    }
    else if ((nvme->nvme_geom.syscfg_cmd[die][0] == 0x00) && (nvme->nvme_geom.syscfg_cmd[die][3] == 0x60000) && (nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) {
        nvme_command_h.cdw[2] = 0x20000;
        nvme_command_h.cdw[3] = 0x00;
        nvme_command_h.cdw[4] = 0xffffffff;
        nvme_command_h.cdw[5] = 0x0000ffff;
    }
    else {
        nvme_command_h.cdw[2] = ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) ? 0x2000A : nvme->nvme_geom.syscfg_cmd[die][2];
        nvme_command_h.cdw[3] = ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) ? 0x20000 : nvme->nvme_geom.syscfg_cmd[die][3];
        nvme_command_h.cdw[4] = ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) ? 0xffffffff : nvme->nvme_geom.syscfg_cmd[die][4];
        nvme_command_h.cdw[5] = ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) ? 0xffff0000 : nvme->nvme_geom.syscfg_cmd[die][5];
    }
    if (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) == 's5e.') {
        nvme_command_h.cdw[5] = 0xffffffff;
    }
    nvme_command_h.cdw[6] = nvme_command_h.cdw[7] = 0x0;
    //
    //printf("erase:\n");
    //for(k=0;k<8;k++) printf(" %08x",nvme_command_h.cdw[k]);printf("\n");
    //
    ch1 = nvme_block_erase((nvme_command_t*)&nvme_command_h);
    if(ch1 != 0) return -202;
    //
	return 0;
}
//-------------------------------------------------------------------------------------------------------
int P11_write_syscfg_all(void)
{
    int dies = 0, die_wr = 0, die_er = -1,die;
    int ch1;
    unsigned int i,k, page_step;
    nvme_command_t nvme_command_h;
    //
    if ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) page_step = 2;
     else page_step = 4;
    //
    if (nvme->nvme_geom.syscfg_cmd[0][0] != 0xffffffff)	dies++;
    if (nvme->nvme_geom.syscfg_cmd[1][0] != 0xffffffff)	dies++;
    if (dies == 2) {
        if (nvme->nvme_geom.syscfg_cmd[0][0] != 0) { die_er = 1; die_wr = 0; }
        else if (nvme->nvme_geom.syscfg_cmd[1][0] != 0) { die_er = 0; die_wr = 1; }
    }
    else if (dies == 1) {
        if (nvme->nvme_geom.syscfg_cmd[0][0] != 0xffffffff) die_wr = 0;
        if (nvme->nvme_geom.syscfg_cmd[1][0] != 0xffffffff) die_wr = 1;
    }
    else if (dies == 0) {
        //硬盘数据错误!!!
        return -301;
    }
    //
    die = die_wr;
    //
    //printf("write:\n");
    for (i = 0; i < nvme->nvme_geom.syscfg_cnt; i++)
    {
		memcpy(&nvme_command_h.cdw[0], &nvme->nvme_geom.syscfg_cmd[die][0], 32);
		nvme_command_h.cdw[3] += i;
		if ((nvme->nvme_geom.Vendor == Hynix) && (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) != 's5e.')) {
			if (nvme->nvme_geom.syscfg_cmd[die][3] == 0x0000) {
				nvme_command_h.cdw[1] = 0x02;			nvme_command_h.cdw[2] = 0x20002;
				nvme_command_h.cdw[4] = 0xffff0000;		nvme_command_h.cdw[5] = 0xffffffff;
			}
			else {
				nvme_command_h.cdw[1] = 0x02;			nvme_command_h.cdw[2] = 0x2000A;
				nvme_command_h.cdw[4] = 0xffffffff;		nvme_command_h.cdw[5] = 0xffff0000;
			}
		}
		if (buf_get4no(&nvme->nvme_geom.MSP_Name[0]) == 's5e.') {
			nvme_command_h.cdw[5] = 0xffffffff;
		}
		//write
    	//for(k=0;k<8;k++) printf(" %08x",nvme_command_h.cdw[k]);printf("\n");
    	//ecc
		for(k=0;k<page_step;k++)
		{
			if((*(uint32_t *)(&p11_all_buf_ecc[i*(page_step*0x10)+k*0x10])) == 0xffffff09)
			{
				ch1 = ecc_vertify_page(&p11_all_buf[i*(page_step*0x1000)+k*0x1000],&p11_all_buf_ecc[i*(page_step*0x10)+k*0x10]);
			}
		}
	   	//设置数据
		for(k=0;k<page_step;k++)  memcpy((unsigned char*)&nvme->page_data->page_buf[0x1000*k],&p11_all_buf[i*(page_step*0x1000)+k*0x1000],0x1000);
		for(k=0;k<page_step;k++)  memcpy((unsigned char*)&nvme->page_data->page_ecc[k],&p11_all_buf_ecc[i*(page_step*0x10)+k*0x10],0x10);
		//
		ch1 = nvme_page_write((nvme_command_t*)&nvme_command_h);
		if(ch1 != 0)
		{
			printf("w->i:%04x ch1:%04x \n",i,ch1);
			mdelay(100);
			//return -302;
		}
    }
    return 0;
}
//-------------------------------------------------------------------------------------------------------
int P11_update_new_disk_information(void)
{
	int ch1;
	static int index2=0;
	if(index2 == 0){ 						//每次上电只需要引导一次
		ch1 = P11_reset();
		if(ch1 < 0) return -99;
		printf("reset1:->ok\n");
		ch1 = nvme_index2(0);
		printf("nvme_index2:->ok\n");
	}
	index2++;
	//
	memcpy(P7_INFO_BUF,(unsigned char*)&nvme->nvme_geom,sizeof(nvme_geom_t));//暂存在P7_INFO_BUF里面
	ch1 = P11_reset(); //再复位一次
	if(ch1 < 0) return -99;
	memcpy((unsigned char*)&nvme->nvme_geom,P7_INFO_BUF,sizeof(nvme_geom_t));//复位后还原
	printf("reset2:->ok\n");
	//read
	ch1 = P11_read_syscfg_all();
	if(ch1 < 0) return ch1;
	printf("read:->ok\n");

	ch1 = P11_update_syscfg_all();
	if(ch1 < 0) return ch1;
	printf("update:->ok\n");
	//
	//erase 1
	ch1 = P11_erase_syscfg_all(1); //第一次擦除需要写的区域
	if(ch1 < 0) return ch1;
	printf("erase1:->ok\n");
	//write
	ch1 =  P11_write_syscfg_all();
	if(ch1 < 0) return ch1;
	printf("write:->ok\n");
	//erase 2
	ch1 = P11_erase_syscfg_all(2); //第二次擦除备份区域
	if(ch1 < 0) return ch1;
	printf("erase2:->ok\n");
	printf("end\n");
	return 0;
}
//-------------------------------------------------------------------------------------------------------
int P7_update_new_disk_information(void)
{
	int r,ch1;
	unsigned short check1;
	printf("P7_update\n");
	//return 0;
	//E3NAND_INFO();
	p11_all_buf_syscfg_offset_address = 0;
	for(r=0;r<0x20000;r+=0x1000)
	{
		ch1 = lookup_image_in_bdev_hzq(3,&p11_all_buf[r],r/NVME_BLOCK_SIZE,1);
	}
	check1 = check_snm_16(p11_all_buf,0x20000);
	printf("ch1:%d check1:%4x \n",ch1,check1);
	if(ch1 < 0) return ch1;
	//
	ch1 = P11_update_syscfg_all();
	if(ch1 < 0) return ch1;
	printf("update:->ok\n");
	//
	memcpy(syscfg_buf,p11_all_buf,0x20000);
	//
	ch1 = syscfg_write_in_bdev("nvme_syscfg0", 1);
	if(ch1 < 0) return ch1;
	printf("write:->ok\n");
	printf("end\n");

	return 0;
}
//-------------------------------------------------------------------------------------------------------
